#include "gnocchi/pose_graph.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

#include <algorithm>
#include <cmath>

using gtsam::symbol_shorthand::X;

namespace gnocchi
{

PoseGraph::PoseGraph(const Options& options) : options_(options)
{
    gtsam::ISAM2Params params;
    params.setRelinearizeThreshold(0.1);
    params.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(params);
    smoother_ = gtsam::IncrementalFixedLagSmoother(options_.lag_time, params);
    world_from_odom_ = gtsam::Pose3();
}

bool PoseGraph::fresh(const Measurement& m, int64_t stamp_ns) const
{
    if (!m.valid) return false;
    const double age = std::abs(static_cast<double>(stamp_ns - m.stamp_ns)) * 1e-9;
    return age <= options_.max_measurement_age;
}

void PoseGraph::setGnss(int64_t stamp_ns, const Eigen::Vector3d& enu, double sigma)
{
    if (!enu.allFinite() || !std::isfinite(sigma) || sigma > options_.gnss_max_sigma) return;
    gnss_ = enu;
    gnss_meta_ = {stamp_ns, sigma, true};
}

void PoseGraph::setHeading(int64_t stamp_ns, double yaw_enu, double sigma)
{
    if (!std::isfinite(yaw_enu) || !std::isfinite(sigma) || sigma <= 0.0) return;
    heading_ = yaw_enu;
    heading_meta_ = {stamp_ns, sigma, true};
}

void PoseGraph::addAttitudeFactor(size_t key, const Eigen::Isometry3d& odom)
{
    if (options_.odom_attitude_sigma <= 0.0) return;
    // Gravity-direction factor: genuinely 2-DOF and yaw invariant, so it cannot
    // fight the heading prior.
    const Eigen::Vector3d body_up_in_world = odom.rotation() * Eigen::Vector3d::UnitZ();
    graph_.add(gtsam::Pose3AttitudeFactor(
        X(key), gtsam::Unit3(body_up_in_world),
        gtsam::noiseModel::Isotropic::Sigma(2, options_.odom_attitude_sigma),
        gtsam::Unit3(Eigen::Vector3d::UnitZ())));
}

void PoseGraph::addVerticalFactor(size_t key, const Eigen::Isometry3d& odom)
{
    if (options_.odom_z_sigma <= 0.0) return;
    // Vertical only: huge horizontal sigmas leave x and y entirely to GNSS.
    graph_.add(gtsam::GPSFactor(
        X(key), gtsam::Point3(0.0, 0.0, odom.translation().z() + z_offset_),
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(1e3, 1e3, options_.odom_z_sigma))));
}

double PoseGraph::yawOf(const Eigen::Matrix3d& r)
{
    // Standard ZYX yaw, safe for any attitude, unlike an Euler decomposition.
    return std::atan2(r(1, 0), r(0, 0));
}

gtsam::SharedNoiseModel PoseGraph::gnssNoise(double sigma) const
{
    const double s = sigma > 0.0 ? std::max(sigma, options_.gnss_sigma) : options_.gnss_sigma;
    // A very large sigma is how altitude is ignored: the factor stays 3-DOF so
    // no new factor type is needed, but z carries no weight.
    const double sz = options_.gnss_fuse_altitude ? options_.gnss_sigma_z : 1e3;
    gtsam::SharedNoiseModel base =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(s, s, sz));
    if (options_.gnss_robust_k <= 0.0) return base;
    // A bad fix bends the trajectory rather than yanking it.
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(options_.gnss_robust_k), base);
}

gtsam::SharedNoiseModel PoseGraph::betweenNoise(const Eigen::Isometry3d& delta) const
{
    const double distance = delta.translation().norm();
    const double angle = Eigen::AngleAxisd(delta.rotation()).angle();
    const double t = std::max(options_.odom_trans_floor, options_.odom_trans_sigma_per_m * distance);
    const double r = std::max(options_.odom_rot_floor,
                              options_.odom_rot_sigma_per_m * distance +
                                  options_.odom_rot_sigma_per_rad * angle);
    return gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << r, r, r, t, t, t).finished());
}

std::string PoseGraph::initBlockedReason(int64_t stamp_ns) const
{
    if (initialized_) return {};
    auto describe = [&](const Measurement& m, const char* what) -> std::string {
        if (!m.valid) return std::string("no ") + what + " received yet";
        const double age = std::abs(static_cast<double>(stamp_ns - m.stamp_ns)) * 1e-9;
        if (age <= options_.max_measurement_age) return {};
        std::string s = std::string(what) + " is " + std::to_string((int)age) +
                        " s away from the odometry stamp";
        // A gap this large is a clock mismatch, not a late message.
        if (age > 60.0) s += " -- clocks disagree; if replaying a bag, "
                             "launch with use_sim_time:=true and play with --clock";
        return s;
    };
    std::string g = describe(gnss_meta_, "GNSS fix");
    std::string h = describe(heading_meta_, "GNSS heading (gpsstatus/gps1/raw)");
    if (g.empty() && h.empty()) return {};
    if (g.empty()) return h;
    if (h.empty()) return g;
    return g + "; " + h;
}

bool PoseGraph::tryInitialize(int64_t stamp_ns, const Eigen::Isometry3d& odom)
{
    // Anchor in ENU from the measurements. Anchoring on the odometry frame would
    // leave the graph rotated by the start heading, which no position prior undoes.
    if (!fresh(gnss_meta_, stamp_ns) || !fresh(heading_meta_, stamp_ns)) return false;

    // Rotating the odometry attitude by (heading - its yaw) sets yaw and preserves
    // roll/pitch, with no Euler decomposition to get wrong.
    const double yaw_correction = heading_ - yawOf(odom.rotation());
    const gtsam::Rot3 rotation =
        gtsam::Rot3::Rz(yaw_correction) * gtsam::Rot3(odom.rotation());
    // z comes from the odometry, not the fix.
    const gtsam::Pose3 pose(
        rotation, gtsam::Point3(gnss_.x(), gnss_.y(), odom.translation().z()));

    values_.insert(X(next_key_), pose);
    stamps_[X(next_key_)] = static_cast<double>(stamp_ns) * 1e-9;
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(next_key_), pose,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << options_.odom_attitude_sigma, options_.odom_attitude_sigma,
                                 heading_meta_.sigma, options_.gnss_sigma, options_.gnss_sigma,
                                 options_.origin_z_sigma)
                .finished())));

    z_offset_ = pose.translation().z() - odom.translation().z();

    keyframe_odom_ = odom;
    accumulated_ = Eigen::Isometry3d::Identity();
    accumulated_distance_ = 0.0;
    accumulated_angle_ = 0.0;
    last_keyframe_ns_ = stamp_ns;

    state_.pose = pose;
    state_.stamp_ns = stamp_ns;
    world_from_odom_ = pose * gtsam::Pose3(odom.matrix()).inverse();

    next_key_++;
    keyframes_ = 1;
    initialized_ = true;
    last_gnss_attached_ns_ = stamp_ns;
    last_heading_attached_ns_ = stamp_ns;
    gnss_meta_.valid = false;
    heading_meta_.valid = false;
    gnss_attached_ = 1;
    heading_attached_ = 1;
    optimize(stamp_ns);
    return true;
}

void PoseGraph::createKeyframe(int64_t stamp_ns, const Eigen::Isometry3d& odom)
{
    const gtsam::Pose3 delta(accumulated_.matrix());
    const size_t key = next_key_;

    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key - 1), X(key), delta,
                                                  betweenNoise(accumulated_)));
    // Seed from the previous estimate walked along the measured increment.
    values_.insert(X(key), state_.pose * delta);
    stamps_[X(key)] = static_cast<double>(stamp_ns) * 1e-9;

    // Attached here, as the keyframe is built, so a node cannot be optimized before
    // its own constraints exist.
    const auto due = [&](int64_t last, double interval) {
        return last == 0 || interval <= 0.0 ||
               static_cast<double>(stamp_ns - last) * 1e-9 >= interval;
    };
    if (fresh(gnss_meta_, stamp_ns) && due(last_gnss_attached_ns_, options_.gnss_min_interval))
    {
        graph_.add(gtsam::GPSFactor(X(key), gnss_, gnssNoise(gnss_meta_.sigma)));
        gnss_meta_.valid = false;
        gnss_attached_++;
        last_gnss_attached_ns_ = stamp_ns;
    }
    if (fresh(heading_meta_, stamp_ns) &&
        due(last_heading_attached_ns_, options_.heading_min_interval))
    {
        // Yaw only: roll and pitch get a free sigma so this constrains one axis.
        graph_.addExpressionFactor(
            gtsam::rotation(X(key)), gtsam::Rot3::Ypr(heading_, 0.0, 0.0),
            gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3(1e2, 1e2, heading_meta_.sigma)));
        heading_meta_.valid = false;
        heading_attached_++;
        last_heading_attached_ns_ = stamp_ns;
    }
    addAttitudeFactor(key, odom);
    addVerticalFactor(key, odom);

    keyframe_odom_ = odom;
    accumulated_ = Eigen::Isometry3d::Identity();
    accumulated_distance_ = 0.0;
    accumulated_angle_ = 0.0;
    last_keyframe_ns_ = stamp_ns;
    next_key_++;
    keyframes_++;

    optimize(stamp_ns);
}

void PoseGraph::optimize(int64_t stamp_ns)
{
    const size_t key = next_key_ - 1;
    try
    {
        gtsam::Values result;
        if (options_.use_fixed_lag)
        {
            smoother_.update(graph_, values_, stamps_);
            result = smoother_.calculateEstimate();
            state_.covariance = smoother_.marginalCovariance(X(key));
        }
        else
        {
            isam_.update(graph_, values_);
            result = isam_.calculateEstimate();
            state_.covariance = isam_.marginalCovariance(X(key));
        }
        state_.pose = result.at<gtsam::Pose3>(X(key));
        state_.stamp_ns = stamp_ns;
        state_.keyframes = keyframes_;
        state_.valid = true;
        // Promote the correction together with the state it belongs to, so a
        // reader can never pair a new anchor with an older pose.
        world_from_odom_ = state_.pose * gtsam::Pose3(keyframe_odom_.matrix()).inverse();
    }
    catch (const std::exception&)
    {
        // Keep the previous state; the next keyframe retries.
        state_.valid = keyframes_ > 1;
    }
    graph_.resize(0);
    values_.clear();
    stamps_.clear();
}

bool PoseGraph::addOdometry(int64_t stamp_ns, const Eigen::Isometry3d& odom)
{
    if (!odom.matrix().allFinite()) return false;

    if (!have_odom_)
    {
        last_odom_ = odom;
        have_odom_ = true;
        return false;
    }

    const Eigen::Isometry3d delta = last_odom_.inverse() * odom;
    last_odom_ = odom;

    if (!initialized_)
    {
        keyframe_odom_ = odom;
        return tryInitialize(stamp_ns, odom);
    }

    accumulated_ = accumulated_ * delta;
    accumulated_distance_ += delta.translation().norm();
    accumulated_angle_ += Eigen::AngleAxisd(delta.rotation()).angle();

    const double elapsed = static_cast<double>(stamp_ns - last_keyframe_ns_) * 1e-9;
    const bool moved = accumulated_distance_ >= options_.keyframe_distance ||
                       accumulated_angle_ >= options_.keyframe_angle;
    const bool waited = options_.keyframe_max_time > 0.0 && elapsed >= options_.keyframe_max_time;
    if (elapsed <= 0.0 || (!moved && !waited)) return false;

    createKeyframe(stamp_ns, odom);
    return true;
}

gtsam::Pose3 PoseGraph::poseFor(const Eigen::Isometry3d& odom) const
{
    return world_from_odom_ * gtsam::Pose3(odom.matrix());
}

}  // namespace gnocchi
