// Pose graph: lidar-odometry relative constraints with GNSS position and heading
// priors. Poses only, no velocity or bias states.
//
// Odometry is the clock. It alone creates keyframes, and measurements are attached
// when the keyframe is created, so no node is optimized without its constraints.
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <string>
#include <optional>

namespace gnocchi
{

struct Options
{
    // Keyframes are created on motion, not on message arrival, so a long
    // mission stays a graph the optimizer can still push a correction through.
    double keyframe_distance = 0.5;   // m
    double keyframe_angle = 0.175;    // rad
    double keyframe_max_time = 1.0;   // s

    // Between-factor sigmas scale with the increment; a flat per-node sigma makes
    // accumulated odometry error uncorrectable.
    double odom_trans_floor = 0.02;          // m
    double odom_rot_floor = 0.1;             // rad
    double odom_trans_sigma_per_m = 0.02;
    double odom_rot_sigma_per_m = 0.005;     // rad per m
    double odom_rot_sigma_per_rad = 0.02;    // rad per rad

    // An estimator posterior reports a covariance that is not a measurement noise,
    // so gnss_sigma is the honest size of one sample, not what the message says.
    double gnss_sigma = 0.5;         // m
    double gnss_max_sigma = 2.0;     // reject a fix worse than this
    double gnss_robust_k = 1.345;    // Huber, whitened units; 0 disables


    // Odometry is gravity aligned, so its roll/pitch is an absolute attitude
    // reference. Yaw is left to the heading prior.
    double odom_attitude_sigma = 0.05;  // rad

    // Gravity alignment also makes the odometry's z a vertical measurement, up to a
    // constant offset fixed at the origin.
    double odom_z_sigma = 0.1;  // m, 0 disables

    // GNSS altitude is the weakest axis of the fix; the odometry's z is better.
    bool gnss_fuse_altitude = false;
    double gnss_sigma_z = 3.0;   // used only when gnss_fuse_altitude is true
    double origin_z_sigma = 0.5; // m

    // A cached measurement older than this is not attached to a new keyframe.
    double max_measurement_age = 0.5;  // s

    // Minimum interval between attached priors. GNSS position error is correlated
    // over tens of seconds so it is spaced out; dual-antenna heading is not.
    double gnss_min_interval = 0.0;
    double heading_min_interval = 0.0;

    // The heading is reported with a transport lag, so its stamp is later than the
    // epoch it describes; this is added to the stamp. Measure it per airframe: on
    // the UGV the residual against odometry correlates with yaw rate until roughly
    // -0.24 s, and mis-association costs yaw_rate * lag, which dwarfs the sigma
    // during a turn.
    double heading_time_offset = 0.0;   // s
    // Huber on the heading prior, whitened. A moving-baseline heading throws
    // occasional large outliers that a bare Gaussian has to absorb by bending yaw.
    double heading_robust_k = 0.0;      // 0 disables
    // Widest gap between the samples bracketing a keyframe still worth interpolating.
    double heading_max_gap = 1.0;       // s
    // Skip the prior when the heading is slewing faster than this: whatever lag
    // correction is left over costs yaw_rate * error, so a mid-turn prior is the
    // least trustworthy one. 0 disables the gate.
    double heading_max_rate = 0.0;      // rad/s

    bool use_fixed_lag = false;
    double lag_time = 30.0;           // s
    size_t min_keyframes = 5;         // optimize this many before publishing
};

struct State
{
    bool valid = false;
    gtsam::Pose3 pose;                                  // local ENU
    Eigen::Matrix<double, 6, 6> covariance =            // GTSAM [rot, trans]
        Eigen::Matrix<double, 6, 6>::Zero();
    int64_t stamp_ns = 0;
    size_t keyframes = 0;
};

class PoseGraph
{
public:
    explicit PoseGraph(const Options& options);

    // Cache the newest measurements. These never touch the graph; they are
    // consumed when the next keyframe is created.
    void setGnss(int64_t stamp_ns, const Eigen::Vector3d& enu, double sigma);
    void setHeading(int64_t stamp_ns, double yaw_enu, double sigma);

    // The clock. Returns true when this reading created a keyframe.
    bool addOdometry(int64_t stamp_ns, const Eigen::Isometry3d& odom);

    bool initialized() const { return initialized_; }
    // Why initialisation has not happened yet, for logging. Empty once running.
    std::string initBlockedReason(int64_t stamp_ns) const;
    bool publishable() const { return initialized_ && keyframes_ >= options_.min_keyframes; }
    const State& state() const { return state_; }
    // How many keyframes actually received each measurement. If these lag the
    // keyframe count, the inputs are stale or the age limit is too tight.
    size_t gnssAttached() const { return gnss_attached_; }
    size_t headingAttached() const { return heading_attached_; }
    size_t keyframes() const { return keyframes_; }

    // world <- odom. Deliberately unfiltered: this is a rigid transform, so slewing
    // its rotation scales by the lever arm and manufactures output steps.
    gtsam::Pose3 worldFromOdom() const { return world_from_odom_; }
    gtsam::Pose3 poseFor(const Eigen::Isometry3d& odom) const;
    // Fallback sigma for a fix that reports no covariance of its own.
    double stateSigmaFallback() const { return options_.gnss_sigma; }

private:
    struct Measurement
    {
        int64_t stamp_ns = 0;
        double sigma = 0.0;
        bool valid = false;
    };
    int64_t last_gnss_attached_ns_ = 0;
    int64_t last_heading_attached_ns_ = 0;

    bool tryInitialize(int64_t stamp_ns, const Eigen::Isometry3d& odom);
    void createKeyframe(int64_t stamp_ns, const Eigen::Isometry3d& odom);
    void optimize(int64_t stamp_ns);
    // Attaches each buffered keyframe's heading once samples bracketing its time
    // exist, so the prior lands on the pose it actually describes.
    void attachHeadings();
    bool due(int64_t last, double interval, int64_t stamp_ns) const;
    gtsam::SharedNoiseModel gnssNoise(double sigma) const;
    gtsam::SharedNoiseModel headingNoise(double sigma) const;
    void addAttitudeFactor(size_t key, const Eigen::Isometry3d& odom);
    static double yawOf(const Eigen::Matrix3d& r);
    void addVerticalFactor(size_t key, const Eigen::Isometry3d& odom);
    gtsam::SharedNoiseModel betweenNoise(const Eigen::Isometry3d& delta) const;
    bool fresh(const Measurement& m, int64_t stamp_ns) const;

    Options options_;

    gtsam::ExpressionFactorGraph graph_;
    gtsam::Values values_;
    gtsam::ISAM2 isam_;
    gtsam::IncrementalFixedLagSmoother smoother_;
    gtsam::FixedLagSmoother::KeyTimestampMap stamps_;

    size_t next_key_ = 0;
    size_t keyframes_ = 0;
    size_t gnss_attached_ = 0;
    size_t heading_attached_ = 0;
    bool initialized_ = false;

    State state_;
    gtsam::Pose3 world_from_odom_;

    // Odometry bookkeeping.
    Eigen::Isometry3d last_odom_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d keyframe_odom_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d accumulated_ = Eigen::Isometry3d::Identity();
    double accumulated_distance_ = 0.0;
    double accumulated_angle_ = 0.0;
    int64_t last_keyframe_ns_ = 0;
    bool have_odom_ = false;
    // world_z - odom_z, fixed when the origin is set.
    double z_offset_ = 0.0;

    // Newest measurements, awaiting the next keyframe.
    Eigen::Vector3d gnss_ = Eigen::Vector3d::Zero();
    Measurement gnss_meta_;
    double heading_ = 0.0;
    Measurement heading_meta_;

    // Heading is buffered rather than consumed newest-first: attaching whatever
    // arrived last dates the prior by the transport lag plus the keyframe spacing,
    // which is an error proportional to yaw rate.
    struct HeadingSample
    {
        int64_t stamp_ns = 0;
        double yaw = 0.0;
        double sigma = 0.0;
    };
    struct PendingKeyframe
    {
        size_t key = 0;
        int64_t stamp_ns = 0;
    };
    std::deque<HeadingSample> heading_buf_;
    std::deque<PendingKeyframe> pending_kf_;
};

}  // namespace gnocchi
