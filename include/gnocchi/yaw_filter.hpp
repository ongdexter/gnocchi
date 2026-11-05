// Yaw and gyro-bias Kalman filter that carries heading through gaps.
//
// Runs all the time so its bias estimate has converged by the moment it is
// needed: lidar odometry stops, and the GNSS-only fallback has to keep yaw
// through dual-antenna dropouts (tens of seconds under trees in the field bags).
// Measurements may be stamped in the past (the dual-antenna heading's transport
// lag); they are compared against the yaw the filter held at that epoch.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

#include <Eigen/Dense>

namespace gnocchi
{

struct YawFilterOptions
{
    double gyro_noise = 0.05;        // rad/s/sqrt(Hz): yaw random walk while propagating
    double bias_random_walk = 1e-6;  // (rad/s)^2 per s
    double max_bias = 2.0 * M_PI / 180.0;  // rad/s; FCU gyros measured within 0.3 deg/s
    double initial_bias_sigma = 0.01;      // rad/s
    // Innovation gate (sigmas, plus a fixed margin) and how many consecutive
    // rejected measurements make the filter re-lock onto the measurement.
    double gate_sigmas = 3.0;
    double gate_margin = 10.0 * M_PI / 180.0;
    int relock_after = 10;
    // Without gyro samples the yaw is only held, never propagated, for this long.
    double hold_without_gyro = 1.0;  // s
    // A gyro gap longer than this is not integrated across.
    double max_gyro_gap = 0.2;  // s
};

class YawFilter
{
public:
    explicit YawFilter(const YawFilterOptions& o = {}) : o_(o) {}

    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    bool initialized() const { return initialized_; }
    double yaw() const { return x_(0); }
    double bias() const { return x_(1); }
    double yawSigma() const { return std::sqrt(std::max(P_(0, 0), 0.0)); }
    int64_t lastMeasurementNs() const { return last_meas_ns_; }
    int64_t lastGyroNs() const { return last_gyro_ns_; }

    // Seconds since the last accepted absolute yaw measurement.
    double measurementAge(int64_t now_ns) const
    {
        return last_meas_ns_ ? static_cast<double>(now_ns - last_meas_ns_) * 1e-9 : 1e9;
    }

    // The yaw is usable at now_ns: measured recently enough, and propagated by the
    // gyro since if the measurement is older than hold_without_gyro.
    bool valid(int64_t now_ns, double max_age) const
    {
        if (!initialized_) return false;
        const double age = measurementAge(now_ns);
        if (age > max_age) return false;
        const bool gyro_live = last_gyro_ns_ &&
                               static_cast<double>(now_ns - last_gyro_ns_) * 1e-9 <= 2.0 * o_.max_gyro_gap;
        return gyro_live || age <= o_.hold_without_gyro;
    }

    // Body z angular rate (FLU), rad/s.
    void gyro(int64_t stamp_ns, double wz)
    {
        if (!std::isfinite(wz)) return;
        if (initialized_ && last_gyro_ns_ && stamp_ns > last_gyro_ns_)
        {
            const double dt = static_cast<double>(stamp_ns - last_gyro_ns_) * 1e-9;
            if (dt <= o_.max_gyro_gap)
            {
                x_(0) = wrap(x_(0) + (wz - x_(1)) * dt);
                Eigen::Matrix2d F;
                F << 1.0, -dt, 0.0, 1.0;
                P_ = F * P_ * F.transpose();
                P_(0, 0) += o_.gyro_noise * o_.gyro_noise * dt;
                P_(1, 1) += o_.bias_random_walk * dt;
            }
            pushHistory(stamp_ns);
        }
        if (stamp_ns > last_gyro_ns_) last_gyro_ns_ = stamp_ns;
    }

    // Absolute yaw (ENU, rad) describing the epoch stamp_ns. Returns false if gated out.
    bool measure(int64_t stamp_ns, double yaw, double sigma)
    {
        if (!std::isfinite(yaw) || !std::isfinite(sigma) || sigma <= 0.0) return false;
        const double R = sigma * sigma;
        if (!initialized_)
        {
            x_ << wrap(yaw), 0.0;
            P_.setZero();
            P_(0, 0) = R;
            P_(1, 1) = o_.initial_bias_sigma * o_.initial_bias_sigma;
            initialized_ = true;
            last_meas_ns_ = stamp_ns;
            history_.clear();
            pushHistory(std::max(stamp_ns, last_gyro_ns_));
            return true;
        }
        // Without gyro the yaw has not moved since the last measurement, so the
        // held value is only as good as its age allows.
        if (!last_gyro_ns_ || static_cast<double>(stamp_ns - last_gyro_ns_) * 1e-9 > 2.0 * o_.max_gyro_gap)
        {
            x_(0) = wrap(yaw);
            P_(0, 0) = R;
            last_meas_ns_ = std::max(last_meas_ns_, stamp_ns);
            history_.clear();
            pushHistory(stamp_ns);
            rejects_ = 0;
            return true;
        }
        const double innovation = wrap(yaw - yawAt(stamp_ns));
        const double S = P_(0, 0) + R;
        if (std::abs(innovation) > o_.gate_sigmas * std::sqrt(S) + o_.gate_margin)
        {
            if (++rejects_ < o_.relock_after) return false;
            // Persistent disagreement: the absolute measurement wins.
            x_(0) = wrap(x_(0) + innovation);
            P_(0, 0) = R;
            P_(0, 1) = P_(1, 0) = 0.0;
            shiftHistory(innovation);
            rejects_ = 0;
            last_meas_ns_ = std::max(last_meas_ns_, stamp_ns);
            return true;
        }
        rejects_ = 0;
        const Eigen::Vector2d K = P_.col(0) / S;
        x_ += K * innovation;
        x_(0) = wrap(x_(0));
        x_(1) = std::clamp(x_(1), -o_.max_bias, o_.max_bias);
        // Joseph form keeps P symmetric positive definite.
        Eigen::Matrix2d IKH = Eigen::Matrix2d::Identity();
        IKH.col(0) -= K;
        P_ = IKH * P_ * IKH.transpose() + K * K.transpose() * R;
        shiftHistory(K(0) * innovation);
        last_meas_ns_ = std::max(last_meas_ns_, stamp_ns);
        return true;
    }

    // The filter's yaw at a past (or present) epoch, from its history.
    double yawAt(int64_t stamp_ns) const
    {
        if (history_.empty()) return x_(0);
        if (stamp_ns >= history_.back().first) return x_(0);
        if (stamp_ns <= history_.front().first) return history_.front().second;
        auto it = std::lower_bound(history_.begin(), history_.end(), stamp_ns,
                                   [](const auto& h, int64_t t) { return h.first < t; });
        const auto& b = *it;
        const auto& a = *(it - 1);
        const double w = static_cast<double>(stamp_ns - a.first) / static_cast<double>(b.first - a.first);
        return wrap(a.second + w * wrap(b.second - a.second));
    }

private:
    void pushHistory(int64_t stamp_ns)
    {
        if (!history_.empty() && stamp_ns <= history_.back().first) return;
        history_.emplace_back(stamp_ns, x_(0));
        // A second is enough to cover the heading's transport lag.
        while (history_.size() > 2 && static_cast<double>(stamp_ns - history_.front().first) * 1e-9 > 1.0)
            history_.pop_front();
    }

    void shiftHistory(double delta)
    {
        for (auto& h : history_) h.second = wrap(h.second + delta);
    }

    YawFilterOptions o_;
    bool initialized_ = false;
    Eigen::Vector2d x_ = Eigen::Vector2d::Zero();
    Eigen::Matrix2d P_ = Eigen::Matrix2d::Zero();
    int64_t last_gyro_ns_ = 0;
    int64_t last_meas_ns_ = 0;
    int rejects_ = 0;
    std::deque<std::pair<int64_t, double>> history_;
};

}  // namespace gnocchi
