#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>

namespace gnocchi {
struct LioHealthOptions {
  bool enabled = false;
  double max_speed = 5.0; // m/s, LIO plausibility for this platform
  double gps_max_speed = 5.0;
  double disagreement = 3.0; // metres of incremental discrepancy over a window
  double window = 2.0;
  double confirmation = 0.5; // seconds of sustained disagreement
  double recovery =
      30.0; // seconds of consistent motion before trusting LIO again
  double degraded_rotation_sigma = 0.5;
  double degraded_sigma =
      10.0; // minimum translation sigma per failed increment
};
// Uses increments, not absolute innovations: an offset GPS fix must not by
// itself classify LIO as faulty. GPS discontinuities reset the evidence window.
class LioHealth {
public:
  explicit LioHealth(const LioHealthOptions &o) : o_(o) {
    for (double v : {o.max_speed, o.gps_max_speed, o.disagreement, o.window,
                     o.confirmation, o.recovery, o.degraded_sigma,
                     o.degraded_rotation_sigma})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument(
            "lio_health parameters must be finite and positive");
    if (o.window < .5)
      throw std::invalid_argument(
          "lio_health.window must be at least 0.5 seconds");
  }
  void fix(int64_t t, const Eigen::Vector3d &p) {
    if (!o_.enabled || (!pending_.empty() && t <= pending_.back().t))
      return;
    pending_.push_back({t, p});
    while (pending_.size() > 256)
      pending_.pop_front();
  }
  void odom(int64_t t, const Eigen::Isometry3d &p,
            const Eigen::Matrix3d &world_rotation) {
    if (!o_.enabled)
      return;
    if (!odoms_.empty() && t <= odoms_.back().t)
      return;
    odoms_.push_back({t, p.translation()});
    while (odoms_.size() > 1 && (t - odoms_[1].t) * 1e-9 > o_.window + 2)
      odoms_.pop_front();
    while (!pending_.empty() && pending_.front().t <= t) {
      const auto f = pending_.front();
      pending_.pop_front();
      std::optional<Eigen::Vector3d> local;
      for (size_t i = 1; i < odoms_.size(); ++i) {
        const auto &a = odoms_[i - 1];
        const auto &b = odoms_[i];
        if (a.t <= f.t && f.t <= b.t && (b.t - a.t) * 1e-9 <= .5) {
          const double w = double(f.t - a.t) / double(b.t - a.t);
          local = (1 - w) * a.p + w * b.p;
          break;
        }
      }
      if (!local)
        continue;
      if (!pairs_.empty()) {
        const double dt = (f.t - pairs_.back().t) * 1e-9;
        if (dt <= 0 || dt < .15)
          continue;
        if (dt > 1 || (f.p - pairs_.back().gps).head<2>().norm() / dt >
                          o_.gps_max_speed) {
          pairs_.clear();
          bad_since_.reset();
          good_since_.reset();
          last_validated_ = 0;
        }
      }
      pairs_.push_back({f.t, f.p, *local});
      while (pairs_.size() > 1 && (f.t - pairs_[1].t) * 1e-9 > o_.window)
        pairs_.pop_front();
      if (pairs_.size() < 3)
        continue;
      const auto &a = pairs_.front();
      const auto &b = pairs_.back();
      const double dt = (b.t - a.t) * 1e-9;
      if (dt < .5)
        continue;
      last_validated_ = f.t;
      const Eigen::Vector3d delta = world_rotation * (b.local - a.local);
      discrepancy_ = (delta.head<2>() - (b.gps - a.gps).head<2>()).norm();
      const bool bad = discrepancy_ > o_.disagreement ||
                       delta.head<2>().norm() / dt > o_.max_speed;
      if (bad) {
        good_since_.reset();
        if (!bad_since_)
          bad_since_ = f.t;
        if ((f.t - *bad_since_) * 1e-9 >= o_.confirmation) {
          degraded_ = true;
          ever_degraded_ = true;
        }
      } else {
        bad_since_.reset();
        if (!good_since_)
          good_since_ = f.t;
        if ((f.t - *good_since_) * 1e-9 >= o_.recovery)
          degraded_ = false;
      }
    }
    reliable_gps_ = last_validated_ && (t - last_validated_) * 1e-9 <= 1.0;
  }
  bool degraded() const { return o_.enabled && degraded_; }
  bool everDegraded() const { return ever_degraded_; }
  bool reliableGps() const { return reliable_gps_; }
  double discrepancy() const { return discrepancy_; }

private:
  struct Sample {
    int64_t t;
    Eigen::Vector3d p;
  };
  struct Pair {
    int64_t t;
    Eigen::Vector3d gps, local;
  };
  LioHealthOptions o_;
  std::deque<Sample> odoms_, pending_;
  std::deque<Pair> pairs_;
  std::optional<int64_t> bad_since_, good_since_;
  int64_t last_validated_ = 0;
  bool degraded_ = false, reliable_gps_ = false, ever_degraded_ = false;
  double discrepancy_ = 0;
};
} // namespace gnocchi
