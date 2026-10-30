#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace gnocchi {
struct GpsHeightOptions {
  bool enabled = false;
  double sigma_floor = 3.0;
  double max_reported_sigma = 2.0;
  double max_speed = 2.0;
  double reference_window = 30.0;
  double reference_delay =
      3.0; // exclude the LIO failure's confirmation interval
  // Fade the initial GPS/estimate height mismatch over this many seconds.
  // Zero retains immediate height recovery. This is not an output rate limit.
  double recovery_ramp_time = 5.0;
};
class GpsHeight {
public:
  struct Measurement {
    double z, sigma;
  };
  explicit GpsHeight(const GpsHeightOptions &o) : o_(o) {
    for (double v : {o.sigma_floor, o.max_reported_sigma, o.max_speed,
                     o.reference_window, o.reference_delay})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument(
            "gps_height parameters must be positive and finite");
    if (o.reference_window <= o.reference_delay)
      throw std::invalid_argument(
          "gps_height reference window must exceed delay");
    if (!std::isfinite(o.recovery_ramp_time) || o.recovery_ramp_time < 0)
      throw std::invalid_argument(
          "gps_height recovery ramp time must be finite and nonnegative");
  }
  void observe(int64_t t, double z, double sigma, double healthy_pose_z,
               bool failed) {
    if (!o_.enabled || t <= last_seen_)
      return;
    last_seen_ = t;
    if (failed && !frozen_) {
      std::vector<double> offsets;
      for (const auto &r : references_)
        if ((t - r.t) * 1e-9 >= o_.reference_delay)
          offsets.push_back(r.offset);
      if (offsets.size() >= 5) {
        std::sort(offsets.begin(), offsets.end());
        offset_ = offsets[offsets.size() / 2];
        frozen_ = true;
      }
    }
    if (!std::isfinite(z) || !std::isfinite(sigma) || sigma <= 0 ||
        sigma > o_.max_reported_sigma) {
      stable_.clear();
      resetRecovery();
      return;
    }
    if (!stable_.empty()) {
      const auto &last = stable_.back();
      double dt = (t - last.t) * 1e-9;
      if (dt <= 0 || dt < .15)
        return;
      if (dt > 1 || std::abs(z - last.z) > o_.max_speed * dt) {
        stable_.clear();
        resetRecovery();
      }
    }
    stable_.push_back({t, z, sigma});
    while (stable_.size() > 1 && (t - stable_[1].t) * 1e-9 > 1.0)
      stable_.pop_front();
    if (!failed && stable_.size() >= 3 &&
        (t - stable_.front().t) * 1e-9 >= .5 && std::isfinite(healthy_pose_z)) {
      references_.push_back({t, healthy_pose_z - z});
    }
    while (!references_.empty() &&
           (t - references_.front().t) * 1e-9 > o_.reference_window)
      references_.pop_front();
  }
  std::optional<Measurement> measurement(int64_t now) const {
    if (!frozen_ || stable_.size() < 3 ||
        (stable_.back().t - stable_.front().t) * 1e-9 < .5 ||
        (now - stable_.back().t) * 1e-9 > .5 || now < stable_.back().t)
      return {};
    return Measurement{stable_.back().z + offset_,
                       std::max(o_.sigma_floor, stable_.back().sigma)};
  }

  // Only bridge the activation mismatch. Following GPS height changes after
  // activation still provides a vertical anchor when LIO increments are bad.
  // Re-anchor after aiding becomes unavailable, rather than applying a stale
  // ramp across a GPS outage. Call for every keyframe, including ineligible ones.
  void suspendIfUnavailable(int64_t now, bool eligible) {
    if (!eligible || !measurement(now))
      resetRecovery();
  }

  std::optional<Measurement> rampedMeasurement(int64_t now,
                                             double predicted_pose_z,
                                             bool eligible) {
    const auto raw = measurement(now);
    if (!eligible || !raw || !std::isfinite(predicted_pose_z)) {
      resetRecovery();
      return {};
    }
    if (o_.recovery_ramp_time == 0)
      return raw;
    if (!ramp_start_ns_) {
      ramp_start_ns_ = now;
      ramp_offset_ = predicted_pose_z - raw->z;
    }
    const double elapsed = std::max(0.0, (now - ramp_start_ns_) * 1e-9);
    const double remaining =
        std::max(0.0, 1.0 - elapsed / o_.recovery_ramp_time);
    return Measurement{raw->z + remaining * ramp_offset_, raw->sigma};
  }

private:
  void resetRecovery() { ramp_start_ns_ = 0; }
  struct Sample {
    int64_t t;
    double z, sigma;
  };
  struct Reference {
    int64_t t;
    double offset;
  };
  GpsHeightOptions o_;
  std::deque<Sample> stable_;
  std::deque<Reference> references_;
  int64_t last_seen_ = 0;
  bool frozen_ = false;
  double offset_ = 0;
  int64_t ramp_start_ns_ = 0;
  double ramp_offset_ = 0;
};
} // namespace gnocchi
