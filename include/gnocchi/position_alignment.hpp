#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace gnocchi {
struct AlignmentOptions {
  bool enabled = false;
  double window = 60.0;
  double min_baseline = 10.0;
  double max_residual = 1.5;
  double max_gap = 0.5;
  double sigma_floor =
      0.175; // 10 degrees; correlated GPS is not N independent samples
  double max_sigma = 0.35;
  int min_samples = 10;
  Eigen::Vector3d antenna = Eigen::Vector3d::Zero();
};

// Initialization only. Match antenna positions by sensor timestamp, never
// arrival time.
class PositionAlignment {
public:
  struct Result {
    double yaw, sigma, rms;
  };
  explicit PositionAlignment(const AlignmentOptions &o) : o_(o) {
    if (!std::isfinite(o.window) || o.window <= 0 ||
        !std::isfinite(o.min_baseline) || o.min_baseline <= 0 ||
        !std::isfinite(o.max_residual) || o.max_residual <= 0 ||
        !std::isfinite(o.max_gap) || o.max_gap <= 0 ||
        !std::isfinite(o.sigma_floor) || o.sigma_floor <= 0 ||
        !std::isfinite(o.max_sigma) || o.max_sigma < o.sigma_floor ||
        o.min_samples < 3 || !o.antenna.allFinite())
      throw std::invalid_argument("Invalid heading.alignment parameters");
  }
  void addOdometry(int64_t t, const Eigen::Isometry3d &p) {
    if (!odoms_.empty() && t < odoms_.back().t) {
      odoms_.clear();
      fixes_.clear();
    }
    if (!odoms_.empty() && t == odoms_.back().t)
      return;
    odoms_.push_back({t, p});
    while (!odoms_.empty() && (t - odoms_.front().t) * 1e-9 > o_.window)
      odoms_.pop_front();
    while (!fixes_.empty() && (t - fixes_.front().t) * 1e-9 > o_.window)
      fixes_.pop_front();
    while (odoms_.size() > 20000)
      odoms_.pop_front();
  }
  void addFix(int64_t t, const Eigen::Vector3d &p, double sigma) {
    // Avoid claiming extra information from densely sampled, correlated fixes.
    if (!fixes_.empty() && (t - fixes_.back().t) * 1e-9 < 1.0)
      return;
    fixes_.push_back({t, p, sigma});
    while (fixes_.size() > 1000)
      fixes_.pop_front();
  }
  std::optional<Eigen::Isometry3d> poseAt(int64_t t) const {
    for (size_t i = 0; i < odoms_.size(); ++i) {
      if (odoms_[i].t == t)
        return odoms_[i].p;
      if (i == 0 || odoms_[i - 1].t > t || odoms_[i].t < t)
        continue;
      const auto &a = odoms_[i - 1];
      const auto &b = odoms_[i];
      if ((b.t - a.t) * 1e-9 > o_.max_gap)
        return {};
      double w = double(t - a.t) / double(b.t - a.t);
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      p.translation() = (1 - w) * a.p.translation() + w * b.p.translation();
      p.linear() = Eigen::Quaterniond(a.p.rotation())
                       .slerp(w, Eigen::Quaterniond(b.p.rotation()))
                       .toRotationMatrix();
      return p;
    }
    return {};
  }
  std::optional<Result> estimate() const {
    std::vector<Eigen::Vector2d> a, b;
    double sigma = 0.0;
    for (const auto &f : fixes_) {
      auto p = poseAt(f.t);
      if (!p)
        continue;
      a.push_back(((*p) * o_.antenna).head<2>());
      b.push_back(f.p.head<2>());
      sigma = std::max(sigma, f.sigma);
    }
    if (a.size() < static_cast<size_t>(o_.min_samples))
      return {};
    Eigen::Vector2d ac = Eigen::Vector2d::Zero(), bc = ac;
    for (size_t i = 0; i < a.size(); ++i) {
      ac += a[i];
      bc += b[i];
    }
    ac /= a.size();
    bc /= a.size();
    double dot = 0, cross = 0, energy = 0, span = 0;
    for (size_t i = 0; i < a.size(); ++i) {
      Eigen::Vector2d x = a[i] - ac, y = b[i] - bc;
      dot += x.dot(y);
      cross += x.x() * y.y() - x.y() * y.x();
      energy += x.squaredNorm();
      for (size_t j = 0; j < i; ++j)
        span = std::max(span, (a[i] - a[j]).norm());
    }
    if (span < o_.min_baseline || energy < 1e-9 ||
        std::hypot(dot, cross) < 1e-9)
      return {};
    double yaw = std::atan2(cross, dot), error = 0;
    Eigen::Rotation2Dd r(yaw);
    for (size_t i = 0; i < a.size(); ++i)
      error += (r * (a[i] - ac) - (b[i] - bc)).squaredNorm();
    double rms = std::sqrt(error / a.size());
    // Deliberately no sqrt(N): this is a conservative heuristic, not a
    // calibrated covariance.
    double angular =
        std::max(o_.sigma_floor, std::atan2(std::max(sigma, rms),
                                            std::sqrt(energy / a.size())));
    if (rms > o_.max_residual || angular > o_.max_sigma)
      return {};
    return Result{yaw, angular, rms};
  }

private:
  struct Odom {
    int64_t t;
    Eigen::Isometry3d p;
  };
  struct Fix {
    int64_t t;
    Eigen::Vector3d p;
    double sigma;
  };
  AlignmentOptions o_;
  std::deque<Odom> odoms_;
  std::deque<Fix> fixes_;
};
} // namespace gnocchi
