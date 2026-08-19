// SPDX-License-Identifier: MIT
#pragma once

#include <array>

#include "odc/types.hpp"

namespace odc {

/// Wrap an angle to (-pi, pi].
double wrapAngle(double a);

struct PoseEkfParams {
  double sigma_v = 0.03;          ///< [m/s] 1-sigma odometry velocity noise
  double sigma_gyro = 0.004;      ///< [rad/s] 1-sigma gyro noise
  double sigma_bias_walk = 1e-5;  ///< [rad/s/sqrt(s)] gyro bias random walk
  double slip_inflation = 25.0;   ///< multiplies Q when slip is detected
  double chi2_gate = 11.34;       ///< 3-DoF chi-square, 99% confidence
  double lost_timeout_s = 4.0;    ///< no accepted fix for this long => lost
};

/// Extended Kalman filter over [x, y, theta, gyro_bias].
///
/// Prediction uses the mecanum odometry twist for translation and the gyro for
/// heading, which is the right split: mecanum rollers slip laterally far more
/// than a rate gyro drifts. Absolute fixes from the scan matcher are gated on
/// Mahalanobis distance so a single bad match cannot teleport the estimate.
class PoseEkf {
 public:
  PoseEkf() { reset(Pose2D{}); }
  explicit PoseEkf(const PoseEkfParams& p) : p_(p) { reset(Pose2D{}); }

  void reset(const Pose2D& initial);

  /// Propagate with the body twist from odometry and the measured yaw rate.
  /// `slip_detected` inflates the process noise so the filter leans on the
  /// next absolute fix instead of trusting corrupted wheel data.
  void predict(const Twist2D& odom_twist, double gyro_z, double dt,
               bool slip_detected);

  /// Fuse an absolute pose. Returns false if the fix was rejected by the gate.
  bool update(const PoseFix& fix);

  /// Normalised innovation squared from the most recent fix, accepted or
  /// rejected. Under correct modelling this is chi-square with 3 degrees of
  /// freedom, so it averages 3. A sustained excess means the odometry
  /// prediction and the absolute fixes disagree systematically -- which is
  /// what wheel slip looks like from the estimator's point of view.
  double lastNis() const { return last_nis_; }

  /// True on the cycle a fix was processed, so consumers can hold the value
  /// between the 10 Hz fixes and the 200 Hz control loop.
  bool nisFresh() const { return nis_fresh_; }

  /// Clear the fresh flag. Called once per control cycle by DriveCore.
  void clearNisFresh() { nis_fresh_ = false; }

  /// Seconds since the last *accepted* absolute fix.
  double timeSinceFix() const { return time_since_fix_; }
  bool localisationLost() const { return time_since_fix_ > p_.lost_timeout_s; }

  Pose2D pose() const { return Pose2D{x_[0], x_[1], x_[2]}; }
  double gyroBias() const { return x_[3]; }
  /// Position uncertainty, trace-based scalar summary [m].
  double positionSigma() const;

 private:
  static constexpr std::size_t kN = 4;
  PoseEkfParams p_{};
  std::array<double, kN> x_{};
  std::array<std::array<double, kN>, kN> P_{};
  double time_since_fix_ = 0.0;
  double last_nis_ = 0.0;
  bool nis_fresh_ = false;
};

}  // namespace odc
