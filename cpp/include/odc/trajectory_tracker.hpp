// SPDX-License-Identifier: MIT
#pragma once

#include "odc/types.hpp"

namespace odc {

struct TrackerParams {
  double kp_xy = 2.2;           ///< [1/s] world-frame position gain
  double kp_theta = 3.0;        ///< [1/s] heading gain
  double ki_xy = 0.25;          ///< [1/s^2] steady-state cross-track rejection
  double integral_limit = 0.4;  ///< [m] clamp on the integral term
  double max_vx = 1.2;          ///< [m/s]
  double max_vy = 0.9;          ///< [m/s] lateral is slower: rollers, not tyres
  double max_wz = 1.5;          ///< [rad/s]
  double max_accel = 1.0;       ///< [m/s^2] on the body-frame linear command
  double max_ang_accel = 2.5;   ///< [rad/s^2]
};

/// Reference the tracker is asked to follow at this instant.
struct TrajectoryPoint {
  Pose2D pose{};
  Twist2D velocity{};  ///< feedforward, expressed in the WORLD frame
};

/// Pose tracker for a holonomic base.
///
/// Because the platform can translate and rotate independently, the control
/// law stays linear: compute the error in the world frame, apply PI, add the
/// feedforward, then rotate the whole thing into the body frame once at the
/// end. No unicycle linearisation, no look-ahead point, no cusp handling.
class TrajectoryTracker {
 public:
  TrajectoryTracker() = default;
  explicit TrajectoryTracker(const TrackerParams& p) : p_(p) {}

  Twist2D update(const TrajectoryPoint& ref, const Pose2D& estimate, double dt);

  void reset();

  /// Signed cross-track and heading error from the most recent update.
  double lastPositionError() const { return last_pos_err_; }
  double lastHeadingError() const { return last_head_err_; }

  void setParams(const TrackerParams& p) { p_ = p; }
  const TrackerParams& params() const { return p_; }

 private:
  TrackerParams p_{};
  double int_x_ = 0.0;
  double int_y_ = 0.0;
  Twist2D prev_cmd_{};
  double last_pos_err_ = 0.0;
  double last_head_err_ = 0.0;
};

}  // namespace odc
