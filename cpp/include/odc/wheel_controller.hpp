// SPDX-License-Identifier: MIT
#pragma once

#include "odc/types.hpp"

namespace odc {

struct WheelControlParams {
  double kp = 0.45;            ///< [Nm/(rad/s)]
  double ki = 6.0;             ///< [Nm/(rad/s)/s]
  double kff_viscous = 0.012;  ///< [Nm/(rad/s)] feedforward vs. speed
  double kff_coulomb = 0.05;   ///< [Nm] static friction feedforward
  double torque_limit = 4.5;   ///< [Nm] per wheel, symmetric
  double accel_limit = 40.0;   ///< [rad/s^2] setpoint slew limit
  /// Back-calculation coefficient. Rule of thumb: ki/kp, which is what makes
  /// the integrator settle near the actuator limit during a stall instead of
  /// parking at ki*error/gain. Too small and windup returns; too large and the
  /// integrator is bled away during ordinary transients.
  double anti_windup_gain = 13.3;
};

/// Single-wheel velocity loop: PI + friction feedforward, with setpoint slew
/// limiting and back-calculation anti-windup.
///
/// Deliberately allocation-free and branch-light -- this is the piece that
/// would be cross-compiled and dropped onto the wheel node's MCU, and the
/// identical object is what the Python simulation exercises.
class WheelController {
 public:
  WheelController() = default;
  explicit WheelController(const WheelControlParams& p) : p_(p) {}

  /// One fixed-step update. Returns the torque command in Nm.
  double update(double velocity_ref, double velocity_meas, double dt);

  void reset();

  double integrator() const { return integral_; }
  double rampedSetpoint() const { return ramped_ref_; }
  bool saturated() const { return saturated_; }

  void setParams(const WheelControlParams& p) { p_ = p; }
  const WheelControlParams& params() const { return p_; }

 private:
  WheelControlParams p_{};
  double integral_ = 0.0;
  double ramped_ref_ = 0.0;
  bool saturated_ = false;
};

}  // namespace odc
