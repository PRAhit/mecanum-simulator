// SPDX-License-Identifier: MIT
#include "odc/wheel_controller.hpp"

#include <algorithm>
#include <cmath>

namespace odc {
namespace {

inline double clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

/// Smooth approximation of sign() so the Coulomb feedforward term does not
/// chatter around zero speed.
inline double softSign(double v, double eps) { return std::tanh(v / eps); }

}  // namespace

double WheelController::update(double velocity_ref, double velocity_meas,
                               double dt) {
  // 1. Slew-limit the setpoint. Protects the gearbox and keeps the four
  //    wheels roughly synchronised when a large twist step arrives.
  const double max_delta = p_.accel_limit * dt;
  ramped_ref_ += clamp(velocity_ref - ramped_ref_, -max_delta, max_delta);

  const double error = ramped_ref_ - velocity_meas;

  // 2. Feedforward: viscous + Coulomb friction model of the drivetrain.
  const double ff = p_.kff_viscous * ramped_ref_ +
                    p_.kff_coulomb * softSign(ramped_ref_, 0.5);

  // 3. PI, integrated before saturation so back-calculation can undo it.
  integral_ += p_.ki * error * dt;
  const double unsaturated = ff + p_.kp * error + integral_;
  const double saturated_out =
      clamp(unsaturated, -p_.torque_limit, p_.torque_limit);

  // 4. Back-calculation anti-windup: bleed the integrator by the amount the
  //    actuator refused to deliver.
  const double excess = unsaturated - saturated_out;
  integral_ -= p_.anti_windup_gain * excess * dt;
  saturated_ = (excess != 0.0);

  return saturated_out;
}

void WheelController::reset() {
  integral_ = 0.0;
  ramped_ref_ = 0.0;
  saturated_ = false;
}

}  // namespace odc
