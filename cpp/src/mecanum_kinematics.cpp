// SPDX-License-Identifier: MIT
#include "odc/mecanum_kinematics.hpp"

namespace odc {

WheelArray<double> MecanumKinematics::inverse(const Twist2D& t) const {
  const double inv_r = 1.0 / geom_.wheel_radius;
  const double l = geom_.lxy();
  WheelArray<double> w{};
  w[kFL] = inv_r * (t.vx - t.vy - l * t.wz);
  w[kFR] = inv_r * (t.vx + t.vy + l * t.wz);
  w[kRL] = inv_r * (t.vx + t.vy - l * t.wz);
  w[kRR] = inv_r * (t.vx - t.vy + l * t.wz);
  return w;
}

Twist2D MecanumKinematics::forward(const WheelArray<double>& w) const {
  const double r4 = geom_.wheel_radius * 0.25;
  Twist2D t{};
  t.vx = r4 * (w[kFL] + w[kFR] + w[kRL] + w[kRR]);
  t.vy = r4 * (-w[kFL] + w[kFR] + w[kRL] - w[kRR]);
  t.wz = r4 / geom_.lxy() * (-w[kFL] + w[kFR] - w[kRL] + w[kRR]);
  return t;
}

double MecanumKinematics::consistencyError(const WheelArray<double>& w) {
  return 0.5 * (w[kFL] + w[kFR] - w[kRL] - w[kRR]);
}

WheelArray<double> MecanumKinematics::slipResidual(
    const WheelArray<double>& measured) const {
  const WheelArray<double> reconstructed = inverse(forward(measured));
  WheelArray<double> residual{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    residual[i] = measured[i] - reconstructed[i];
  }
  return residual;
}

}  // namespace odc
