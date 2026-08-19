// SPDX-License-Identifier: MIT
#pragma once

#include "odc/types.hpp"

namespace odc {

struct ChassisGeometry {
  double wheel_radius = 0.0625;  ///< [m]
  double half_wheelbase = 0.30;  ///< [m] lx, base centre to axle along x
  double half_track = 0.25;      ///< [m] ly, base centre to wheel along y

  /// The lever arm that shows up in every mecanum equation.
  constexpr double lxy() const { return half_wheelbase + half_track; }
};

/// Kinematics for a four-wheel mecanum base with rollers at +/-45 degrees.
///
/// The map from body twist (3 DoF) to wheel speeds (4 DoF) is over-determined.
/// That redundancy is not a nuisance -- it is a free, sensor-less slip
/// detector, and this class exposes it deliberately. See `slipResidual()`.
class MecanumKinematics {
 public:
  explicit MecanumKinematics(const ChassisGeometry& geom) : geom_(geom) {}

  /// Body twist -> wheel angular velocities [rad/s]. Exact.
  WheelArray<double> inverse(const Twist2D& twist) const;

  /// Wheel angular velocities -> body twist. Least-squares (pseudo-inverse),
  /// so it is the best fit even when the wheels disagree.
  Twist2D forward(const WheelArray<double>& wheel_velocity) const;

  /// Scalar consistency check. For a rigid, non-slipping mecanum base the
  /// front pair and rear pair must sum to the same value:
  ///
  ///     w_FL + w_FR - w_RL - w_RR == 0
  ///
  /// because that combination spans the null space of the 4x3 kinematic map.
  /// Any non-zero value is wheel slip, an encoder fault, or a geometry error.
  static double consistencyError(const WheelArray<double>& wheel_velocity);

  /// Per-wheel residual: measured speed minus the speed implied by the
  /// least-squares body twist.
  ///
  /// NOTE: this vector carries no more information than `consistencyError()`.
  /// The four kinematic columns are mutually orthogonal, so `forward()` is the
  /// exact Moore-Penrose pseudo-inverse and the residual is the orthogonal
  /// projection onto the single left-null direction:
  ///
  ///     residual == consistencyError(w) * [0.5, 0.5, -0.5, -0.5]
  ///
  /// All four entries therefore share one magnitude. It is useful for logging
  /// and for plotting alongside wheel speeds, but it cannot identify which
  /// wheel is slipping -- see SlipDetector for that.
  WheelArray<double> slipResidual(
      const WheelArray<double>& wheel_velocity) const;

  const ChassisGeometry& geometry() const { return geom_; }

 private:
  ChassisGeometry geom_;
};

}  // namespace odc
