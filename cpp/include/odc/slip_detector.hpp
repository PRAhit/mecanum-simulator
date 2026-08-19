// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include "odc/mecanum_kinematics.hpp"
#include "odc/types.hpp"

namespace odc {

/// Which evidence tripped. Exposed so the diagnostic log says *why*, and so a
/// supervisor can react differently to "a wheel is jammed" and "the floor is
/// wet" even though both are reported as slip.
enum SlipChannel : std::uint8_t {
  kChannelNone = 0u,
  kChannelKinematic = 1u << 0,  ///< wheel speeds disagree with each other
  kChannelOdometry = 1u << 1,   ///< odometry disagrees with the absolute fixes
  kChannelGyro = 1u << 2,       ///< odometry yaw disagrees with the gyro
};

struct SlipDetectorParams {
  // --- Channel 1: kinematic consistency ------------------------------------
  double kin_enter = 1.2;  ///< [rad/s] |consistency error| to trip
  double kin_exit = 0.5;   ///< [rad/s] to clear (Schmitt hysteresis)

  // --- Channel 2: estimator innovation --------------------------------------
  /// Filtered normalised innovation squared from the pose EKF. Under correct
  /// modelling NIS is chi-square with 3 DoF, so it averages 3.0; these
  /// thresholds are multiples of that expectation, not tuned constants.
  double nis_enter = 9.0;
  double nis_exit = 5.0;
  /// Consecutive bad fixes required to trip, and consecutive good fixes
  /// required to clear. Counting FIXES rather than filtering a continuous
  /// statistic is what makes this channel robust to the occasional scan match
  /// that latches onto the wrong wall: at a 1% outlier rate, three in a row is
  /// a one-in-a-million event, while a single outlier through an EWMA is
  /// enough to fake a slip and protective-stop a healthy robot.
  int nis_confirm_fixes = 3;
  int nis_clear_fixes = 3;

  // --- Channel 3: gyro vs odometry -----------------------------------------
  double gyro_enter = 0.20;  ///< [rad/s] disagreement to trip
  double gyro_exit = 0.10;   ///< [rad/s]

  // --- Shared debouncing ----------------------------------------------------
  double filter_tau = 0.05;    ///< [s] first-order filter on every statistic
  double confirm_time = 0.06;  ///< [s] evidence must persist this long
  double min_speed = 0.05;     ///< [rad/s] below this, do not judge
  double attribution_deadband =
      0.30;  ///< [A] refuse to name a wheel below this
};

/// Sentinel returned by worstWheel() when the culprit cannot be identified.
inline constexpr std::size_t kUnknownWheel = kNumWheels;

/// Wheel-slip and traction-loss detection for a four-wheel mecanum base.
///
/// Three independent channels, OR'd. Each exists because the others are blind
/// to a specific, real failure:
///
/// 1. KINEMATIC CONSISTENCY. The map from body twist (3 DoF) to wheel speeds
///    (4 DoF) is over-determined, so a rigid base must satisfy exactly one
///    scalar constraint:
///
///        w_FL + w_FR - w_RL - w_RR == 0
///
///    Sensor-less and exact. Catches wheels that fail to TRACK their setpoint:
///    a seized bearing, a wheel wedged against a pallet foot, a dead encoder.
///
///    Crucially, it does NOT catch traction loss. Under per-wheel velocity
///    control the setpoints are themselves produced by inverse kinematics, so
///    four well-tracking wheels satisfy the constraint BY CONSTRUCTION however
///    much the platform is actually sliding across the floor. This is the
///    single most important thing to understand about the method, and the
///    reason the next two channels exist.
///
/// 2. ESTIMATOR INNOVATION. Traction loss is only observable against an
///    EXTERNAL reference, so use the one the robot already has: the pose EKF
///    predicts forward on odometry and is corrected by the scan matcher, and
///    its normalised innovation is precisely a measure of how much the wheels
///    are lying. NIS is chi-square with 3 DoF, so a sustained value far above
///    3.0 means the wheels and the world disagree. This is the channel that
///    sees the oil spill.
///
///    An earlier version of this class compared each wheel's current against
///    the fleet median instead, on the theory that a slipping wheel transmits
///    less force. That is wrong for a mecanum base: the four wheels carry
///    genuinely different loads during any curved or diagonal motion -- a
///    figure-of-eight produces a normal 3.5 A spread -- so the statistic
///    false-positives constantly. Current is kept, but only for attribution
///    once another channel has already fired.
///
/// 3. GYRO VERSUS ODOMETRY. Asymmetric slip rotates the platform in a way the
///    wheels do not report. Comparing the odometry yaw rate against the rate
///    gyro catches that even when both channels above look clean.
///
/// What none of them catch: all four wheels slipping equally in pure
/// translation. That is genuinely unobservable from proprioception alone --
/// there is no internal reference left to disagree with. The EKF's pose-fix
/// innovation is the backstop, which is why `PoseEkf` gates its updates rather
/// than trusting odometry.
class SlipDetector {
 public:
  SlipDetector() = default;
  explicit SlipDetector(const SlipDetectorParams& p) : p_(p) {}

  /// @param kin       chassis kinematics
  /// @param measured  wheel angular velocities [rad/s]
  /// @param current   motor currents [A]; used only for attribution
  /// @param gyro_z    measured yaw rate [rad/s]
  /// @param nis       PoseEkf::lastNis()
  /// @param nis_fresh PoseEkf::nisFresh()
  /// @param dt        control period [s]
  void update(const MecanumKinematics& kin, const WheelArray<double>& measured,
              const WheelArray<double>& current, double gyro_z, double nis,
              bool nis_fresh, double dt);

  void reset();

  bool slipping() const { return channels_ != kChannelNone; }
  std::uint8_t channels() const { return channels_; }

  /// Index of the wheel believed responsible, or kUnknownWheel when the
  /// evidence does not single one out.
  std::size_t worstWheel() const { return worst_wheel_; }

  double kinematicError() const { return kin_stat_; }     ///< [rad/s], filtered
  double innovationStat() const { return nis_stat_; }     ///< filtered NIS
  double gyroDisagreement() const { return gyro_stat_; }  ///< [rad/s], filtered

  /// Signed residual vector. Always parallel to [1, 1, -1, -1] -- see
  /// MecanumKinematics::slipResidual. Logged, never ranked.
  const WheelArray<double>& residual() const { return residual_; }

 private:
  /// One debounced Schmitt trigger. Returns the updated latch state.
  static bool trip(double statistic, double enter, double exit, bool latched,
                   double& above_for, double confirm_time, double dt);

  void attribute(double signed_error, const WheelArray<double>& current);

  SlipDetectorParams p_{};
  WheelArray<double> residual_{};

  double kin_stat_ = 0.0, nis_stat_ = 3.0, gyro_stat_ = 0.0;
  int nis_bad_ = 0, nis_good_ = 0;
  double kin_timer_ = 0.0, gyro_timer_ = 0.0;
  bool kin_latched_ = false, nis_latched_ = false, gyro_latched_ = false;

  std::uint8_t channels_ = kChannelNone;
  std::size_t worst_wheel_ = kUnknownWheel;
};

}  // namespace odc
