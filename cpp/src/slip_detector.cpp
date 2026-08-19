// SPDX-License-Identifier: MIT
#include "odc/slip_detector.hpp"

#include <algorithm>
#include <cmath>

namespace odc {
namespace {

/// Median of four values, without sorting in place.
double median4(WheelArray<double> v) {
  std::sort(v.begin(), v.end());
  return 0.5 * (v[1] + v[2]);
}

}  // namespace

bool SlipDetector::trip(double statistic, double enter, double exit,
                        bool latched, double& above_for, double confirm_time,
                        double dt) {
  const double threshold = latched ? exit : enter;
  if (statistic > threshold) {
    above_for += dt;
    return latched || above_for >= confirm_time;
  }
  above_for = 0.0;
  return false;
}

void SlipDetector::update(const MecanumKinematics& kin,
                          const WheelArray<double>& measured,
                          const WheelArray<double>& current, double gyro_z,
                          double nis, bool nis_fresh, double dt) {
  residual_ = kin.slipResidual(measured);
  const double signed_error = MecanumKinematics::consistencyError(measured);
  const Twist2D odom = kin.forward(measured);

  double speed = 0.0;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    speed = std::max(speed, std::abs(measured[i]));
  }

  // --- Statistics -----------------------------------------------------------
  const double gyro_raw = std::abs(gyro_z - odom.wz);

  const double alpha = dt / std::max(p_.filter_tau + dt, 1e-9);
  kin_stat_ += alpha * (std::abs(signed_error) - kin_stat_);
  gyro_stat_ += alpha * (gyro_raw - gyro_stat_);

  // Fixes arrive an order of magnitude slower than the control loop, so the
  // innovation statistic only moves when there is new evidence. Between
  // fixes it holds, rather than decaying towards zero and forgetting.
  if (nis_fresh) {
    // Reported statistic is a clipped EWMA, for logging only. The trip
    // decision below counts fixes instead, so one wild value cannot move it.
    nis_stat_ += 0.35 * (std::min(nis, 3.0 * p_.nis_enter) - nis_stat_);

    if (nis > p_.nis_enter) {
      ++nis_bad_;
      nis_good_ = 0;
    } else if (nis < p_.nis_exit) {
      ++nis_good_;
      nis_bad_ = 0;
    }
    if (nis_bad_ >= p_.nis_confirm_fixes) nis_latched_ = true;
    if (nis_good_ >= p_.nis_clear_fixes) nis_latched_ = false;
  }

  if (speed < p_.min_speed) {
    // Standing still: every constraint is trivially satisfied. Do not accuse.
    kin_timer_ = gyro_timer_ = 0.0;
    nis_bad_ = nis_good_ = 0;
    kin_latched_ = nis_latched_ = gyro_latched_ = false;
    channels_ = kChannelNone;
    worst_wheel_ = kUnknownWheel;
    return;
  }

  // --- Channels -------------------------------------------------------------
  kin_latched_ = trip(kin_stat_, p_.kin_enter, p_.kin_exit, kin_latched_,
                      kin_timer_, p_.confirm_time, dt);
  gyro_latched_ = trip(gyro_stat_, p_.gyro_enter, p_.gyro_exit, gyro_latched_,
                       gyro_timer_, p_.confirm_time, dt);

  channels_ = static_cast<std::uint8_t>(
      (kin_latched_ ? static_cast<unsigned>(kChannelKinematic) : 0u) |
      (nis_latched_ ? static_cast<unsigned>(kChannelOdometry) : 0u) |
      (gyro_latched_ ? static_cast<unsigned>(kChannelGyro) : 0u));

  if (channels_ != kChannelNone) {
    attribute(signed_error, current);
  } else {
    worst_wheel_ = kUnknownWheel;
  }
}

void SlipDetector::attribute(double signed_error,
                             const WheelArray<double>& current) {
  // Attribution is only possible when the KINEMATIC channel fired, i.e. when
  // a wheel is failing to track its setpoint. Then the sign of the error
  // narrows the culprit to a pair -- a positive error means either a FRONT
  // wheel running fast or a REAR wheel being held back -- and current picks
  // within that pair, because a free-spinning wheel draws less than its
  // siblings and a dragged one draws more.
  //
  // When only the innovation or gyro channel fired, the platform is sliding
  // but every wheel is tracking, and nothing in the proprioceptive data
  // singles one out. Say so rather than invent an answer.
  if (!kin_latched_) {
    worst_wheel_ = kUnknownWheel;
    return;
  }

  WheelArray<double> mag{};
  for (std::size_t i = 0; i < kNumWheels; ++i) mag[i] = std::abs(current[i]);
  const double med = median4(mag);

  const bool front_is_fast = signed_error > 0.0;

  double best = 0.0;
  std::size_t best_idx = kUnknownWheel;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const bool is_front = (i == kFL || i == kFR);
    const bool expect_free = (front_is_fast == is_front);
    const double dev = mag[i] - med;
    const double score = expect_free ? -dev : dev;
    if (score > best) {
      best = score;
      best_idx = i;
    }
  }

  worst_wheel_ = (best >= p_.attribution_deadband) ? best_idx : kUnknownWheel;
}

void SlipDetector::reset() {
  residual_ = WheelArray<double>{};
  kin_stat_ = gyro_stat_ = 0.0;
  nis_stat_ = 3.0;  // the expected value of chi-square with 3 DoF
  kin_timer_ = gyro_timer_ = 0.0;
  nis_bad_ = nis_good_ = 0;
  kin_latched_ = nis_latched_ = gyro_latched_ = false;
  channels_ = kChannelNone;
  worst_wheel_ = kUnknownWheel;
}

}  // namespace odc
