// SPDX-License-Identifier: MIT
#include "odc/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "odc/pose_ekf.hpp"  // wrapAngle

namespace odc {
namespace {
inline double clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}
inline double slew(double target, double previous, double max_delta) {
  return previous + clamp(target - previous, -max_delta, max_delta);
}
}  // namespace

Twist2D TrajectoryTracker::update(const TrajectoryPoint& ref, const Pose2D& est,
                                  double dt) {
  if (dt <= 0.0) return prev_cmd_;

  // --- World-frame error.
  const double ex = ref.pose.x - est.x;
  const double ey = ref.pose.y - est.y;
  const double eth = wrapAngle(ref.pose.theta - est.theta);

  last_pos_err_ = std::sqrt(ex * ex + ey * ey);
  last_head_err_ = eth;

  int_x_ = clamp(int_x_ + ex * dt, -p_.integral_limit, p_.integral_limit);
  int_y_ = clamp(int_y_ + ey * dt, -p_.integral_limit, p_.integral_limit);

  // --- PI + feedforward, still in the world frame.
  const double vx_w = ref.velocity.vx + p_.kp_xy * ex + p_.ki_xy * int_x_;
  const double vy_w = ref.velocity.vy + p_.kp_xy * ey + p_.ki_xy * int_y_;
  const double wz = ref.velocity.wz + p_.kp_theta * eth;

  // --- Single rotation into the body frame.
  const double c = std::cos(est.theta), s = std::sin(est.theta);
  Twist2D cmd{};
  cmd.vx = vx_w * c + vy_w * s;
  cmd.vy = -vx_w * s + vy_w * c;
  cmd.wz = wz;

  // --- Envelope: per-axis speed limits, then acceleration limits.
  cmd.vx = clamp(cmd.vx, -p_.max_vx, p_.max_vx);
  cmd.vy = clamp(cmd.vy, -p_.max_vy, p_.max_vy);
  cmd.wz = clamp(cmd.wz, -p_.max_wz, p_.max_wz);

  const double dv = p_.max_accel * dt;
  const double dw = p_.max_ang_accel * dt;
  cmd.vx = slew(cmd.vx, prev_cmd_.vx, dv);
  cmd.vy = slew(cmd.vy, prev_cmd_.vy, dv);
  cmd.wz = slew(cmd.wz, prev_cmd_.wz, dw);

  prev_cmd_ = cmd;
  return cmd;
}

void TrajectoryTracker::reset() {
  int_x_ = int_y_ = 0.0;
  prev_cmd_ = Twist2D{};
  last_pos_err_ = last_head_err_ = 0.0;
}

}  // namespace odc
