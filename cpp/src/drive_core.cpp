// SPDX-License-Identifier: MIT
#include "odc/drive_core.hpp"

#include <cmath>

namespace odc {

DriveCore::DriveCore(const DriveCoreConfig& cfg)
    : cfg_(cfg),
      kin_(cfg.geometry),
      ekf_(cfg.ekf),
      slip_(cfg.slip),
      tracker_(cfg.tracker),
      safety_(cfg.safety) {
  for (auto& c : wheel_ctl_) c = WheelController(cfg.wheel);
}

ControlOutput DriveCore::step(const CycleInput& in, double dt) {
  ControlOutput out{};

  // -------------------------------------------------------------------------
  // 1. Estimation. Odometry from the four wheels, then slip, then the EKF.
  //    Slip is computed *before* the EKF so the filter can distrust the very
  //    odometry sample that is currently corrupted.
  // -------------------------------------------------------------------------
  const Twist2D odom = kin_.forward(in.wheels.velocity);

  // The EKF inflates its process noise while slip is active, and the slip
  // detector consumes the EKF's innovation. To break the circular dependency
  // the filter uses the PREVIOUS cycle's slip verdict -- a 5 ms lag against a
  // fault that persists for hundreds of milliseconds.
  ekf_.predict(odom, in.gyro_z, dt, slip_.slipping());
  if (in.pose_fix.valid) {
    ekf_.update(in.pose_fix);
  }
  const Pose2D pose = ekf_.pose();

  slip_.update(kin_, in.wheels.velocity, in.wheels.current, in.gyro_z,
               ekf_.lastNis(), ekf_.nisFresh(), dt);
  ekf_.clearNisFresh();

  // -------------------------------------------------------------------------
  // 2. Safety arbitration.
  // -------------------------------------------------------------------------
  if (in.reference_fresh) safety_.notifyCommand();
  safety_.update(in.safety, in.wheels, slip_.slipping(),
                 ekf_.localisationLost(), dt);

  // -------------------------------------------------------------------------
  // 3. Guidance. Always run the tracker so its integrator sees the real error
  //    history, then scale the result. Scaling after the fact keeps the
  //    *direction* of motion intact while the magnitude ramps down -- which is
  //    what you want when creeping past a pallet, not a hard axis-wise clip.
  // -------------------------------------------------------------------------
  Twist2D twist = tracker_.update(in.reference, pose, dt);

  const double scale = safety_.speedScale();
  twist.vx *= scale;
  twist.vy *= scale;
  twist.wz *= scale;

  if (safety_.stopRequired()) {
    twist = Twist2D{};
    tracker_.reset();
  }

  // -------------------------------------------------------------------------
  // 4. Allocation to wheels, then four independent velocity loops.
  // -------------------------------------------------------------------------
  const WheelArray<double> wheel_ref = kin_.inverse(twist);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    out.velocity_ref[i] = wheel_ref[i];
    out.torque_cmd[i] =
        wheel_ctl_[i].update(wheel_ref[i], in.wheels.velocity[i], dt);
  }

  // A wheel node that reports itself unhealthy gets zero torque rather than a
  // stale command; the safety layer has already ordered a stop, this makes it
  // explicit at the actuator.
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (!in.wheels.healthy[i]) {
      out.torque_cmd[i] = 0.0;
      wheel_ctl_[i].reset();
    }
  }

  // -------------------------------------------------------------------------
  // 5. State machine.
  // -------------------------------------------------------------------------
  const std::uint32_t faults = safety_.faults();
  constexpr std::uint32_t kLatching = kFaultEstop | kFaultWheelUnhealthy;

  if ((faults & kLatching) != 0u) {
    state_ = DriveState::kFault;
  } else if (safety_.stopRequired()) {
    state_ = DriveState::kProtectiveStop;
  } else if (scale < 0.999) {
    state_ = DriveState::kSpeedLimited;
  } else {
    const double motion =
        std::abs(twist.vx) + std::abs(twist.vy) + std::abs(twist.wz);
    state_ = (motion > 1e-4) ? DriveState::kRunning : DriveState::kIdle;
  }

  out.commanded_twist = twist;
  out.estimated_pose = pose;
  out.estimated_twist = odom;
  out.slip_residual = slip_.residual();
  out.slip_channels = slip_.channels();
  out.slip_wheel = slip_.worstWheel();
  out.state = state_;
  out.faults = faults;
  out.speed_scale = scale;
  return out;
}

void DriveCore::reset() {
  safety_.reset();
  slip_.reset();
  tracker_.reset();
  for (auto& c : wheel_ctl_) c.reset();
  state_ = DriveState::kIdle;
}

void DriveCore::setPose(const Pose2D& pose) { ekf_.reset(pose); }

}  // namespace odc
