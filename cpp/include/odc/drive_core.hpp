// SPDX-License-Identifier: MIT
#pragma once

#include "odc/mecanum_kinematics.hpp"
#include "odc/pose_ekf.hpp"
#include "odc/safety_monitor.hpp"
#include "odc/slip_detector.hpp"
#include "odc/trajectory_tracker.hpp"
#include "odc/types.hpp"
#include "odc/wheel_controller.hpp"

namespace odc {

struct DriveCoreConfig {
  ChassisGeometry geometry{};
  WheelControlParams wheel{};
  TrackerParams tracker{};
  PoseEkfParams ekf{};
  SlipDetectorParams slip{};
  SafetyParams safety{};
  double control_period = 0.005;  ///< [s] nominal, 200 Hz
};

/// Everything the core needs from the outside world for one cycle.
struct CycleInput {
  WheelFeedback wheels{};
  double gyro_z = 0.0;  ///< [rad/s] measured yaw rate
  PoseFix pose_fix{};   ///< optional absolute fix
  TrajectoryPoint reference{};
  bool reference_fresh = false;  ///< a new setpoint arrived this cycle
  SafetyInput safety{};
};

/// The whole control stack behind one deterministic function call.
///
/// `step()` is the entire real-time contract: fixed period in, torque commands
/// out, no allocation, no I/O, no clock reads, no exceptions. Every dependency
/// is passed in and every result is returned, which is what makes the same
/// object usable from a 200 Hz ISR on the main wheel, from a ROS 2 node, and
/// from the Python simulator in this repository without recompiling anything.
class DriveCore {
 public:
  explicit DriveCore(const DriveCoreConfig& cfg);

  /// Run one control cycle.
  /// @param dt elapsed time since the previous call [s]
  ControlOutput step(const CycleInput& in, double dt);

  /// Clear latched faults and controller state, keep the pose estimate.
  void reset();
  /// Re-initialise the pose estimate (e.g. after a relocalisation).
  void setPose(const Pose2D& pose);

  const DriveCoreConfig& config() const { return cfg_; }
  const PoseEkf& ekf() const { return ekf_; }
  const SlipDetector& slipDetector() const { return slip_; }
  const TrajectoryTracker& tracker() const { return tracker_; }

 private:
  DriveCoreConfig cfg_;
  MecanumKinematics kin_;
  PoseEkf ekf_;
  SlipDetector slip_;
  TrajectoryTracker tracker_;
  SafetyMonitor safety_;
  WheelArray<WheelController> wheel_ctl_{};
  DriveState state_ = DriveState::kIdle;
};

}  // namespace odc
