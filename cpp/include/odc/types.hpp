// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <cstdint>

namespace odc {

/// Number of driven mecanum wheels. Fixed at compile time: every buffer in the
/// control path is sized from this, so the hot loop never touches the heap.
inline constexpr std::size_t kNumWheels = 4;

/// Wheel indices. Physical layout matches the four-caster mounting pattern:
///
///        x (forward)
///          ^
///   FL[0]  |  FR[1]
///   -------+-------> y is to the LEFT (REP-103, right-handed, z up)
///   RL[2]  |  RR[3]
///
enum WheelIndex : std::size_t { kFL = 0, kFR = 1, kRL = 2, kRR = 3 };

template <typename T>
using WheelArray = std::array<T, kNumWheels>;

/// Planar body velocity, expressed in the base frame (REP-103).
struct Twist2D {
  double vx = 0.0;  ///< [m/s] forward
  double vy = 0.0;  ///< [m/s] left
  double wz = 0.0;  ///< [rad/s] yaw rate, CCW positive
};

/// Planar pose in the map/world frame.
struct Pose2D {
  double x = 0.0;      ///< [m]
  double y = 0.0;      ///< [m]
  double theta = 0.0;  ///< [rad], wrapped to (-pi, pi]
};

/// One control cycle of measurements coming off the wheel bus.
struct WheelFeedback {
  WheelArray<double> velocity{};  ///< [rad/s] measured wheel angular velocity
  WheelArray<double> current{};   ///< [A] measured motor current
  WheelArray<bool> healthy{};  ///< per-node liveness reported by the bus layer
};

/// Absolute pose fix from the localisation front-end (scan matcher / 3D
/// vision).
struct PoseFix {
  Pose2D pose{};
  double std_xy = 0.05;     ///< [m] 1-sigma position uncertainty
  double std_theta = 0.02;  ///< [rad] 1-sigma heading uncertainty
  bool valid = false;
};

/// Everything the safety layer needs to know about the surroundings this cycle.
struct SafetyInput {
  double nearest_obstacle_m =
      1e9;  ///< range to closest point inside the protective field
  bool estop_engaged = false;  ///< hardware emergency stop line
  bool bus_ok = true;          ///< wheel bus is transporting frames
};

/// Escalating health states. The core only ever moves *up* the ladder inside a
/// cycle; clearing a fault requires an explicit reset() from the supervisor.
enum class DriveState : std::uint8_t {
  kIdle = 0,            ///< powered, no motion commanded
  kRunning = 1,         ///< tracking a reference
  kSpeedLimited = 2,    ///< obstacle in the warning field, velocity scaled down
  kProtectiveStop = 3,  ///< recoverable stop (obstacle / watchdog / slip)
  kFault = 4,           ///< latched, requires reset
};

/// Bit flags describing *why* the core is unhappy. Cheap to log over CAN.
enum FaultBits : std::uint32_t {
  kFaultNone = 0u,
  kFaultCommandTimeout = 1u << 0,
  kFaultBusTimeout = 1u << 1,
  kFaultWheelUnhealthy = 1u << 2,
  kFaultWheelSlip = 1u << 3,
  kFaultEstop = 1u << 4,
  kFaultObstacle = 1u << 5,
  kFaultLocalisationLost = 1u << 6,
};

/// Result of one deterministic control step.
struct ControlOutput {
  WheelArray<double> torque_cmd{};    ///< [Nm] per-wheel torque setpoint
  WheelArray<double> velocity_ref{};  ///< [rad/s] per-wheel velocity setpoint
  Twist2D commanded_twist{};  ///< body twist actually applied after limiting
  Pose2D estimated_pose{};    ///< EKF output
  Twist2D estimated_twist{};  ///< odometry output, body frame
  WheelArray<double>
      slip_residual{};  ///< [rad/s] per-wheel kinematic inconsistency
  std::uint8_t slip_channels = 0;  ///< SlipChannel bitmask, why slip tripped
  std::size_t slip_wheel = 4;      ///< suspected wheel, kUnknownWheel if none
  DriveState state = DriveState::kIdle;
  std::uint32_t faults = kFaultNone;
  double speed_scale = 1.0;  ///< safety-derived velocity multiplier in [0, 1]
};

}  // namespace odc
