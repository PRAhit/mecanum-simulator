// SPDX-License-Identifier: MIT
#pragma once

#include "odc/types.hpp"

namespace odc {

struct SafetyParams {
  double stop_distance = 0.35;     ///< [m] protective field boundary
  double slow_distance = 1.40;     ///< [m] warning field boundary
  double min_speed_scale = 0.0;    ///< scale applied at stop_distance
  double command_timeout = 0.30;   ///< [s] no new twist command => stop
  double bus_timeout = 0.10;       ///< [s] no wheel feedback => stop
  double clear_hysteresis = 0.15;  ///< [m] extra clearance needed to resume
  /// Minimum time a protective stop is held after its cause clears.
  ///
  /// Without this the machine chatters: slip trips a stop, the stop halts the
  /// wheels, still wheels report no slip, motion resumes, the wheels slip
  /// again -- several times a second. The dwell turns a limit cycle into a
  /// single clean stop-and-restart.
  double min_stop_time = 1.0;  ///< [s]
};

/// Speed supervision and fault arbitration.
///
/// Models the shape of an ISO 3691-4 protective/warning field pair: full speed
/// outside the warning field, a linear ramp down through it, and a protective
/// stop at the inner boundary. Recovery needs extra clearance so the platform
/// does not oscillate between stopping and creeping at the field edge.
class SafetyMonitor {
 public:
  SafetyMonitor() = default;
  explicit SafetyMonitor(const SafetyParams& p) : p_(p) {}

  /// @param in            environment and hardware inputs
  /// @param wheels        this cycle's wheel feedback (health flags are read)
  /// @param slip_detected output of SlipDetector
  /// @param localisation_lost EKF has had no accepted fix for too long
  /// @param dt            control period [s]
  void update(const SafetyInput& in, const WheelFeedback& wheels,
              bool slip_detected, bool localisation_lost, double dt);

  /// Call when a fresh external twist command arrives, to pet the watchdog.
  void notifyCommand() { since_command_ = 0.0; }

  /// Clear latched faults. The supervisor must call this deliberately.
  void reset();

  double speedScale() const { return speed_scale_; }
  std::uint32_t faults() const { return faults_; }
  bool stopRequired() const { return stop_required_; }

  const SafetyParams& params() const { return p_; }

 private:
  SafetyParams p_{};
  double speed_scale_ = 1.0;
  double since_command_ = 0.0;
  double since_bus_ = 0.0;
  bool stop_required_ = false;
  bool in_protective_stop_ = false;
  double stop_dwell_ = 0.0;
  std::uint32_t faults_ = kFaultNone;
};

}  // namespace odc
