// SPDX-License-Identifier: MIT
#include "odc/safety_monitor.hpp"

#include <algorithm>

namespace odc {

void SafetyMonitor::update(const SafetyInput& in, const WheelFeedback& wheels,
                           bool slip_detected, bool localisation_lost,
                           double dt) {
  since_command_ += dt;
  since_bus_ = in.bus_ok ? 0.0 : since_bus_ + dt;

  std::uint32_t f = kFaultNone;

  // --- Distance-based speed scaling with recovery hysteresis.
  const double d = in.nearest_obstacle_m;
  const double stop_at = in_protective_stop_
                             ? p_.stop_distance + p_.clear_hysteresis
                             : p_.stop_distance;

  double scale;
  if (d <= stop_at) {
    scale = p_.min_speed_scale;
    in_protective_stop_ = true;
    f |= kFaultObstacle;
  } else if (d >= p_.slow_distance) {
    scale = 1.0;
    in_protective_stop_ = false;
  } else {
    const double span = std::max(p_.slow_distance - p_.stop_distance, 1e-6);
    scale = (d - p_.stop_distance) / span;
    in_protective_stop_ = false;
  }
  speed_scale_ = std::min(std::max(scale, 0.0), 1.0);

  // --- Watchdogs.
  if (since_command_ > p_.command_timeout) f |= kFaultCommandTimeout;
  if (since_bus_ > p_.bus_timeout) f |= kFaultBusTimeout;

  // --- Node health.
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (!wheels.healthy[i]) {
      f |= kFaultWheelUnhealthy;
      break;
    }
  }

  if (slip_detected) f |= kFaultWheelSlip;
  if (localisation_lost) f |= kFaultLocalisationLost;
  if (in.estop_engaged) f |= kFaultEstop;

  // --- Latch the sticky ones. An e-stop or a dead wheel node must not clear
  //     itself just because the next frame looked fine.
  constexpr std::uint32_t kLatching = kFaultEstop | kFaultWheelUnhealthy;
  faults_ = (faults_ & kLatching) | f;

  constexpr std::uint32_t kStopping = kFaultEstop | kFaultWheelUnhealthy |
                                      kFaultBusTimeout | kFaultCommandTimeout |
                                      kFaultObstacle | kFaultWheelSlip |
                                      kFaultLocalisationLost;
  const bool cause_present = (faults_ & kStopping) != 0u;
  if (cause_present) {
    stop_required_ = true;
    stop_dwell_ = 0.0;
  } else if (stop_required_) {
    stop_dwell_ += dt;
    if (stop_dwell_ >= p_.min_stop_time) {
      stop_required_ = false;
      stop_dwell_ = 0.0;
    }
  }
  if (stop_required_) speed_scale_ = 0.0;
}

void SafetyMonitor::reset() {
  speed_scale_ = 1.0;
  since_command_ = 0.0;
  since_bus_ = 0.0;
  stop_required_ = false;
  in_protective_stop_ = false;
  stop_dwell_ = 0.0;
  faults_ = kFaultNone;
}

}  // namespace odc
