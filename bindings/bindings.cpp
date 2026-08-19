// SPDX-License-Identifier: MIT
//
// pybind11 bindings.
//
// The point of this file is that the Python simulator drives the *same
// compiled objects* that would run on the wheel node. There is no reference
// implementation in Python to drift out of sync with the C++ -- if a scenario
// passes in simulation, it passed against production control code.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "odc/drive_core.hpp"

namespace py = pybind11;
using namespace odc;

PYBIND11_MODULE(omni_drive_core, m) {
  m.doc() =
      "Real-time holonomic AMR control core (C++), exposed for SIL testing";

  m.attr("NUM_WHEELS") = py::int_(static_cast<int>(kNumWheels));
  m.attr("FL") = py::int_(static_cast<int>(kFL));
  m.attr("FR") = py::int_(static_cast<int>(kFR));
  m.attr("RL") = py::int_(static_cast<int>(kRL));
  m.attr("RR") = py::int_(static_cast<int>(kRR));

  m.def("wrap_angle", &wrapAngle, py::arg("angle"));

  // --- Value types -----------------------------------------------------------
  py::class_<Twist2D>(m, "Twist2D")
      .def(py::init<>())
      .def(py::init([](double vx, double vy, double wz) {
             return Twist2D{vx, vy, wz};
           }),
           py::arg("vx") = 0.0, py::arg("vy") = 0.0, py::arg("wz") = 0.0)
      .def_readwrite("vx", &Twist2D::vx)
      .def_readwrite("vy", &Twist2D::vy)
      .def_readwrite("wz", &Twist2D::wz)
      .def("__repr__", [](const Twist2D& t) {
        return "Twist2D(vx=" + std::to_string(t.vx) +
               ", vy=" + std::to_string(t.vy) + ", wz=" + std::to_string(t.wz) +
               ")";
      });

  py::class_<Pose2D>(m, "Pose2D")
      .def(py::init<>())
      .def(py::init(
               [](double x, double y, double th) { return Pose2D{x, y, th}; }),
           py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("theta") = 0.0)
      .def_readwrite("x", &Pose2D::x)
      .def_readwrite("y", &Pose2D::y)
      .def_readwrite("theta", &Pose2D::theta)
      .def("__repr__", [](const Pose2D& p) {
        return "Pose2D(x=" + std::to_string(p.x) +
               ", y=" + std::to_string(p.y) +
               ", theta=" + std::to_string(p.theta) + ")";
      });

  py::class_<WheelFeedback>(m, "WheelFeedback")
      .def(py::init<>())
      .def_readwrite("velocity", &WheelFeedback::velocity)
      .def_readwrite("current", &WheelFeedback::current)
      .def_readwrite("healthy", &WheelFeedback::healthy);

  py::class_<PoseFix>(m, "PoseFix")
      .def(py::init<>())
      .def_readwrite("pose", &PoseFix::pose)
      .def_readwrite("std_xy", &PoseFix::std_xy)
      .def_readwrite("std_theta", &PoseFix::std_theta)
      .def_readwrite("valid", &PoseFix::valid);

  py::class_<SafetyInput>(m, "SafetyInput")
      .def(py::init<>())
      .def_readwrite("nearest_obstacle_m", &SafetyInput::nearest_obstacle_m)
      .def_readwrite("estop_engaged", &SafetyInput::estop_engaged)
      .def_readwrite("bus_ok", &SafetyInput::bus_ok);

  py::class_<TrajectoryPoint>(m, "TrajectoryPoint")
      .def(py::init<>())
      .def_readwrite("pose", &TrajectoryPoint::pose)
      .def_readwrite("velocity", &TrajectoryPoint::velocity);

  py::enum_<DriveState>(m, "DriveState")
      .value("IDLE", DriveState::kIdle)
      .value("RUNNING", DriveState::kRunning)
      .value("SPEED_LIMITED", DriveState::kSpeedLimited)
      .value("PROTECTIVE_STOP", DriveState::kProtectiveStop)
      .value("FAULT", DriveState::kFault);

  py::module_ faults = m.def_submodule("Fault", "Fault bit flags");
  faults.attr("NONE") = py::int_(static_cast<unsigned>(kFaultNone));
  faults.attr("COMMAND_TIMEOUT") =
      py::int_(static_cast<unsigned>(kFaultCommandTimeout));
  faults.attr("BUS_TIMEOUT") =
      py::int_(static_cast<unsigned>(kFaultBusTimeout));
  faults.attr("WHEEL_UNHEALTHY") =
      py::int_(static_cast<unsigned>(kFaultWheelUnhealthy));
  faults.attr("WHEEL_SLIP") = py::int_(static_cast<unsigned>(kFaultWheelSlip));
  faults.attr("ESTOP") = py::int_(static_cast<unsigned>(kFaultEstop));
  faults.attr("OBSTACLE") = py::int_(static_cast<unsigned>(kFaultObstacle));
  faults.attr("LOCALISATION_LOST") =
      py::int_(static_cast<unsigned>(kFaultLocalisationLost));

  py::class_<ControlOutput>(m, "ControlOutput")
      .def(py::init<>())
      .def_readonly("torque_cmd", &ControlOutput::torque_cmd)
      .def_readonly("velocity_ref", &ControlOutput::velocity_ref)
      .def_readonly("commanded_twist", &ControlOutput::commanded_twist)
      .def_readonly("estimated_pose", &ControlOutput::estimated_pose)
      .def_readonly("estimated_twist", &ControlOutput::estimated_twist)
      .def_readonly("slip_residual", &ControlOutput::slip_residual)
      .def_readonly("slip_channels", &ControlOutput::slip_channels)
      .def_readonly("slip_wheel", &ControlOutput::slip_wheel)
      .def_readonly("state", &ControlOutput::state)
      .def_readonly("faults", &ControlOutput::faults)
      .def_readonly("speed_scale", &ControlOutput::speed_scale);

  // --- Parameter blocks ------------------------------------------------------
  py::class_<ChassisGeometry>(m, "ChassisGeometry")
      .def(py::init<>())
      .def_readwrite("wheel_radius", &ChassisGeometry::wheel_radius)
      .def_readwrite("half_wheelbase", &ChassisGeometry::half_wheelbase)
      .def_readwrite("half_track", &ChassisGeometry::half_track)
      .def("lxy", &ChassisGeometry::lxy);

  py::class_<WheelControlParams>(m, "WheelControlParams")
      .def(py::init<>())
      .def_readwrite("kp", &WheelControlParams::kp)
      .def_readwrite("ki", &WheelControlParams::ki)
      .def_readwrite("kff_viscous", &WheelControlParams::kff_viscous)
      .def_readwrite("kff_coulomb", &WheelControlParams::kff_coulomb)
      .def_readwrite("torque_limit", &WheelControlParams::torque_limit)
      .def_readwrite("accel_limit", &WheelControlParams::accel_limit)
      .def_readwrite("anti_windup_gain", &WheelControlParams::anti_windup_gain);

  py::class_<TrackerParams>(m, "TrackerParams")
      .def(py::init<>())
      .def_readwrite("kp_xy", &TrackerParams::kp_xy)
      .def_readwrite("kp_theta", &TrackerParams::kp_theta)
      .def_readwrite("ki_xy", &TrackerParams::ki_xy)
      .def_readwrite("integral_limit", &TrackerParams::integral_limit)
      .def_readwrite("max_vx", &TrackerParams::max_vx)
      .def_readwrite("max_vy", &TrackerParams::max_vy)
      .def_readwrite("max_wz", &TrackerParams::max_wz)
      .def_readwrite("max_accel", &TrackerParams::max_accel)
      .def_readwrite("max_ang_accel", &TrackerParams::max_ang_accel);

  py::class_<PoseEkfParams>(m, "PoseEkfParams")
      .def(py::init<>())
      .def_readwrite("sigma_v", &PoseEkfParams::sigma_v)
      .def_readwrite("sigma_gyro", &PoseEkfParams::sigma_gyro)
      .def_readwrite("sigma_bias_walk", &PoseEkfParams::sigma_bias_walk)
      .def_readwrite("slip_inflation", &PoseEkfParams::slip_inflation)
      .def_readwrite("chi2_gate", &PoseEkfParams::chi2_gate)
      .def_readwrite("lost_timeout_s", &PoseEkfParams::lost_timeout_s);

  py::class_<SlipDetectorParams>(m, "SlipDetectorParams")
      .def(py::init<>())
      .def_readwrite("kin_enter", &SlipDetectorParams::kin_enter)
      .def_readwrite("kin_exit", &SlipDetectorParams::kin_exit)
      .def_readwrite("nis_enter", &SlipDetectorParams::nis_enter)
      .def_readwrite("nis_exit", &SlipDetectorParams::nis_exit)
      .def_readwrite("nis_confirm_fixes",
                     &SlipDetectorParams::nis_confirm_fixes)
      .def_readwrite("nis_clear_fixes", &SlipDetectorParams::nis_clear_fixes)
      .def_readwrite("gyro_enter", &SlipDetectorParams::gyro_enter)
      .def_readwrite("gyro_exit", &SlipDetectorParams::gyro_exit)
      .def_readwrite("filter_tau", &SlipDetectorParams::filter_tau)
      .def_readwrite("confirm_time", &SlipDetectorParams::confirm_time)
      .def_readwrite("min_speed", &SlipDetectorParams::min_speed)
      .def_readwrite("attribution_deadband",
                     &SlipDetectorParams::attribution_deadband);

  py::module_ ch = m.def_submodule("SlipChannel", "Slip evidence bit flags");
  ch.attr("NONE") = py::int_(static_cast<unsigned>(kChannelNone));
  ch.attr("KINEMATIC") = py::int_(static_cast<unsigned>(kChannelKinematic));
  ch.attr("ODOMETRY") = py::int_(static_cast<unsigned>(kChannelOdometry));
  ch.attr("GYRO") = py::int_(static_cast<unsigned>(kChannelGyro));
  m.attr("UNKNOWN_WHEEL") = py::int_(static_cast<int>(kUnknownWheel));

  py::class_<SafetyParams>(m, "SafetyParams")
      .def(py::init<>())
      .def_readwrite("stop_distance", &SafetyParams::stop_distance)
      .def_readwrite("slow_distance", &SafetyParams::slow_distance)
      .def_readwrite("min_speed_scale", &SafetyParams::min_speed_scale)
      .def_readwrite("command_timeout", &SafetyParams::command_timeout)
      .def_readwrite("bus_timeout", &SafetyParams::bus_timeout)
      .def_readwrite("clear_hysteresis", &SafetyParams::clear_hysteresis)
      .def_readwrite("min_stop_time", &SafetyParams::min_stop_time);

  py::class_<DriveCoreConfig>(m, "DriveCoreConfig")
      .def(py::init<>())
      .def_readwrite("geometry", &DriveCoreConfig::geometry)
      .def_readwrite("wheel", &DriveCoreConfig::wheel)
      .def_readwrite("tracker", &DriveCoreConfig::tracker)
      .def_readwrite("ekf", &DriveCoreConfig::ekf)
      .def_readwrite("slip", &DriveCoreConfig::slip)
      .def_readwrite("safety", &DriveCoreConfig::safety)
      .def_readwrite("control_period", &DriveCoreConfig::control_period);

  py::class_<CycleInput>(m, "CycleInput")
      .def(py::init<>())
      .def_readwrite("wheels", &CycleInput::wheels)
      .def_readwrite("gyro_z", &CycleInput::gyro_z)
      .def_readwrite("pose_fix", &CycleInput::pose_fix)
      .def_readwrite("reference", &CycleInput::reference)
      .def_readwrite("reference_fresh", &CycleInput::reference_fresh)
      .def_readwrite("safety", &CycleInput::safety);

  // --- Components ------------------------------------------------------------
  py::class_<MecanumKinematics>(m, "MecanumKinematics")
      .def(py::init<const ChassisGeometry&>(), py::arg("geometry"))
      .def("inverse", &MecanumKinematics::inverse, py::arg("twist"))
      .def("forward", &MecanumKinematics::forward, py::arg("wheel_velocity"))
      .def_static("consistency_error", &MecanumKinematics::consistencyError,
                  py::arg("wheel_velocity"))
      .def("slip_residual", &MecanumKinematics::slipResidual,
           py::arg("wheel_velocity"));

  py::class_<PoseEkf>(m, "PoseEkf")
      .def(py::init<const PoseEkfParams&>(), py::arg("params"))
      .def("reset", &PoseEkf::reset, py::arg("initial"))
      .def("predict", &PoseEkf::predict, py::arg("odom_twist"),
           py::arg("gyro_z"), py::arg("dt"), py::arg("slip_detected") = false)
      .def("update", &PoseEkf::update, py::arg("fix"))
      .def("pose", &PoseEkf::pose)
      .def("gyro_bias", &PoseEkf::gyroBias)
      .def("position_sigma", &PoseEkf::positionSigma)
      .def("time_since_fix", &PoseEkf::timeSinceFix)
      .def("last_nis", &PoseEkf::lastNis)
      .def("localisation_lost", &PoseEkf::localisationLost);

  py::class_<DriveCore>(m, "DriveCore")
      .def(py::init<const DriveCoreConfig&>(), py::arg("config"))
      .def("step", &DriveCore::step, py::arg("cycle_input"), py::arg("dt"))
      .def("reset", &DriveCore::reset)
      .def("set_pose", &DriveCore::setPose, py::arg("pose"));
}
