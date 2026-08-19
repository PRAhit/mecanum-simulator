// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cmath>

#include "odc/drive_core.hpp"

using namespace odc;

namespace {
ChassisGeometry geom() { return ChassisGeometry{0.0625, 0.30, 0.25}; }

WheelFeedback healthy(const WheelArray<double>& v) {
  WheelFeedback fb{};
  fb.velocity = v;
  for (std::size_t i = 0; i < kNumWheels; ++i) fb.healthy[i] = true;
  return fb;
}
}  // namespace

// -----------------------------------------------------------------------------
// Kinematics
// -----------------------------------------------------------------------------

TEST(Kinematics, ForwardInvertsInverseForArbitraryTwists) {
  MecanumKinematics kin(geom());
  const Twist2D cases[] = {{0.8, 0.0, 0.0},
                           {0.0, 0.6, 0.0},
                           {0.0, 0.0, 1.1},
                           {0.4, -0.3, 0.7},
                           {-0.9, 0.25, -0.5}};
  for (const auto& t : cases) {
    const Twist2D back = kin.forward(kin.inverse(t));
    EXPECT_NEAR(back.vx, t.vx, 1e-12);
    EXPECT_NEAR(back.vy, t.vy, 1e-12);
    EXPECT_NEAR(back.wz, t.wz, 1e-12);
  }
}

TEST(Kinematics, PureStrafeUsesOppositeWheelSigns) {
  MecanumKinematics kin(geom());
  const auto w = kin.inverse(Twist2D{0.0, 1.0, 0.0});
  // Strafing left: FR and RL spin forward, FL and RR spin backward.
  EXPECT_LT(w[kFL], 0.0);
  EXPECT_GT(w[kFR], 0.0);
  EXPECT_GT(w[kRL], 0.0);
  EXPECT_LT(w[kRR], 0.0);
  EXPECT_NEAR(w[kFL], -w[kFR], 1e-12);
}

TEST(Kinematics, ConsistentWheelSetsHaveZeroResidual) {
  MecanumKinematics kin(geom());
  const auto w = kin.inverse(Twist2D{0.5, -0.2, 0.3});
  EXPECT_NEAR(MecanumKinematics::consistencyError(w), 0.0, 1e-12);
  for (double r : kin.slipResidual(w)) EXPECT_NEAR(r, 0.0, 1e-12);
}

// The residual is the projection onto the single left-null direction, so it is
// always parallel to [1, 1, -1, -1]. Locking that down stops anyone from later
// "improving" the slip detector by ranking residual magnitudes, which would be
// reading a four-way tie.
TEST(Kinematics, ResidualIsRankOneAndCannotAttributeAWheel) {
  MecanumKinematics kin(geom());
  auto w = kin.inverse(Twist2D{0.5, -0.2, 0.3});
  w[kRL] += 4.0;  // rear-left spins up freely

  const double e = MecanumKinematics::consistencyError(w);
  EXPECT_NEAR(e, -2.0, 1e-12);

  const auto res = kin.slipResidual(w);
  const double expect[kNumWheels] = {0.5 * e, 0.5 * e, -0.5 * e, -0.5 * e};
  for (std::size_t i = 0; i < kNumWheels; ++i)
    EXPECT_NEAR(res[i], expect[i], 1e-12);

  // Equal magnitudes: no wheel stands out.
  for (std::size_t i = 1; i < kNumWheels; ++i)
    EXPECT_NEAR(std::abs(res[i]), std::abs(res[0]), 1e-12);
}

TEST(Kinematics, ResidualIsOrthogonalToEveryAchievableTwist) {
  MecanumKinematics kin(geom());
  auto w = kin.inverse(Twist2D{0.7, 0.1, -0.4});
  w[kFL] += 3.0;
  const auto res = kin.slipResidual(w);

  // The residual must be invisible to the forward map, otherwise the
  // least-squares odometry would be absorbing slip into the pose estimate.
  const Twist2D leak = kin.forward(res);
  EXPECT_NEAR(leak.vx, 0.0, 1e-12);
  EXPECT_NEAR(leak.vy, 0.0, 1e-12);
  EXPECT_NEAR(leak.wz, 0.0, 1e-12);
}

// -----------------------------------------------------------------------------
// Wheel controller
// -----------------------------------------------------------------------------

TEST(WheelController, ConvergesOnAFirstOrderPlant) {
  WheelController c{WheelControlParams{}};
  double omega = 0.0;
  const double dt = 0.005, inertia = 0.02, damping = 0.01;
  for (int i = 0; i < 2000; ++i) {
    const double tau = c.update(20.0, omega, dt);
    omega += (tau - damping * omega) / inertia * dt;
  }
  EXPECT_NEAR(omega, 20.0, 0.05);
}

TEST(WheelController, NeverExceedsTheTorqueLimit) {
  WheelControlParams p{};
  p.torque_limit = 1.0;
  WheelController c{p};
  for (int i = 0; i < 4000; ++i) {
    const double tau = c.update(100.0, 0.0, 0.005);  // stalled wheel
    EXPECT_LE(std::abs(tau), p.torque_limit + 1e-9);
  }
  EXPECT_TRUE(c.saturated());
}

TEST(WheelController, AntiWindupBoundsTheIntegratorDuringAStall) {
  const auto stall_integrator = [](double aw_gain) {
    WheelControlParams p{};
    p.torque_limit = 1.0;
    p.anti_windup_gain = aw_gain;
    WheelController c{p};
    for (int i = 0; i < 4000; ++i) c.update(100.0, 0.0, 0.005);
    return std::abs(c.integrator());
  };

  const double without = stall_integrator(0.0);
  const double with = stall_integrator(WheelControlParams{}.anti_windup_gain);

  EXPECT_GT(without, 1000.0) << "sanity: an unprotected integrator runs away";
  EXPECT_LT(with, 0.01 * without);
  // The contract that matters: after an unbounded stall the integrator sits
  // within a small multiple of the actuator limit, so the controller can
  // reverse almost immediately once the wheel frees up.
  EXPECT_LT(with, 10.0 * 1.0 /* torque_limit */);
}

TEST(WheelController, SlewLimitsTheSetpoint) {
  WheelControlParams p{};
  p.accel_limit = 10.0;
  WheelController c{p};
  c.update(100.0, 0.0, 0.01);
  EXPECT_NEAR(c.rampedSetpoint(), 0.1, 1e-12);
}

// -----------------------------------------------------------------------------
// EKF
// -----------------------------------------------------------------------------

TEST(PoseEkf, DeadReckonsAStraightLine) {
  PoseEkf ekf{PoseEkfParams{}};
  ekf.reset(Pose2D{});
  for (int i = 0; i < 1000; ++i)
    ekf.predict(Twist2D{1.0, 0.0, 0.0}, 0.0, 0.001, false);
  EXPECT_NEAR(ekf.pose().x, 1.0, 1e-9);
  EXPECT_NEAR(ekf.pose().y, 0.0, 1e-9);
}

TEST(PoseEkf, StrafeWhileRotatedGoesTheRightWay) {
  PoseEkf ekf{PoseEkfParams{}};
  ekf.reset(Pose2D{0.0, 0.0, M_PI / 2.0});
  for (int i = 0; i < 1000; ++i)
    ekf.predict(Twist2D{1.0, 0.0, 0.0}, 0.0, 0.001, false);
  // Facing +y, driving "forward" must move along +y.
  EXPECT_NEAR(ekf.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(ekf.pose().y, 1.0, 1e-9);
}

TEST(PoseEkf, AbsoluteFixPullsTheEstimateIn) {
  PoseEkf ekf{PoseEkfParams{}};
  ekf.reset(Pose2D{});
  for (int i = 0; i < 500; ++i)
    ekf.predict(Twist2D{1.0, 0.0, 0.0}, 0.0, 0.002, false);

  PoseFix fix{};
  fix.valid = true;
  fix.pose = Pose2D{1.05, 0.02, 0.0};
  EXPECT_TRUE(ekf.update(fix));
  EXPECT_GT(ekf.pose().x, 1.0);
  EXPECT_NEAR(ekf.timeSinceFix(), 0.0, 1e-12);
}

TEST(PoseEkf, RejectsAnOutlierFixAndFlagsLostLocalisation) {
  PoseEkfParams p{};
  p.lost_timeout_s = 0.5;
  PoseEkf ekf{p};
  ekf.reset(Pose2D{});
  for (int i = 0; i < 100; ++i)
    ekf.predict(Twist2D{0.0, 0.0, 0.0}, 0.0, 0.01, false);

  PoseFix bad{};
  bad.valid = true;
  bad.pose = Pose2D{50.0, -30.0, 2.0};  // scan matcher latched onto a wall
  bad.std_xy = 0.02;
  bad.std_theta = 0.01;
  EXPECT_FALSE(ekf.update(bad));
  EXPECT_LT(std::abs(ekf.pose().x), 0.1);
  EXPECT_TRUE(ekf.localisationLost());
}

TEST(PoseEkf, EstimatesGyroBias) {
  PoseEkf ekf{PoseEkfParams{}};
  ekf.reset(Pose2D{});
  const double true_bias = 0.02;

  // Robot is physically still; the gyro insists it is turning.
  for (int cycle = 0; cycle < 400; ++cycle) {
    for (int i = 0; i < 50; ++i)
      ekf.predict(Twist2D{}, true_bias, 0.002, false);
    PoseFix fix{};
    fix.valid = true;
    fix.pose = Pose2D{0.0, 0.0, 0.0};
    fix.std_xy = 0.02;
    fix.std_theta = 0.01;
    ekf.update(fix);
  }
  EXPECT_NEAR(ekf.gyroBias(), true_bias, 5e-3);
}

// -----------------------------------------------------------------------------
// Slip detector
// -----------------------------------------------------------------------------

namespace {
WheelArray<double> evenCurrents(double a) { return {a, a, a, a}; }

/// Drive the detector to steady state with a fixed input.
void soak(SlipDetector& d, const MecanumKinematics& kin,
          const WheelArray<double>& w, const WheelArray<double>& amps,
          double gyro, int cycles = 200, double nis = 3.0) {
  // A fix arrives every 20th cycle: 10 Hz localisation against a 200 Hz loop.
  for (int i = 0; i < cycles; ++i) {
    const bool fresh = (i % 20) == 0;
    d.update(kin, w, amps, gyro, nis, fresh, 0.005);
  }
}
}  // namespace

TEST(SlipDetector, IgnoresCleanData) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  soak(d, kin, w, evenCurrents(1.0), 0.0, 400);
  EXPECT_FALSE(d.slipping());
  EXPECT_EQ(d.channels(), kChannelNone);
  EXPECT_EQ(d.worstWheel(), kUnknownWheel);
}

TEST(SlipDetector, KinematicChannelCatchesAWheelThatCannotTrack) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  w[kRL] -= 6.0;  // dragged / jammed: it fails to follow its setpoint

  auto amps = evenCurrents(1.2);
  amps[kRL] = 4.0;  // and pulls hard against the obstruction

  soak(d, kin, w, amps, 0.0);
  EXPECT_TRUE(d.slipping());
  EXPECT_TRUE(d.channels() & kChannelKinematic);
  EXPECT_EQ(d.worstWheel(), static_cast<std::size_t>(kRL));
}

// The load-bearing test for this whole module. Under per-wheel velocity
// control the setpoints come FROM inverse kinematics, so four well-tracking
// wheels satisfy the consistency constraint no matter how much the platform
// is sliding. Kinematics alone is structurally blind here.
TEST(SlipDetector, KinematicChannelIsBlindToTractionLoss) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});  // perfectly consistent

  auto amps = evenCurrents(2.5);
  amps[kFR] = 0.2;  // front-right is spinning on oil, transmitting nothing

  soak(d, kin, w, amps, 0.0);
  EXPECT_NEAR(d.kinematicError(), 0.0, 1e-9)
      << "the kinematic statistic cannot see this failure at all";
  EXPECT_FALSE(d.channels() & kChannelKinematic);
}

TEST(SlipDetector, InnovationChannelCatchesTractionLoss) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});  // wheels look perfect

  // The scan matcher keeps insisting the robot is not where odometry says.
  soak(d, kin, w, evenCurrents(1.0), 0.0, 400, /*nis=*/40.0);
  EXPECT_TRUE(d.slipping());
  EXPECT_TRUE(d.channels() & kChannelOdometry);
  EXPECT_EQ(d.worstWheel(), kUnknownWheel)
      << "sliding platform, all wheels tracking: nothing singles one out";
}

TEST(SlipDetector, InnovationChannelToleratesNormalEstimatorNoise) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  // NIS is chi-square with 3 DoF: it averages 3 and occasionally spikes.
  const double samples[] = {1.2, 4.8, 2.1, 11.9, 0.7, 3.4, 2.9, 6.1};
  for (int i = 0; i < 400; ++i) {
    const bool fresh = (i % 20) == 0;
    d.update(kin, w, evenCurrents(1.0), 0.0, samples[(i / 20) % 8], fresh,
             0.005);
  }
  EXPECT_FALSE(d.channels() & kChannelOdometry);
}

// Regression guard. An earlier design compared each wheel's current against
// the fleet median. That is wrong for a mecanum base: the four wheels carry
// genuinely different loads whenever the platform moves diagonally, so the
// statistic fired constantly on healthy hardware.
TEST(SlipDetector, WheelsUnderUnequalButHealthyLoadDoNotTrip) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto w = kin.inverse(Twist2D{0.5, 0.4, 0.2});  // diagonal + spin
  const WheelArray<double> amps{3.9, 0.4, 2.8, 1.1};   // a 3.5 A spread
  soak(d, kin, w, amps, 0.2);
  EXPECT_FALSE(d.slipping());
}

TEST(SlipDetector, GyroChannelCatchesRotationTheWheelsDoNotReport) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});  // wheels say wz = 0
  soak(d, kin, w, evenCurrents(1.0), 0.6);             // gyro says otherwise
  EXPECT_TRUE(d.slipping());
  EXPECT_TRUE(d.channels() & kChannelGyro);
}

TEST(SlipDetector, RefusesToNameAWheelWhenCurrentsAreAmbiguous) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  w[kFR] += 6.0;
  soak(d, kin, w, evenCurrents(1.2), 0.0);
  EXPECT_TRUE(d.slipping()) << "detection still works without current";
  EXPECT_EQ(d.worstWheel(), kUnknownWheel) << "but it must not guess";
}

TEST(SlipDetector, DoesNotTripOnAOneCycleGlitch) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto clean = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  auto glitch = clean;
  glitch[kRR] += 30.0;

  const auto amps = evenCurrents(1.0);
  soak(d, kin, clean, amps, 0.0, 50);
  d.update(kin, glitch, amps, 0.0, 3.0, false, 0.005);  // one bad CAN frame
  EXPECT_FALSE(d.slipping());
}

TEST(SlipDetector, HysteresisPreventsChatterAtTheThreshold) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const auto amps = evenCurrents(1.0);

  auto w = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  w[kFR] += 6.0;
  soak(d, kin, w, amps, 0.0);
  ASSERT_TRUE(d.channels() & kChannelKinematic);

  auto w2 = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  w2[kFR] += 1.6;  // error 0.8: between exit (0.5) and enter (1.2)
  soak(d, kin, w2, amps, 0.0);
  EXPECT_TRUE(d.channels() & kChannelKinematic) << "must stay latched";

  auto w3 = kin.inverse(Twist2D{0.8, 0.0, 0.0});
  w3[kFR] += 0.4;
  soak(d, kin, w3, amps, 0.0);
  EXPECT_FALSE(d.channels() & kChannelKinematic) << "must clear below exit";
}

TEST(SlipDetector, StandingStillNeverAccusesAnyone) {
  MecanumKinematics kin(geom());
  SlipDetector d{SlipDetectorParams{}};
  const WheelArray<double> stopped{0.0, 0.0, 0.0, 0.0};
  const WheelArray<double> weird{5.0, 0.1, 0.1, 0.1};  // holding torque
  soak(d, kin, stopped, weird, 0.0);
  EXPECT_FALSE(d.slipping());
}

// -----------------------------------------------------------------------------
// Safety monitor
// -----------------------------------------------------------------------------

TEST(SafetyMonitor, RampsSpeedThroughTheWarningField) {
  SafetyParams p{};
  SafetyMonitor s{p};
  const auto fb = healthy({0, 0, 0, 0});

  SafetyInput in{};
  in.nearest_obstacle_m = 2.0;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_NEAR(s.speedScale(), 1.0, 1e-9);

  in.nearest_obstacle_m = 0.5 * (p.stop_distance + p.slow_distance);
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_NEAR(s.speedScale(), 0.5, 1e-6);

  in.nearest_obstacle_m = 0.20;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_NEAR(s.speedScale(), 0.0, 1e-9);
  EXPECT_TRUE(s.stopRequired());
}

TEST(SafetyMonitor, NeedsExtraClearanceToResume) {
  SafetyParams p{};
  SafetyMonitor s{p};
  const auto fb = healthy({0, 0, 0, 0});
  SafetyInput in{};

  in.nearest_obstacle_m = 0.20;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_TRUE(s.stopRequired());

  // Just past the nominal boundary is not enough.
  in.nearest_obstacle_m = p.stop_distance + 0.5 * p.clear_hysteresis;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_TRUE(s.stopRequired());

  // Enough clearance AND the minimum dwell must both be satisfied.
  in.nearest_obstacle_m = p.stop_distance + 2.0 * p.clear_hysteresis;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_TRUE(s.stopRequired()) << "dwell not yet elapsed";

  for (int i = 0; i < 250; ++i) {  // 1.25 s > min_stop_time
    s.notifyCommand();
    s.update(in, fb, false, false, 0.005);
  }
  EXPECT_FALSE(s.stopRequired());
}

// Regression guard for a chattering limit cycle observed in the wet-strip
// scenario: slip trips a stop, the stop halts the wheels, still wheels report
// no slip, motion resumes, the wheels slip again -- several times a second.
TEST(SafetyMonitor, DoesNotChatterWhenTheCauseFlickers) {
  SafetyParams p{};
  p.min_stop_time = 1.0;
  SafetyMonitor s{p};
  const auto fb = healthy({0, 0, 0, 0});
  SafetyInput in{};
  in.nearest_obstacle_m = 5.0;

  int releases = 0;
  bool was_stopped = false;
  for (int i = 0; i < 400; ++i) {    // 2 s
    const bool slip = (i % 20) < 2;  // cause flickers at 10 Hz
    s.notifyCommand();
    s.update(in, fb, slip, false, 0.005);
    if (was_stopped && !s.stopRequired()) ++releases;
    was_stopped = s.stopRequired();
  }
  EXPECT_LE(releases, 1) << "must not release and re-trip repeatedly";
}

TEST(SafetyMonitor, CommandWatchdogStopsTheRobot) {
  SafetyParams p{};
  SafetyMonitor s{p};
  const auto fb = healthy({0, 0, 0, 0});
  SafetyInput in{};
  in.nearest_obstacle_m = 5.0;

  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_FALSE(s.stopRequired());

  for (int i = 0; i < 100; ++i) s.update(in, fb, false, false, 0.005);
  EXPECT_TRUE(s.stopRequired());
  EXPECT_TRUE(s.faults() & kFaultCommandTimeout);
}

TEST(SafetyMonitor, EstopLatchesUntilReset) {
  SafetyMonitor s{SafetyParams{}};
  const auto fb = healthy({0, 0, 0, 0});
  SafetyInput in{};
  in.nearest_obstacle_m = 5.0;
  in.estop_engaged = true;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_TRUE(s.faults() & kFaultEstop);

  in.estop_engaged = false;
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_TRUE(s.faults() & kFaultEstop) << "e-stop must not self-clear";

  s.reset();
  s.notifyCommand();
  s.update(in, fb, false, false, 0.005);
  EXPECT_FALSE(s.faults() & kFaultEstop);
}

// -----------------------------------------------------------------------------
// DriveCore integration
// -----------------------------------------------------------------------------

TEST(DriveCore, ProducesZeroTorqueWhenIdleAndAtRest) {
  DriveCore core{DriveCoreConfig{}};
  CycleInput in{};
  in.wheels = healthy({0, 0, 0, 0});
  in.reference_fresh = true;
  in.safety.nearest_obstacle_m = 5.0;

  ControlOutput out{};
  for (int i = 0; i < 20; ++i) out = core.step(in, 0.005);
  for (double t : out.torque_cmd) EXPECT_NEAR(t, 0.0, 1e-6);
  EXPECT_EQ(out.state, DriveState::kIdle);
  EXPECT_EQ(out.faults, kFaultNone);
}

TEST(DriveCore, CommandsTorqueTowardsAPositionError) {
  DriveCore core{DriveCoreConfig{}};
  CycleInput in{};
  in.wheels = healthy({0, 0, 0, 0});
  in.reference_fresh = true;
  in.safety.nearest_obstacle_m = 5.0;
  in.reference.pose = Pose2D{2.0, 0.0, 0.0};

  ControlOutput out{};
  for (int i = 0; i < 40; ++i) out = core.step(in, 0.005);

  EXPECT_GT(out.commanded_twist.vx, 0.0);
  for (double t : out.torque_cmd) EXPECT_GT(t, 0.0);
  EXPECT_EQ(out.state, DriveState::kRunning);
}

TEST(DriveCore, ObstacleForcesAProtectiveStop) {
  DriveCore core{DriveCoreConfig{}};
  CycleInput in{};
  in.wheels = healthy({0, 0, 0, 0});
  in.reference_fresh = true;
  in.reference.pose = Pose2D{5.0, 0.0, 0.0};
  in.safety.nearest_obstacle_m = 0.15;

  ControlOutput out{};
  for (int i = 0; i < 20; ++i) out = core.step(in, 0.005);

  EXPECT_EQ(out.state, DriveState::kProtectiveStop);
  EXPECT_NEAR(out.commanded_twist.vx, 0.0, 1e-12);
  EXPECT_TRUE(out.faults & kFaultObstacle);
}

TEST(DriveCore, DeadWheelNodeLatchesFaultAndZerosThatActuator) {
  DriveCore core{DriveCoreConfig{}};
  CycleInput in{};
  in.wheels = healthy({0, 0, 0, 0});
  in.reference_fresh = true;
  in.reference.pose = Pose2D{5.0, 0.0, 0.0};
  in.safety.nearest_obstacle_m = 5.0;

  for (int i = 0; i < 20; ++i) core.step(in, 0.005);

  in.wheels.healthy[kRR] = false;
  ControlOutput out{};
  for (int i = 0; i < 5; ++i) out = core.step(in, 0.005);

  EXPECT_EQ(out.state, DriveState::kFault);
  EXPECT_TRUE(out.faults & kFaultWheelUnhealthy);
  EXPECT_NEAR(out.torque_cmd[kRR], 0.0, 1e-12);
}

TEST(DriveCore, IsBitwiseDeterministicAcrossRuns) {
  const auto run = []() {
    DriveCore core{DriveCoreConfig{}};
    CycleInput in{};
    in.wheels = healthy({0, 0, 0, 0});
    in.reference_fresh = true;
    in.safety.nearest_obstacle_m = 5.0;
    double acc = 0.0;
    for (int i = 0; i < 500; ++i) {
      in.reference.pose = Pose2D{0.002 * i, 0.001 * i, 0.0005 * i};
      in.gyro_z = 0.01 * std::sin(0.01 * i);
      const auto out = core.step(in, 0.005);
      for (std::size_t w = 0; w < kNumWheels; ++w) acc += out.torque_cmd[w];
      in.wheels.velocity = out.velocity_ref;  // ideal actuator
    }
    return acc;
  };
  EXPECT_EQ(run(), run());
}
