"""Python-side tests.

These cover the binding surface and the properties that are easier to state in
Python than in C++ (round-trips over random twists, statistical behaviour of
the estimator). The control logic itself is tested in cpp/tests/test_core.cpp
against the same compiled objects.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import omni_drive_core as odc  # noqa: E402
from odc_sim import scenarios  # noqa: E402
from odc_sim.plant import MecanumPlant, PlantParams  # noqa: E402

# -----------------------------------------------------------------------------
# Kinematics
# -----------------------------------------------------------------------------

def kin() -> odc.MecanumKinematics:
    g = odc.ChassisGeometry()
    g.wheel_radius, g.half_wheelbase, g.half_track = 0.0625, 0.30, 0.25
    return odc.MecanumKinematics(g)


@pytest.mark.parametrize("seed", range(20))
def test_forward_inverse_round_trip_over_random_twists(seed):
    rng = np.random.default_rng(seed)
    k = kin()
    vx, vy, wz = rng.uniform(-1.5, 1.5, 3)
    back = k.forward(k.inverse(odc.Twist2D(vx, vy, wz)))
    assert back.vx == pytest.approx(vx, abs=1e-12)
    assert back.vy == pytest.approx(vy, abs=1e-12)
    assert back.wz == pytest.approx(wz, abs=1e-12)


@pytest.mark.parametrize("seed", range(20))
def test_residual_is_always_parallel_to_the_null_direction(seed):
    """The single most important structural property of this kinematics.

    Four wheels, three degrees of freedom, one scalar of redundancy. The
    residual therefore lives on a line and cannot identify a wheel.
    """
    rng = np.random.default_rng(seed)
    k = kin()
    w = np.array(k.inverse(odc.Twist2D(*rng.uniform(-1, 1, 3))))
    w += rng.normal(0, 2.0, 4)  # arbitrary corruption

    res = np.array(k.slip_residual(list(w)))
    e = odc.MecanumKinematics.consistency_error(list(w))
    np.testing.assert_allclose(res, e * np.array([0.5, 0.5, -0.5, -0.5]), atol=1e-12)
    # Equal magnitudes: ranking them is reading a four-way tie.
    assert np.allclose(np.abs(res), abs(res[0]), atol=1e-12)


def test_plant_and_controller_agree_on_geometry():
    """A sign or lever-arm mismatch here silently degrades everything."""
    p = PlantParams()
    plant = MecanumPlant(p)
    k = kin()
    for twist in [(1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (0.3, -0.4, 0.5)]:
        np.testing.assert_allclose(
            plant.wheel_speeds_from_twist(*twist),
            np.array(k.inverse(odc.Twist2D(*twist))),
            atol=1e-12,
        )


# -----------------------------------------------------------------------------
# Estimator
# -----------------------------------------------------------------------------

def test_ekf_rejects_outliers_but_accepts_plausible_fixes():
    ekf = odc.PoseEkf(odc.PoseEkfParams())
    ekf.reset(odc.Pose2D())
    for _ in range(200):
        ekf.predict(odc.Twist2D(0.5, 0, 0), 0.0, 0.005, False)

    good = odc.PoseFix()
    good.valid, good.pose = True, odc.Pose2D(0.52, 0.01, 0.0)
    assert ekf.update(good)

    bad = odc.PoseFix()
    bad.valid, bad.pose = True, odc.Pose2D(40.0, -20.0, 1.5)
    bad.std_xy, bad.std_theta = 0.02, 0.01
    assert not ekf.update(bad)
    assert abs(ekf.pose().x - 0.52) < 0.1


def test_ekf_nis_averages_near_three_on_consistent_data():
    """NIS is chi-square with 3 DoF, so a well-tuned filter sits near 3."""
    rng = np.random.default_rng(0)
    ekf = odc.PoseEkf(odc.PoseEkfParams())
    ekf.reset(odc.Pose2D())
    x, samples = 0.0, []
    for k in range(400):
        for _ in range(20):
            ekf.predict(odc.Twist2D(0.5, 0, 0), 0.0, 0.005, False)
            x += 0.5 * 0.005
        fix = odc.PoseFix()
        fix.valid = True
        fix.std_xy, fix.std_theta = 0.03, 0.012
        fix.pose = odc.Pose2D(x + rng.normal(0, 0.03), rng.normal(0, 0.03),
                              rng.normal(0, 0.012))
        ekf.update(fix)
        if k > 50:
            samples.append(ekf.last_nis())
    assert 1.0 < float(np.median(samples)) < 6.0


# -----------------------------------------------------------------------------
# Core behaviour
# -----------------------------------------------------------------------------

def test_step_is_deterministic():
    def run():
        core = odc.DriveCore(odc.DriveCoreConfig())
        out_hash = []
        cyc = odc.CycleInput()
        wf = odc.WheelFeedback()
        wf.healthy = [True] * 4
        for i in range(300):
            cyc.wheels = wf
            tp = odc.TrajectoryPoint()
            tp.pose = odc.Pose2D(0.002 * i, 0.001 * i, 0.0)
            cyc.reference = tp
            cyc.reference_fresh = True
            si = odc.SafetyInput()
            si.nearest_obstacle_m = 5.0
            cyc.safety = si
            out = core.step(cyc, 0.005)
            out_hash.append(tuple(out.torque_cmd))
        return out_hash
    assert run() == run()


def test_estop_latches_until_explicit_reset():
    core = odc.DriveCore(odc.DriveCoreConfig())
    cyc = odc.CycleInput()
    wf = odc.WheelFeedback()
    wf.healthy = [True] * 4
    cyc.wheels = wf
    cyc.reference_fresh = True
    si = odc.SafetyInput()
    si.nearest_obstacle_m = 5.0
    si.estop_engaged = True
    cyc.safety = si
    out = core.step(cyc, 0.005)
    assert out.faults & odc.Fault.ESTOP

    si.estop_engaged = False
    cyc.safety = si
    out = core.step(cyc, 0.005)
    assert out.faults & odc.Fault.ESTOP, "must not self-clear"

    core.reset()
    out = core.step(cyc, 0.005)
    assert not (out.faults & odc.Fault.ESTOP)


# -----------------------------------------------------------------------------
# Scenario suite
# -----------------------------------------------------------------------------

@pytest.mark.parametrize("scenario", scenarios.all_scenarios(),
                         ids=lambda s: s.name)
def test_scenario_gates(scenario):
    """Every scenario's acceptance criteria, as individual pytest cases."""
    result = scenario.execute(seed=0)
    failures = []
    for gate in scenario.gates:
        ok, value = gate.evaluate(result)
        if not ok:
            failures.append(f"{gate.name}: {value:.4g} {gate.op} {gate.limit:.4g} "
                            f"{gate.units}")
    assert not failures, "; ".join(failures)


def test_control_step_is_fast_enough_for_200hz():
    """The loop budget is 5 ms; the core should use a tiny fraction of it."""
    result = scenarios.all_scenarios()[0].execute(seed=0)
    p999 = float(np.percentile(result.step_us, 99.9))
    assert p999 < 250.0, f"p99.9 step time {p999:.0f} us"
