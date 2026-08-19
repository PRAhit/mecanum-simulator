"""Software-in-the-loop harness.

Closes the loop between the compiled C++ `DriveCore` and the Python plant at a
fixed 200 Hz, models the wheel bus (latency, dropouts) and the localisation
front-end (10 Hz pose fixes with outliers), and records every signal so the
scenario gates in `tools/analyze.py` have something to judge.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
import omni_drive_core as odc

from .plant import NUM_WHEELS, MecanumPlant, PlantParams


@dataclass
class BusModel:
    """Wheel bus between the main wheel and the three followers."""
    latency_cycles: int = 1     # one control period of transport delay
    dropout_windows: list[tuple[float, float]] = field(default_factory=list)

    def is_down(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.dropout_windows)


@dataclass
class LocalisationModel:
    """Scan matcher / 3D vision front-end."""
    rate_hz: float = 10.0
    std_xy: float = 0.03
    std_theta: float = 0.012
    outlier_rate: float = 0.01   # fraction of fixes that latch onto the wrong wall
    blackout_windows: list[tuple[float, float]] = field(default_factory=list)

    def is_blacked_out(self, t: float) -> bool:
        return any(a <= t < b for a, b in self.blackout_windows)


@dataclass
class SimResult:
    t: np.ndarray
    true_pose: np.ndarray        # (N, 3)
    est_pose: np.ndarray         # (N, 3)
    ref_pose: np.ndarray         # (N, 3)
    cmd_twist: np.ndarray        # (N, 3)
    wheel_omega: np.ndarray      # (N, 4)
    torque: np.ndarray           # (N, 4)
    consistency: np.ndarray      # (N,)
    state: np.ndarray            # (N,) DriveState as int
    faults: np.ndarray           # (N,) bitfield
    speed_scale: np.ndarray      # (N,)
    current: np.ndarray          # (N, 4) measured motor current
    slip_channels: np.ndarray    # (N,) SlipChannel bitmask
    step_us: np.ndarray          # (N,) wall-clock cost of DriveCore::step

    @property
    def position_error(self) -> np.ndarray:
        return np.linalg.norm(self.true_pose[:, :2] - self.ref_pose[:, :2], axis=1)

    @property
    def estimator_error(self) -> np.ndarray:
        return np.linalg.norm(self.true_pose[:, :2] - self.est_pose[:, :2], axis=1)


def default_config() -> odc.DriveCoreConfig:
    cfg = odc.DriveCoreConfig()
    p = PlantParams()
    cfg.geometry.wheel_radius = p.wheel_radius
    cfg.geometry.half_wheelbase = p.half_wheelbase
    cfg.geometry.half_track = p.half_track
    return cfg


def run(
    duration: float,
    reference: Callable[[float], tuple[odc.Pose2D, odc.Twist2D]],
    *,
    config: odc.DriveCoreConfig | None = None,
    plant: MecanumPlant | None = None,
    bus: BusModel | None = None,
    localisation: LocalisationModel | None = None,
    obstacle: Callable[[float], float] | None = None,
    estop: Callable[[float], bool] | None = None,
    on_step: Callable[[float, MecanumPlant], None] | None = None,
    dt: float = 0.005,
    seed: int = 0,
) -> SimResult:
    """Run one closed-loop scenario.

    `on_step` is the fault-injection hook: it is called before every cycle with
    the current time and the plant, so a scenario can drop friction on a wheel
    at t=3s, jam a bearing, or move an obstacle into the protective field.
    """
    import time

    cfg = config or default_config()
    plant = plant or MecanumPlant(seed=seed)
    bus = bus or BusModel()
    loc = localisation or LocalisationModel()
    rng = np.random.default_rng(seed + 977)

    core = odc.DriveCore(cfg)
    x0, y0, th0 = plant.true_pose()
    core.set_pose(odc.Pose2D(x0, y0, th0))

    n = int(round(duration / dt))
    delay: deque = deque([(np.zeros(NUM_WHEELS), np.zeros(NUM_WHEELS))]
                         * max(bus.latency_cycles, 0), maxlen=max(bus.latency_cycles, 1))

    rec = {k: [] for k in
           ("t", "true", "est", "ref", "cmd", "omega", "tau",
            "cons", "state", "faults", "scale", "us", "cur", "chan")}

    fix_period = 1.0 / loc.rate_hz
    next_fix = 0.0
    torque = np.zeros(NUM_WHEELS)

    for k in range(n):
        t = k * dt
        if on_step is not None:
            on_step(t, plant)

        plant.step(torque, dt)

        # --- Wheel bus: quantised encoders and currents, delayed by transport.
        enc, cur = plant.read_encoders(), plant.read_currents()
        if bus.latency_cycles > 0:
            delay.append((enc, cur))
            enc_rx, cur_rx = delay[0]
        else:
            enc_rx, cur_rx = enc, cur

        bus_ok = not bus.is_down(t)

        cyc = odc.CycleInput()
        wf = odc.WheelFeedback()
        # A dead bus means stale data, not zeroed data -- that distinction is
        # exactly what the bus watchdog exists to catch.
        wf.velocity = list(enc_rx if bus_ok else np.zeros(NUM_WHEELS))
        wf.current = list(cur_rx if bus_ok else np.zeros(NUM_WHEELS))
        wf.healthy = [bus_ok] * NUM_WHEELS
        cyc.wheels = wf
        cyc.gyro_z = plant.read_gyro()

        # --- Localisation front-end.
        if t >= next_fix:
            next_fix += fix_period
            if not loc.is_blacked_out(t):
                tx, ty, tth = plant.true_pose()
                fix = odc.PoseFix()
                if rng.random() < loc.outlier_rate:
                    tx += rng.normal(0, 3.0)
                    ty += rng.normal(0, 3.0)
                    tth += rng.normal(0, 1.0)
                else:
                    tx += rng.normal(0, loc.std_xy)
                    ty += rng.normal(0, loc.std_xy)
                    tth += rng.normal(0, loc.std_theta)
                fix.pose = odc.Pose2D(tx, ty, tth)
                fix.std_xy = loc.std_xy
                fix.std_theta = loc.std_theta
                fix.valid = True
                cyc.pose_fix = fix

        # --- Reference and environment.
        ref_pose, ref_vel = reference(t)
        tp = odc.TrajectoryPoint()
        tp.pose = ref_pose
        tp.velocity = ref_vel
        cyc.reference = tp
        cyc.reference_fresh = True

        si = odc.SafetyInput()
        si.nearest_obstacle_m = obstacle(t) if obstacle else 1e9
        si.estop_engaged = estop(t) if estop else False
        si.bus_ok = bus_ok
        cyc.safety = si

        t0 = time.perf_counter()
        out = core.step(cyc, dt)
        step_us = (time.perf_counter() - t0) * 1e6

        torque = np.array(out.torque_cmd)

        tx, ty, tth = plant.true_pose()
        rec["t"].append(t)
        rec["true"].append((tx, ty, tth))
        rec["est"].append((out.estimated_pose.x, out.estimated_pose.y,
                           out.estimated_pose.theta))
        rec["ref"].append((ref_pose.x, ref_pose.y, ref_pose.theta))
        rec["cmd"].append((out.commanded_twist.vx, out.commanded_twist.vy,
                           out.commanded_twist.wz))
        rec["omega"].append(tuple(plant.state.wheel_omega))
        rec["tau"].append(tuple(out.torque_cmd))
        rec["cons"].append(
            odc.MecanumKinematics.consistency_error(list(plant.state.wheel_omega)))
        rec["state"].append(int(out.state))
        rec["faults"].append(int(out.faults))
        rec["scale"].append(out.speed_scale)
        rec["us"].append(step_us)
        rec["cur"].append(tuple(cur))
        rec["chan"].append(int(out.slip_channels))

    return SimResult(
        t=np.array(rec["t"]),
        true_pose=np.array(rec["true"]),
        est_pose=np.array(rec["est"]),
        ref_pose=np.array(rec["ref"]),
        cmd_twist=np.array(rec["cmd"]),
        wheel_omega=np.array(rec["omega"]),
        torque=np.array(rec["tau"]),
        consistency=np.array(rec["cons"]),
        state=np.array(rec["state"]),
        faults=np.array(rec["faults"]),
        speed_scale=np.array(rec["scale"]),
        current=np.array(rec["cur"]),
        slip_channels=np.array(rec["chan"]),
        step_us=np.array(rec["us"]),
    )
