"""Scenario library.

Each scenario is a named closed-loop test with explicit numeric acceptance
criteria, in the style of an automotive test specification: the scenario owns
its own pass/fail gate, so `tools/run_scenarios.py` can be wired straight into
CI and fail the build on a regression rather than producing a plot for someone
to squint at.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
import omni_drive_core as odc

from .harness import BusModel, LocalisationModel, SimResult, run
from .plant import FL, FR, RL, MecanumPlant

# --- Reference generators ----------------------------------------------------


def hold(x: float, y: float, theta: float = 0.0):
    def ref(_t: float):
        return odc.Pose2D(x, y, theta), odc.Twist2D()
    return ref


def ramp_line(speed: float, heading: float = 0.0, spin: float = 0.0,
              accel: float = 0.5, start: float = 0.0):
    """Acceleration-limited straight line in the world frame.

    A velocity STEP is not a reference a real motion planner would ever emit,
    and asking a tracker limited to 1 m/s^2 to follow one just measures the
    infeasibility of the request. This ramps up at `accel` and holds, so any
    tracking error the gates see belongs to the controller.
    """
    # Jerk-limited (S-curve) velocity profile. A trapezoid has a step in
    # acceleration at each corner, and the resulting transient dominated the
    # peak-error gate even though the controller was behaving. Real motion
    # planners emit S-curves for exactly this reason.
    #
    #   v(tau) = speed * (3 tau^2 - 2 tau^3),  tau = t / t_ramp
    #
    # peak acceleration is 1.5 * speed / t_ramp, so scale t_ramp to respect
    # the requested limit.
    t_ramp = 1.5 * speed / accel

    def profile(t: float) -> tuple[float, float]:
        """Returns (distance, velocity) along the heading."""
        tt = max(t - start, 0.0)
        if tt < t_ramp:
            tau = tt / t_ramp
            v = speed * (3.0 * tau * tau - 2.0 * tau ** 3)
            d = speed * t_ramp * (tau ** 3 - 0.5 * tau ** 4)
            return d, v
        return speed * (tt - 0.5 * t_ramp), speed

    ch, sh = math.cos(heading), math.sin(heading)

    def ref(t: float):
        d, v = profile(t)
        return (odc.Pose2D(d * ch, d * sh, spin * max(t - start, 0.0)),
                odc.Twist2D(v * ch, v * sh, spin))
    return ref


def lemniscate(a: float = 2.0, period: float = 20.0):
    """Figure-of-eight with the body held at constant heading.

    The interesting case for a holonomic base: the platform sweeps a curved
    path while never turning, so every axis of the mecanum allocation is
    exercised continuously and any sign error in the kinematics shows up
    immediately as a drift off the loop.
    """
    w = 2 * math.pi / period

    def ref(t: float):
        s = math.sin(w * t)
        c = math.cos(w * t)
        d = 1.0 + s * s
        x = a * c / d
        y = a * s * c / d
        # Analytic derivative, so the tracker gets a clean feedforward.
        h = 1e-6
        s2, c2 = math.sin(w * (t + h)), math.cos(w * (t + h))
        d2 = 1.0 + s2 * s2
        vx = (a * c2 / d2 - x) / h
        vy = (a * s2 * c2 / d2 - y) / h
        return odc.Pose2D(x, y, 0.0), odc.Twist2D(vx, vy, 0.0)
    return ref


# --- Scenario definition -----------------------------------------------------


@dataclass
class Gate:
    name: str
    check: Callable[[SimResult], float]
    limit: float
    op: str = "<="
    units: str = ""

    def evaluate(self, r: SimResult) -> tuple[bool, float]:
        v = float(self.check(r))
        ok = v <= self.limit if self.op == "<=" else v >= self.limit
        return ok, v


@dataclass
class Scenario:
    name: str
    description: str
    duration: float
    reference: Callable
    gates: list[Gate]
    obstacle: Callable[[float], float] | None = None
    estop: Callable[[float], bool] | None = None
    on_step: Callable | None = None
    bus: BusModel = field(default_factory=BusModel)
    localisation: LocalisationModel = field(default_factory=LocalisationModel)
    plant_factory: Callable[[], MecanumPlant] | None = None

    def execute(self, seed: int = 0) -> SimResult:
        plant = self.plant_factory() if self.plant_factory else MecanumPlant(seed=seed)
        # Place the platform on its own reference at t=0. Otherwise every gate
        # is dominated by the initial-condition transient rather than by the
        # controller's steady behaviour.
        p0, _ = self.reference(0.0)
        plant.state.x, plant.state.y, plant.state.theta = p0.x, p0.y, p0.theta
        return run(
            self.duration,
            self.reference,
            plant=plant,
            bus=self.bus,
            localisation=self.localisation,
            obstacle=self.obstacle,
            estop=self.estop,
            on_step=self.on_step,
            seed=seed,
        )


# --- Metric helpers ----------------------------------------------------------

def settled(r: SimResult, after: float = 2.0) -> np.ndarray:
    return r.t >= after


def rms_tracking(after: float = 2.0):
    def f(r: SimResult) -> float:
        m = settled(r, after)
        return float(np.sqrt(np.mean(r.position_error[m] ** 2)))
    return f


def peak_tracking(after: float = 2.0):
    def f(r: SimResult) -> float:
        return float(np.max(r.position_error[settled(r, after)]))
    return f


def rms_estimator(after: float = 2.0):
    def f(r: SimResult) -> float:
        m = settled(r, after)
        return float(np.sqrt(np.mean(r.estimator_error[m] ** 2)))
    return f


def worst_step_us(r: SimResult) -> float:
    return float(np.percentile(r.step_us, 99.9))


def fault_latency(bit: int, onset: float):
    """Seconds between a fault being injected and the core raising its flag."""
    def f(r: SimResult) -> float:
        after = r.t >= onset
        raised = (r.faults & bit) != 0
        idx = np.nonzero(after & raised)[0]
        return float(r.t[idx[0]] - onset) if idx.size else 1e9
    return f


def truth_slip_onset(r: SimResult, threshold: float = 0.6,
                     min_cycles: int = 20) -> float:
    """First instant the PLANT's wheels genuinely diverge, from ground truth.

    Timing detection from when the command was issued conflates controller
    latency with however long the physics took to break traction, so this
    anchors the measurement to the event itself.

    The event must be SUSTAINED for `min_cycles` (0.1 s at 200 Hz). Without
    that, a single-cycle startup transient anywhere in the run gets mistaken
    for the onset and the reported latency becomes meaningless -- one seed in
    eight produced a 9-cycle blip at t=0.1 s and turned a healthy 0.34 s
    detection into an apparent 3.7 s miss.
    """
    over = np.abs(r.consistency) > threshold
    if over.size < min_cycles:
        return float("inf")
    # Sliding-window all(): index i qualifies if cycles [i, i+min_cycles) are
    # all above threshold.
    win = np.convolve(over.astype(int), np.ones(min_cycles, dtype=int), "valid")
    idx = np.nonzero(win == min_cycles)[0]
    return float(r.t[idx[0]]) if idx.size else float("inf")


def detection_latency(bit: int, threshold: float = 0.6):
    """Seconds from real slip onset to the core raising the flag."""
    def f(r: SimResult) -> float:
        onset = truth_slip_onset(r, threshold)
        if not np.isfinite(onset):
            return 1e9
        raised = np.nonzero(((r.faults & bit) != 0) & (r.t >= onset))[0]
        return float(r.t[raised[0]] - onset) if raised.size else 1e9
    return f


def protective_stop_entries(r: SimResult) -> float:
    """How many times the core ENTERED a protective stop.

    A correct response to one hazard is one stop. A high count means the
    machine is chattering: stopping, releasing because the stopped wheels look
    healthy, moving, and tripping again.
    """
    stopped = r.state == int(odc.DriveState.PROTECTIVE_STOP)
    return float(np.count_nonzero(np.diff(stopped.astype(int)) == 1))


def max_speed_after(onset: float):
    def f(r: SimResult) -> float:
        m = r.t >= onset
        return float(np.max(np.abs(r.cmd_twist[m]).sum(axis=1)))
    return f


# --- The scenarios -----------------------------------------------------------

def _wet_patch(wheels, onset: float, mu: float = 0.05):
    """Drop the friction coefficient under one or more wheels."""
    if isinstance(wheels, int):
        wheels = (wheels,)

    def hook(t: float, plant: MecanumPlant) -> None:
        for wh in wheels:
            plant.set_friction(wh, mu if t >= onset else plant.p.mu_nominal)
    return hook


def _jam(wheel: int, onset: float):
    def hook(t: float, plant: MecanumPlant) -> None:
        plant.block_wheel(wheel, t >= onset)
    return hook


def all_scenarios() -> list[Scenario]:
    return [
        Scenario(
            name="straight_line",
            description="1 m/s forward for 10 s. Baseline tracking and timing.",
            duration=10.0,
            reference=ramp_line(1.0),
            gates=[
                Gate("rms_tracking_error", rms_tracking(), 0.05, units="m"),
                Gate("peak_tracking_error", peak_tracking(), 0.12, units="m"),
                Gate("rms_estimator_error", rms_estimator(), 0.05, units="m"),
                Gate("step_time_p99.9", worst_step_us, 250.0, units="us"),
            ],
        ),
        Scenario(
            name="lateral_strafe",
            description=("Pure sideways translation, heading locked. "
                         "A manoeuvre only a holonomic base can do."),
            duration=10.0,
            reference=ramp_line(0.6, heading=math.pi / 2),
            gates=[
                Gate("rms_tracking_error", rms_tracking(), 0.06, units="m"),
                Gate("heading_drift",
                     lambda r: float(np.max(np.abs(r.true_pose[:, 2]))),
                     0.15, units="rad"),
            ],
        ),
        Scenario(
            name="figure_eight",
            description="Curved path at constant heading -- all three axes active at once.",
            duration=25.0,
            reference=lemniscate(),
            gates=[
                Gate("rms_tracking_error", rms_tracking(3.0), 0.10, units="m"),
                Gate("rms_estimator_error", rms_estimator(3.0), 0.06, units="m"),
            ],
        ),
        Scenario(
            name="spin_in_place",
            description="Rotate at 0.8 rad/s without translating. Checks yaw allocation.",
            duration=12.0,
            reference=lambda t: (odc.Pose2D(0.0, 0.0, 0.8 * t),
                                 odc.Twist2D(0.0, 0.0, 0.8)),
            gates=[
                Gate("position_drift",
                     lambda r: float(np.max(np.linalg.norm(r.true_pose[:, :2], axis=1))),
                     0.20, units="m"),
            ],
        ),
        Scenario(
            name="traction_loss_wet_strip",
            description=(
                "A wet strip runs across the aisle: both left-side wheels lose "
                "grip while the loaded cart is launched at 0.9 m/s^2. Demand "
                "(19 N/wheel) exceeds what mu=0.05 can transmit (10 N), so the "
                "left side breaks away and the platform yaws."
            ),
            duration=9.0,
            reference=ramp_line(1.1, accel=0.9, start=3.0),
            on_step=_wet_patch((FL, RL), 0.0),
            gates=[
                Gate("detection_latency_from_real_onset",
                     detection_latency(odc.Fault.WHEEL_SLIP), 0.45, units="s"),
                Gate("heading_deviation",
                     lambda r: float(np.max(np.abs(r.true_pose[r.t >= 3.0, 2]))),
                     0.25, units="rad"),
                Gate("protective_stop_entries", protective_stop_entries, 3.0,
                     units="entries"),
            ],
        ),
        Scenario(
            name="traction_loss_single_wheel_absorbed",
            description=(
                "The same hard start with only ONE wheel on the oil. Three "
                "healthy wheels take up the slack, so the platform degrades "
                "gracefully -- estimator error stays inside 5 cm. Documents a "
                "real property of a four-wheel holonomic base rather than "
                "pretending every traction event is an emergency."
            ),
            duration=9.0,
            reference=ramp_line(1.1, accel=0.9, start=3.0),
            on_step=_wet_patch(FR, 0.0),
            gates=[
                Gate("estimator_error", rms_estimator(3.0), 0.05, units="m"),
                Gate("peak_tracking_error", peak_tracking(3.0), 0.15, units="m"),
            ],
        ),
        Scenario(
            name="wheel_jam",
            description="Rear-left bearing seizes at t=3 s.",
            duration=8.0,
            reference=ramp_line(0.8),
            on_step=_jam(RL, 3.0),
            gates=[
                Gate("detection_latency_from_real_onset",
                     detection_latency(odc.Fault.WHEEL_SLIP), 0.45, units="s"),
            ],
        ),
        Scenario(
            name="obstacle_intrusion",
            description=("A person steps into the warning field at t=4 s and the "
                         "protective field at t=6 s."),
            duration=10.0,
            reference=ramp_line(1.0),
            obstacle=lambda t: 5.0 if t < 4.0 else (0.9 if t < 6.0 else 0.2),
            gates=[
                Gate("stop_latency",
                     fault_latency(odc.Fault.OBSTACLE, 6.0), 0.05, units="s"),
                Gate("speed_after_stop", max_speed_after(6.2), 1e-9, units="m/s"),
                Gate("speed_scale_in_warning_field",
                     lambda r: float(np.max(r.speed_scale[(r.t > 4.2) & (r.t < 5.8)])),
                     0.60),
            ],
        ),
        Scenario(
            name="bus_dropout",
            description="Wheel bus goes silent for 300 ms at t=5 s.",
            duration=10.0,
            reference=ramp_line(0.9),
            bus=BusModel(dropout_windows=[(5.0, 5.3)]),
            gates=[
                Gate("bus_fault_latency",
                     fault_latency(odc.Fault.WHEEL_UNHEALTHY, 5.0), 0.05, units="s"),
                Gate("speed_during_dropout",
                     lambda r: float(np.max(
                         np.abs(r.cmd_twist[(r.t > 5.15) & (r.t < 5.3)]).sum(axis=1))),
                     1e-9, units="m/s"),
            ],
        ),
        Scenario(
            name="localisation_blackout",
            description="Featureless aisle: no pose fixes for 5 s from t=3 s.",
            duration=12.0,
            reference=ramp_line(0.8),
            localisation=LocalisationModel(blackout_windows=[(3.0, 8.0)]),
            gates=[
                Gate("lost_fault_latency",
                     fault_latency(odc.Fault.LOCALISATION_LOST, 3.0), 5.0, units="s"),
                Gate("dead_reckoning_drift",
                     lambda r: float(np.max(
                         r.estimator_error[(r.t > 3.0) & (r.t < 7.0)])),
                     0.50, units="m"),
            ],
        ),
        Scenario(
            name="estop",
            description="Hardware emergency stop asserted at t=4 s and released at t=6 s.",
            duration=9.0,
            reference=ramp_line(1.0),
            estop=lambda t: 4.0 <= t < 6.0,
            gates=[
                Gate("estop_latency", fault_latency(odc.Fault.ESTOP, 4.0), 0.02, units="s"),
                Gate("no_self_clear", max_speed_after(6.5), 1e-9, units="m/s"),
            ],
        ),
    ]
