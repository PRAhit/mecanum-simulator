"""Physical plant model for the four-wheel mecanum base.

This is deliberately the *only* place in the repository where robot physics
lives. The control law is entirely on the C++ side; this module exists to
disagree with it -- friction, roller slip, motor lag, encoder quantisation,
gyro bias, bus dropouts. If the controller only ever sees a perfect plant, the
tests prove nothing.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

NUM_WHEELS = 4
FL, FR, RL, RR = 0, 1, 2, 3


@dataclass
class PlantParams:
    # --- Geometry (must match the controller's ChassisGeometry) -------------
    wheel_radius: float = 0.0625
    half_wheelbase: float = 0.30
    half_track: float = 0.25

    # --- Inertias ----------------------------------------------------------
    mass: float = 85.0            # [kg] loaded cart
    inertia_z: float = 12.0       # [kg m^2]
    wheel_inertia: float = 0.02   # [kg m^2] rotor + gearbox, referred to wheel

    # --- Friction ----------------------------------------------------------
    viscous_wheel: float = 0.010  # [Nm/(rad/s)]
    coulomb_wheel: float = 0.045  # [Nm]
    chassis_drag: float = 6.0     # [N/(m/s)]
    chassis_drag_yaw: float = 1.5  # [Nm/(rad/s)]

    # --- Traction ----------------------------------------------------------
    mu_nominal: float = 0.85      # baseline friction coefficient
    slip_stiffness: float = 900.0  # [N/(m/s)] contact patch stiffness

    # --- Sensors -----------------------------------------------------------
    encoder_lsb: float = 2 * np.pi / 4096   # [rad] quantisation
    encoder_noise: float = 0.02             # [rad/s] 1-sigma
    current_noise: float = 0.03             # [A] 1-sigma
    gyro_noise: float = 0.003               # [rad/s] 1-sigma
    gyro_bias: float = 0.012                # [rad/s] constant offset
    torque_constant: float = 0.42           # [Nm/A]

    # --- Actuation ---------------------------------------------------------
    motor_tau: float = 0.012      # [s] current loop lag

    @property
    def lxy(self) -> float:
        return self.half_wheelbase + self.half_track


@dataclass
class PlantState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0     # body frame
    vy: float = 0.0
    wz: float = 0.0
    wheel_omega: np.ndarray = field(default_factory=lambda: np.zeros(NUM_WHEELS))
    motor_torque: np.ndarray = field(default_factory=lambda: np.zeros(NUM_WHEELS))


class MecanumPlant:
    """Semi-implicit Euler simulation of a mecanum base with per-wheel traction.

    Each wheel's roller contact is modelled as a compliant patch: the force it
    can transmit is proportional to the slip velocity between the commanded
    rolling speed and the actual ground speed at that contact, saturated at
    mu * normal_load. Reducing mu on one wheel is therefore all it takes to
    produce a physically honest slip event, rather than injecting a fake
    encoder offset.
    """

    def __init__(self, params: PlantParams | None = None, seed: int = 0):
        self.p = params or PlantParams()
        self.state = PlantState()
        self.rng = np.random.default_rng(seed)
        self.mu = np.full(NUM_WHEELS, self.p.mu_nominal)
        self.wheel_blocked = np.zeros(NUM_WHEELS, dtype=bool)
        # Roller direction signs for a standard X-configuration mecanum set.
        self._sy = np.array([-1.0, 1.0, 1.0, -1.0])
        self._sw = np.array([-1.0, 1.0, -1.0, 1.0])

    # -- kinematics -----------------------------------------------------------
    def wheel_speeds_from_twist(self, vx: float, vy: float, wz: float) -> np.ndarray:
        r, lever = self.p.wheel_radius, self.p.lxy
        return (vx + self._sy * vy + self._sw * lever * wz) / r

    # -- dynamics -------------------------------------------------------------
    def step(self, torque_cmd: np.ndarray, dt: float) -> None:
        p, s = self.p, self.state
        torque_cmd = np.asarray(torque_cmd, dtype=float)

        # Current loop lag.
        alpha = dt / (p.motor_tau + dt)
        s.motor_torque += alpha * (torque_cmd - s.motor_torque)

        # Ground speed at each contact, expressed as an equivalent wheel speed.
        omega_ground = self.wheel_speeds_from_twist(s.vx, s.vy, s.wz)

        # Longitudinal slip velocity at the contact patch [m/s].
        slip_v = (s.wheel_omega - omega_ground) * p.wheel_radius

        normal_load = p.mass * 9.81 / NUM_WHEELS
        f_max = self.mu * normal_load
        traction = np.clip(p.slip_stiffness * slip_v, -f_max, f_max)

        # Wheel rotational dynamics.
        friction = (p.viscous_wheel * s.wheel_omega
                    + p.coulomb_wheel * np.tanh(s.wheel_omega / 0.4))
        net = s.motor_torque - friction - traction * p.wheel_radius
        net = np.where(self.wheel_blocked, 0.0, net)
        s.wheel_omega += net / p.wheel_inertia * dt
        s.wheel_omega = np.where(self.wheel_blocked, 0.0, s.wheel_omega)

        # Resolve contact forces into a body wrench (transpose of the
        # kinematic map -- the same matrix, used the other way round).
        fx = traction.sum()
        fy = float(self._sy @ traction)
        mz = float(self._sw @ traction) * p.lxy

        ax = (fx - p.chassis_drag * s.vx) / p.mass
        ay = (fy - p.chassis_drag * s.vy) / p.mass
        aw = (mz - p.chassis_drag_yaw * s.wz) / p.inertia_z

        s.vx += ax * dt
        s.vy += ay * dt
        s.wz += aw * dt

        # Integrate pose in the world frame.
        c, sn = np.cos(s.theta), np.sin(s.theta)
        s.x += (s.vx * c - s.vy * sn) * dt
        s.y += (s.vx * sn + s.vy * c) * dt
        s.theta = float(np.arctan2(np.sin(s.theta + s.wz * dt),
                                   np.cos(s.theta + s.wz * dt)))

    # -- sensors --------------------------------------------------------------
    def read_encoders(self) -> np.ndarray:
        raw = self.state.wheel_omega + self.rng.normal(0, self.p.encoder_noise, NUM_WHEELS)
        return np.round(raw / self.p.encoder_lsb) * self.p.encoder_lsb

    def read_currents(self) -> np.ndarray:
        i = self.state.motor_torque / self.p.torque_constant
        return i + self.rng.normal(0, self.p.current_noise, NUM_WHEELS)

    def read_gyro(self) -> float:
        return float(self.state.wz + self.p.gyro_bias
                     + self.rng.normal(0, self.p.gyro_noise))

    def true_pose(self) -> tuple[float, float, float]:
        return self.state.x, self.state.y, self.state.theta

    # -- fault injection ------------------------------------------------------
    def set_friction(self, wheel: int, mu: float) -> None:
        """Simulate a wet patch, spilled oil, or a floor drain grating."""
        self.mu[wheel] = mu

    def block_wheel(self, wheel: int, blocked: bool = True) -> None:
        """Simulate a jammed bearing or a wheel wedged against a pallet foot."""
        self.wheel_blocked[wheel] = blocked
