# omni-drive-core

**A real-time control core for a four-wheel holonomic AMR, with the validation harness that proves it works.**

C++17 control stack for a mecanum-wheeled autonomous mobile robot — the kind built by mounting four independently driven wheels under an existing cart or rack. The control law is compiled once and driven from three places without modification: a ROS 2 node, a Python physics simulator, and (by design) a microcontroller on the wheel itself.

[![ci](https://github.com/PRAhit/omni-drive-core/actions/workflows/ci.yml/badge.svg)](https://github.com/PRAhit/omni-drive-core/actions/workflows/ci.yml)

| | |
|---|---|
| C++ unit tests | **35 passing**, `-Werror -Wconversion -Wshadow -Wold-style-cast` |
| Python tests | **57 passing** |
| Closed-loop scenarios | **11 passing** against numeric gates, over 12 random seeds |
| Control step cost | **35 µs** p99.9 — 0.7 % of a 5 ms budget at 200 Hz |
| Sanitizers | ASan + UBSan clean |

---

## Why this repository exists

Most robotics portfolio projects demonstrate that a controller *can* drive a robot. This one is built around a harder and more useful question: **how do you know the controller is right, and how do you keep knowing it?**

So the architecture is arranged around testability rather than around ROS:

```
                      ┌──────────────────────────────────┐
   cmd_vel ──────────▶│                                  │
   wheel_states ─────▶│         odc::DriveCore           │──▶ torque_cmd[4]
   imu/data ─────────▶│                                  │──▶ odom + TF
   scan ─────────────▶│  step(CycleInput, dt)            │──▶ diagnostics
   estop ────────────▶│                                  │
                      └──────────────────────────────────┘
                        no allocation · no I/O · no clock
                        no exceptions · no ROS · no globals
                                      │
              ┌───────────────────────┼───────────────────────┐
              ▼                       ▼                       ▼
      ROS 2 node               Python simulator         wheel-node MCU
      (topics, TF)             (pybind11, SIL)          (cross-compile)
```

`DriveCore::step()` is the entire real-time contract. Every dependency is passed in; every result is returned. It reads no clock, allocates nothing, throws nothing, and touches no global state. That is what makes the same object usable from a 200 Hz interrupt, from a ROS callback, and from a pytest case — and it is why the closed-loop scenarios below are testing *production control code*, not a Python re-implementation that will silently drift out of sync with it.

---

## Architecture

| Module | Responsibility |
|---|---|
| `MecanumKinematics` | Forward/inverse kinematics; exposes the kinematic redundancy as a diagnostic |
| `WheelController` ×4 | Velocity loop: PI + friction feedforward, slew limiting, back-calculation anti-windup |
| `PoseEkf` | 4-state EKF over `[x, y, θ, gyro_bias]`, Mahalanobis-gated absolute fixes |
| `SlipDetector` | Three-channel slip and traction-loss detection |
| `TrajectoryTracker` | Holonomic pose tracking, world-frame PI + feedforward |
| `SafetyMonitor` | Protective/warning fields, watchdogs, latching fault state machine |
| `DriveCore` | Orchestrates the above into one deterministic cycle |

### Kinematics

Wheel layout, REP-103 frame (x forward, y left, z up):

```
        x
        ▲
  FL[0] │ FR[1]
  ──────┼──────▶ y
  RL[2] │ RR[3]
```

Inverse (body twist → wheel speeds), with `L = lx + ly`:

```
ω_FL = (vx − vy − L·ωz) / r        ω_FR = (vx + vy + L·ωz) / r
ω_RL = (vx + vy − L·ωz) / r        ω_RR = (vx − vy + L·ωz) / r
```

The three columns of this map are mutually orthogonal, so the least-squares inverse is the exact Moore–Penrose pseudo-inverse and forward kinematics is a clean closed form. Four wheels, three degrees of freedom, one scalar of redundancy:

```
ω_FL + ω_FR − ω_RL − ω_RR = 0
```

Any violation is slip, an encoder fault, or wrong geometry — measured with no extra hardware.

### Slip detection: three channels, and why each is necessary

```
┌───────────────────────────────────────────────────────────────────────┐
│ 1. KINEMATIC     |ω_FL + ω_FR − ω_RL − ω_RR| / 2      →  70 ms        │
│    catches: seized bearing, wedged wheel, dead encoder                │
│    blind to: traction loss  (see below — this matters)                │
├───────────────────────────────────────────────────────────────────────┤
│ 2. INNOVATION    EKF normalised innovation squared    →  140 ms       │
│    catches: the platform actually sliding                             │
│    blind to: anything the scan matcher cannot see                     │
├───────────────────────────────────────────────────────────────────────┤
│ 3. GYRO          |ω_gyro − ω_odom|                    →  fast         │
│    catches: asymmetric slip that rotates the platform                 │
└───────────────────────────────────────────────────────────────────────┘
```

**What none of them catch:** all four wheels slipping equally in pure translation. That is genuinely unobservable from proprioception — there is no internal reference left to disagree with. The gated EKF is the backstop, which is why it never blindly trusts odometry.

---

## Results

Every scenario below carries numeric acceptance criteria and runs in CI. `tools/run_scenarios.py` exits non-zero on any gate failure, so a control regression breaks the build exactly like a failing unit test. Worst value across **12 seeds**:

| Scenario | Gate | Limit | Worst of 12 |
|---|---|---|---|
| `straight_line` | RMS tracking error | ≤ 0.05 m | **0.017** |
| | Peak tracking error | ≤ 0.12 m | **0.025** |
| | Step time p99.9 | ≤ 250 µs | **34.7** |
| `lateral_strafe` | RMS tracking error | ≤ 0.06 m | **0.017** |
| | Heading drift | ≤ 0.15 rad | **0.014** |
| `figure_eight` | RMS tracking error | ≤ 0.10 m | **0.017** |
| | RMS estimator error | ≤ 0.06 m | **0.013** |
| `spin_in_place` | Position drift | ≤ 0.20 m | **0.032** |
| `traction_loss_wet_strip` | Detection latency from real onset | ≤ 0.45 s | **0.14** |
| | Heading deviation | ≤ 0.25 rad | **0.108** |
| | Protective-stop entries | ≤ 3 | **3** |
| `traction_loss_single_wheel_absorbed` | RMS estimator error | ≤ 0.05 m | **0.031** |
| `wheel_jam` | Detection latency from real onset | ≤ 0.45 s | **0.07** |
| `obstacle_intrusion` | Stop latency | ≤ 0.05 s | **0** |
| | Speed after stop | ≈ 0 | **0** |
| `bus_dropout` | Bus fault latency | ≤ 0.05 s | **0** |
| `localisation_blackout` | Dead-reckoning drift | ≤ 0.50 m | **0.119** |
| `estop` | Latency | ≤ 0.02 s | **0** |
| | No self-clear after release | ≈ 0 | **0** |

Detection latency is measured from **ground-truth slip onset in the plant**, not from when the command was issued — timing from the command conflates controller latency with however long the physics took to break traction.

### Figure-of-eight at constant heading

Curved path with the body never turning: all three axes of the mecanum allocation are active continuously, so any sign error in the kinematics shows up immediately as drift off the loop.

![figure eight](docs/img/figure_eight.png)

### Wet strip across the aisle

Both left-side wheels lose grip during a hard start. Traction demand (19 N/wheel) exceeds what µ=0.05 can transmit (10 N), so the left side breaks away and the platform yaws.

![wet strip](docs/img/traction_loss_wet_strip.png)

### Obstacle intrusion

Linear speed ramp through the warning field, protective stop at the inner boundary, and extra clearance required before resuming.

![obstacle](docs/img/obstacle_intrusion.png)

---

## Five bugs this harness caught

The scenarios and plots are not decoration. Each of these was a real defect in code that looked fine and compiled clean.

**1 — The slip residual is rank-1 and cannot name a wheel.**
The first detector ranked per-wheel residual magnitudes. But the residual is the projection onto the single left-null direction, so it is *always* parallel to `[1, 1, −1, −1]` — all four entries share one magnitude. Ranking them is reading a four-way tie. A test now locks the property down so nobody "improves" it back.

**2 — Anti-windup gain was 2.0 when the rule of thumb is `ki/kp ≈ 13.3`.**
During a stall the integrator settled at 252 instead of ~5, because back-calculation reaches equilibrium at `ki·e / gain`. Caught by a test that compares against anti-windup disabled rather than asserting a magic number.

**3 — Kinematic consistency is structurally blind to traction loss.**
Under per-wheel velocity control the setpoints *come from* inverse kinematics, so four well-tracking wheels satisfy the constraint **by construction** however much the platform is sliding. This is the single most important property of the method and it is easy to miss: the check detects *tracking* failures, not *traction* failures. It drove the redesign into three channels.

**4 — Comparing wheel current against the fleet median false-positives constantly.**
The replacement for (3) assumed all four wheels carry comparable load. They do not: a figure-of-eight produces a normal 3.5 A spread, because a mecanum base loads its wheels very unevenly in curved motion. Replaced with the EKF innovation, which is the correct signal and was already being computed. A regression test now encodes the counter-example.

**5 — Two failures visible only in the plots.**
A single localisation outlier at a 1 % rate spiked NIS through an EWMA and protective-stopped a healthy robot on 3 seeds out of 8. Fixed by counting consecutive bad *fixes* rather than filtering a continuous statistic — three in a row is a one-in-a-million event. And the wet-strip trace showed the machine chattering in and out of protective stop **12 times**: slip trips a stop, the stop halts the wheels, still wheels report no slip, motion resumes, the wheels slip again. A minimum stop dwell turned that limit cycle into 3 clean stop-and-restart cycles.

One more result worth stating because it is *not* a bug: single-wheel traction loss on a four-wheel base is **absorbed**. Three healthy wheels take up the slack and estimator error stays inside 3 cm, so the innovation channel is correct not to fire. That is now its own scenario documenting graceful degradation, rather than pretending every traction event is an emergency.

---

## Build and run

```bash
# C++ core and unit tests
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

# Python bindings and the SIL harness
pip install numpy matplotlib pytest pybind11
cmake -S . -B build -DODC_BUILD_PYTHON=ON \
  -Dpybind11_DIR=$(python -c "import pybind11; print(pybind11.get_cmake_dir())")
cmake --build build -j && cp build/omni_drive_core*.so python/

python -m pytest tests/ -q
python tools/run_scenarios.py --seeds 12     # acceptance gates
python tools/plot_scenarios.py               # regenerate docs/img
```

```bash
# ROS 2 (Jazzy)
cd ros2_ws && colcon build && source install/setup.bash
ros2 launch omni_drive_ros drive.launch.py
```

### ROS 2 interface

| Direction | Topic | Type |
|---|---|---|
| in | `cmd_vel` | `geometry_msgs/Twist` |
| in | `wheel_states` | `sensor_msgs/JointState` (velocity, effort) |
| in | `imu/data` | `sensor_msgs/Imu` |
| in | `scan` | `sensor_msgs/LaserScan` |
| in | `estop` | `std_msgs/Bool` |
| out | `odom` + TF | `nav_msgs/Odometry` |
| out | `wheel_commands` | `sensor_msgs/JointState` |
| out | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| srv | `reset_faults` | `std_srvs/Trigger` |

Velocity commands are integrated into a moving **pose** reference rather than passed through, so the tracker closes a position loop. That is what stops the platform drifting off an aisle centre line over a 40 m run.

---

## Layout

```
cpp/include/odc/   control core headers
cpp/src/           implementations
cpp/tests/         35 GoogleTest cases
bindings/          pybind11 module
python/odc_sim/    plant model, SIL harness, 11 scenarios
tools/             scenario runner (CI gate), plot generator
ros2_ws/           ROS 2 wrapper package
tests/             pytest suite
```

## Design notes

- **Fixed-step, explicit state.** No `std::chrono` inside the core. `dt` is an argument, so a replay of recorded inputs reproduces recorded outputs bit-for-bit — there is a test asserting exactly that.
- **Latching faults.** An e-stop or a dead wheel node does not self-clear because the next frame looked fine. Clearing requires an explicit `reset()` from a supervisor.
- **Safety scales, then clips.** Speed limiting multiplies the twist rather than clipping per-axis, preserving the *direction* of motion while the magnitude ramps down.
- **One circular dependency, resolved explicitly.** The EKF inflates process noise during slip and the slip detector consumes the EKF innovation. The filter uses the previous cycle's verdict: a 5 ms lag against a fault that persists for hundreds of milliseconds.

MIT licensed.

---

## Provenance

Written from published fundamentals, not derived from any company's codebase.
Mecanum kinematics dates to Ilon's 1972 patent and appears in every mobile
robotics textbook; the estimator is a standard EKF; the velocity loops use
back-calculation anti-windup from the classical control literature; the
protective and warning field behaviour follows the shape described in
ISO 3691-4. The parameter values are plausible figures for a loaded cart, not
measurements from any real product.

No third-party source is vendored here. GoogleTest is fetched at build time
and pybind11 comes from pip; NumPy and Matplotlib are used only by the
simulation and plotting tools. All are permissively licensed.

No affiliation with, or endorsement by, any robotics manufacturer. Product and
company names, if mentioned anywhere, belong to their respective owners.
