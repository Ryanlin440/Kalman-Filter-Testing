# littleX UKF plan: from the Python sandbox to the PROS robot

Instructions and context for Claude in VS Code. Written Nov 1, 2026 in a claude.ai chat.
Read this together with `littleX_handoff.md` (the Desmos calculator + wall-reset port).

**Goal:** replace the robot's current localization (two separate 1D Kalman filters, one for x and
one for y) with a full 3-state UKF over [x, y, theta], running in PROS C++ on the V5 brain.

**Key finding:** the full UKF already exists and works, in Python, in Ryan's repo
**https://github.com/Ryanlin440/ukfwithpython**. This project is mostly a careful *port* of a
verified filter, plus adapting it to the real robot. Do not redesign the math; copy it.

---

## 0. Before starting

1. Read `littleX_handoff.md` (conventions, Desmos equations, `wall_math.hpp`).
2. Clone the Python repo, install deps, run its tests:
   ```
   git clone https://github.com/Ryanlin440/ukfwithpython
   cd ukfwithpython
   pip install -r requirements.txt        # Flask 3.0.3, numpy 2.1.3
   python -m unittest discover -s tests   # expected: Ran 63 tests ... OK
   python app.py                          # web sandbox at http://localhost:5001
   ```
3. Find the robot's **current 1D Kalman filter code** in the PROS project (it was not available
   when this file was written). Note what it uses as inputs (tracking wheels? drive encoders?
   distance sensors? IMU?), its loop period, and where its x/y output is consumed (autonomous
   paths, PID targets, LCD). The UKF must provide the same outputs to the same consumers.
4. Ask the user for anything section 6 lists as "measure on the robot" before Phase 4.

---

## 1. Why a UKF instead of two 1D filters

- Two 1D filters treat x and y as independent and assume heading is exact. Heading error is
  the main thing that couples them: a 1° heading error over 50 in of driving puts ~0.9 in of
  error in the *other* axis, which neither 1D filter can represent or correct.
- A distance reading depends on x or y *and* theta at the same time (the beam is tilted).
  The UKF uses that relationship; the 1D filters have to pretend it doesn't exist.
- The measurement model (ray to the nearest wall) is nonlinear and changes which wall it hits.
  A UKF handles this with sigma points instead of derivatives, so there are no Jacobians to get wrong.
- The UKF keeps an honest uncertainty (covariance P) for the whole pose, which is useful for
  deciding when the robot is confident enough to trust a wall reset or start a precise move.

---

## 2. The Python repo, file by file

```
app.py                        Flask server (UI only; not needed on the robot)
templates/, static/           Web UI (canvas drawing, sliders, presets)
simulation/physics.py         Conventions, angle wrap, rotation, ray_box_intersection
simulation/robot.py           Ground-truth differential-drive robot
simulation/sensors.py         Odometry pods, IMU, distance sensors (+ noise)
simulation/ukf.py             THE FILTER (port this)
simulation/simulation.py      Glue: builds controls Q and R, runs predict/update each step
tests/test_ukf_stability.py   63 tests (ray casting, numerics, wiring, odometry, convergence...)
```

### 2.1 Conventions in the Python sandbox (DIFFERENT from Desmos/littleX!)

| | Python sandbox | Desmos / littleX `wall_math.hpp` / PROS IMU |
|---|---|---|
| Heading | `theta`, **radians, counterclockwise from world +X** | degrees, **clockwise from +Y** |
| Robot frame | **+X = forward, +Y = left** | +offX = right, +offY = forward |
| Field | half-size **72 in** (144 in square) | b = 70 in (measure the real field) |
| Sensor range | **75 in** | 78.74 in (V5 spec, 2000 mm) |
| Robot | 14 × 14 in, sensors flush on face centers (mount_offset = 7 in) | Desmos: front (0,5), back (0,−5), right (5,2), left (−5,2) |
| Time step | 20 ms fixed | PROS loop typically 10 ms |

Conversions (put these in ONE place, e.g. `localization/frames.hpp`, and test them):
```
theta_rad_ccw_from_x  = (90 − heading_deg_cw_from_y) · π/180      (then wrap to [−π, π])
heading_deg_cw_from_y = 90 − theta_rad · 180/π                     (then wrap to [0, 360))
robot frame: (forward, left) = (offY, −offX)        and        (offX, offY) = (−left, forward)
```

**Decision for the port:** keep the Python convention *inside* the UKF (it is the verified
reference and lets the C++ be checked against Python number for number), and convert only at
the PROS boundary: IMU heading in, pose out to the rest of the robot code. If the rest of the
robot code uses the Desmos convention, convert in the getter (`getPoseDesmos()`).

### 2.2 State, control, and measurements

- **State** `x = [x, y, theta]` (inches, inches, radians). Covariance `P` is 3×3.
- **Initial P:** `diag(4.0, 4.0, (15°)²)` i.e. `initial_covariance = 4.0 in²`,
  `initial_heading_deg = 15` (std). Position and heading variances are separate on purpose
  (different units).
- **Control (prediction input)** = body-frame velocity from odometry, `u = [vx, vy, ω]`:
  ```
  ω  = odom_dtheta / dt
  vx = odom_forward / dt
  vy = odom_lateral / dt − lateral_offset · ω      # removes the lateral pod's swing when turning
  ```
  The lateral pod sits `lateral_offset = 5 in` ahead of the center of rotation, so turning sweeps
  it sideways by `lateral_offset · dθ`; that term must be subtracted. (If the real forward pod is
  also offset sideways by `s`, it reads `forward − s·dθ` (sign depends on side); correct it the same way.)
- **Measurements** (only channels that are enabled AND produced a reading this step):
  - `imu`: absolute heading, angular channel (residuals wrapped to [−π, π]).
  - `front`, `back`, `left`, `right`: range to the wall.
  - Order comes from `UKF.measurement_layout()`; the vector changes length each step as sensors
    go in/out of range. A reading past range is **dropped**, never fed in as −1.

### 2.3 The filter math (simulation/ukf.py), exactly as implemented

**Sigma points** (scaled unscented transform, n = 3, defaults α = 1.0, β = 2.0, κ = 0):
```
λ = α²(n + κ) − n,  floored so that n + λ ≥ 1e−3
S = chol( make_PD( (n + λ) · P ) )         # lower-triangular Cholesky
χ0 = x,  χi = x + S[:, i],  χ(i+n) = x − S[:, i]          (i = 1..n)   → 7 points
Wm0 = λ/(n+λ),   Wc0 = λ/(n+λ) + (1 − α² + β),   Wmi = Wci = 1 / (2(n+λ))
```
With the defaults λ = 0, so Wm0 = 0 and the other six weights are 1/6. Wc0 = 2.

`make_PD(M)`: symmetrize `(M + Mᵀ)/2`; if the smallest eigenvalue ≤ 1e−9, add `(1e−9 − min_eig)·I`
and symmetrize again. (For 3×3 in C++: try Cholesky; if it fails, add a small diagonal and retry.
That is acceptable as long as the parity test in Phase 3 still passes.)

**Predict** (motion model uses the heading at the START of the step):
```
for each sigma point (x, y, θ):
    x' = x + (cosθ·vx − sinθ·vy)·dt
    y' = y + (sinθ·vx + cosθ·vy)·dt
    θ' = wrap(θ + ω·dt)
mean: x̄, ȳ = weighted sums;   θ̄ = atan2(Σ Wm·sinθ', Σ Wm·cosθ')      # circular mean
P̄ = Σ Wc·d·dᵀ (d = χ' − mean, with d[2] wrapped) + Q
P̄ = make_PD(P̄);  state = clamp x, y to ±position_bound
```

**Process noise Q** (simulation.py `build_process_noise`), body-frame, rotated into the world:
```
with odometry:  forward_var = σ_odom²
                lateral_var = σ_odom² + (lateral_offset · σ_odom_ang)²
                angular_var = σ_odom_ang²
no odometry:    forward_var = lateral_var = (v_max·dt)²,  angular_var = (ω_max·dt)²
Q[0:2,0:2] = R(θ̂) · diag(forward_var, lateral_var) · R(θ̂)ᵀ ;  Q[2,2] = angular_var
(each variance floored at 1e−9)
```
σ_odom is **per step** (inches per 20 ms step), not per second. If the loop period changes, rescale.

**Measurement model h(x)** for a range channel (physics.py `ray_box_intersection`):
```
d_world = R(θ) · d_body                         # d_body: front (1,0), back (−1,0), left (0,1), right (0,−1)
origin  = (x, y) + R(θ) · (d_body · mount_offset)
origin  = clip(origin, −B, B)                   # keeps h continuous if an estimate leaves the field
t_exit  = min over axes with |d| > 1e−12 of max((−B − o)/d, (B − o)/d)
return clamp(t_exit, 0, 4B)
```
This is the same ray-to-box geometry as the Desmos `a_*`/`d_*` equations and `loc::raycast()` in
`wall_math.hpp`. The prediction is **deliberately not capped at the 75 in range**: a channel only
appears when the real sensor read something, and capping would flatten the gradient.
IMU channel: `h = wrap(θ)`.

**Update:**
```
Z_i = h(χ_i)  for each sigma point (recompute sigma points from the predicted x, P)
z̄   = weighted mean (circular mean for the imu channel)
Pzz = Σ Wc·dz·dzᵀ + R,  then make_PD          (dz wrapped on angular channels)
Pxz = Σ Wc·dx·dzᵀ                             (dx[2] wrapped)
ν   = z − z̄   (wrapped on angular channels)
K   = solve(Pzzᵀ, Pxzᵀ)ᵀ                      # never invert Pzz explicitly
x   = x + K·ν ;  θ wrapped ;  clamp x, y
P   = make_PD(P − K·Pzz·Kᵀ)
```

**Measurement noise R** = diagonal: `σ_imu²` for imu (floor 1e−6), `σ_dist²` for ranges
(floor 1e−4). Zero noise makes Pzz singular, hence the floors.

**Position bound:** x, y are clamped to `±(72 − 7) = ±65` after every step. This is map
knowledge (the robot's center can't be closer than half its width to a wall), not ground truth.

### 2.4 Default noise values in the sandbox
```
odometry          0.4   in / step   (20 ms)
odometry_angular  0.002 rad / step
imu               0.5°  (0.00873 rad)
distance          0.8   in
```
Presets: perfect_sensors, noisy_odometry (3.0 in/step), noisy_imu (15°), no_odometry,
odometry_only, all_sensors (default), high_noise (5 in/step, 5°, 3 in).

---

## 3. Measured performance of the Python UKF (Nov 1, 2026)

Headless runs: 3000–6000 steps of a drive / arc / drive / spin-in-place pattern, 3 seeds,
first 200 steps skipped. Position error in inches, heading error in degrees.

**Presets in the repo:**

| Preset | mean pos | p95 pos | max pos | mean hdg | max hdg |
|---|---|---|---|---|---|
| perfect_sensors | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| all_sensors (default) | 0.60 | 1.66 | 4.59 | 0.18 | 0.78 |
| noisy_odometry | 1.90 | 9.30 | 33.6 | 0.36 | 1.76 |
| noisy_imu | 0.61 | 1.61 | 4.33 | 0.40 | 1.80 |
| no_odometry | 2.33 | 14.9 | 48.9 | 0.38 | 5.01 |
| high_noise | 4.16 | 15.3 | 49.4 | 2.04 | 14.0 |
| odometry_only | 27.8 | 55.5 | 75.9 | 11.1 | 23.6 (drifts, as expected) |
| all_sensors, bad start (15, −15, 10° off) | 0.60 | 1.66 | 4.59 | 0.18 | 0.78 |
| all_sensors, IMU off | 0.61 | 1.62 | 4.33 | 0.40 | 2.00 |

**Realistic V5-like noise** (closer to a real robot with tracking wheels):

| Case | mean pos | p95 | max | mean hdg | max hdg | % of steps error < 2σ |
|---|---|---|---|---|---|---|
| odom 0.05 in/step, dist 1.5 in, IMU 0.5° | 0.30 | 0.67 | 1.15 | 0.09 | 0.41 | 95.1 |
| same, dist 3.0 in | 0.45 | 1.03 | 1.68 | 0.09 | 0.42 | 92.3 |
| odom 0.2 in/step (wheel slip), dist 1.5 | 0.60 | 1.33 | 3.38 | 0.18 | 0.83 | 97.5 |

**Interpretation:**
- At realistic noise the filter is accurate and its covariance is honest. The math is fine.
- The big spikes in noisy_odometry / no_odometry / high_noise are **corner ambiguity**: the
  estimate drifts far enough that a sensor's predicted beam hits a *different wall* than the real
  beam (e.g. predicted range 73 in to the left wall vs. measured 48 in to the top wall). On that
  wall the reading carries no information about the axis that is wrong, so the filter can't pull
  back. Those presets inject 150–250 in/s of odometry noise, which is not physical.
- Two fixes were tried and **both made things worse**; don't repeat them without a new idea:
  1. A 3σ per-channel innovation gate alone: no help on the spikes, and in no_odometry the filter
     got lost and then rejected every reading forever (mean error 29 in, max 130 in).
  2. Dropping a range channel when the sigma points disagree on which wall it hits: it discards
     too much information (no_odometry max 75 in, high_noise max 84 in).
- What actually matters on the real robot is different: blocked beams (robots, game elements,
  field elements against walls). The sim has none. See Phase 1.

**Doc bug in the repo README:** it claims "at least one ray per axis is always in range,
whatever the pose". False at diagonal headings: at (0, 0) facing 45°, every beam is
72/cos45° − 7 ≈ 94.8 in, past the 75 in range. (Same mid-field dead zone the Desmos graph shows.)
The filter handles it fine by coasting on odometry; just fix the sentence.

Benchmark script used (save as `tools/bench_realistic.py` in the Python repo and run
`python tools/bench_realistic.py .`):
```python
import sys, math, numpy as np
sys.path.insert(0, sys.argv[1])
from simulation.simulation import Simulation
from simulation.physics import normalize_angle

def run(cfg, seed, steps=6000):
    np.random.seed(seed); s = Simulation()
    s.noise_config.update(cfg); s.sync_sensor_noise()
    pe, he, inside = [], [], []
    for k in range(steps):
        ph = (k // 150) % 4                       # drive, arc, drive, spin
        s.key_states.update({'W': ph in (0, 1, 2), 'S': False,
                             'ArrowLeft': ph == 1, 'ArrowRight': ph == 3})
        s.update_simulation(s.dt, force=True)
        r, e = s.robot, s.ukf.state
        err = math.hypot(r.x - e[0], r.y - e[1])
        pe.append(err); he.append(abs(math.degrees(normalize_angle(r.theta - e[2]))))
        inside.append(err < 2 * math.sqrt(s.ukf.P[0, 0] + s.ukf.P[1, 1]))
    pe, he = np.array(pe[200:]), np.array(he[200:])
    return pe.mean(), np.percentile(pe, 95), pe.max(), he.mean(), he.max(), 100 * np.mean(inside[200:])

deg = math.pi / 180
cases = {
    'V5-like':        dict(odometry=0.05, odometry_angular=0.0005, imu=0.5 * deg, distance=1.5),
    'noisy walls':    dict(odometry=0.05, odometry_angular=0.0005, imu=0.5 * deg, distance=3.0),
    'wheel slip':     dict(odometry=0.2,  odometry_angular=0.002,  imu=0.5 * deg, distance=1.5),
}
for name, cfg in cases.items():
    r = np.mean([run(cfg, s) for s in range(3)], axis=0)
    print(f"{name:14s} mean {r[0]:.2f}  p95 {r[1]:.2f}  max {r[2]:.2f} | "
          f"hdg {r[3]:.2f} / {r[4]:.2f} | {r[5]:.1f}% < 2σ")
```

---

## 4. Build plan (do the phases in order; each has a pass/fail check)

### Phase 1: make the Python sim match the real robot

Change the Python repo (on a branch) so the simulator matches the real robot before porting.
The 63 existing tests must keep passing (update a test only if its assumption intentionally changed,
and say so in the commit).

1. **Arbitrary sensor mounts.** Replace the single `mount_offset` + axis-aligned directions with
   per-sensor `(forward, left, angle)` in the body frame, so the Desmos-style offsets
   (front (0,5), right (5,2), ...) and later the real robot's measured offsets can be used.
   `UKF.distance_sensor_measurement` and `DistanceSensor.origin` must use the same mount table.
2. **Field and sensor limits as config:** half-size (real field, see section 6), max range 78.74 in,
   robot size. Keep 72 / 75 as a selectable "sandbox" profile so old results are reproducible.
3. **Realistic distance noise:** V5 distance sensor spec is about ±15 mm (0.6 in) below 200 mm and
   about ±5% beyond. Use `σ(d) = max(0.6, 0.05·d)` in the simulator AND in the filter's R
   (R becomes per-channel, computed from the measured range).
4. **Add a "realistic V5" preset** with the section 3 values; make it the default for future work.
5. **Simulate blocked beams:** with probability p per reading (start 2–5%), replace a reading with
   a short random range (an object between robot and wall). Optionally add rectangular obstacles
   near walls (goals) that the ray cast hits.
6. **Outlier rejection that can't get stuck:** per-channel normalized innovation
   `|ν_i| / sqrt(Pzz_ii) > 3` → skip that channel this step, BUT also count consecutive rejected
   steps; if most channels are rejected for N steps (start N = 25 at 20 ms), assume the filter is
   lost: inflate P (e.g. position var × 25) instead of rejecting, or re-seed from the wall-reset
   `globalFix()` logic. Section 3 shows a plain gate gets stuck forever without this.
7. **Incidence check** (optional, from `wall_math.hpp`): skip a range channel when the beam hits the
   wall at more than ~35° from perpendicular at the current estimate, because the cone reads short.
   Compare with and without in the benchmark; keep it only if it helps.

**Pass:** all tests pass; on the "realistic V5" preset with 3% blocked beams, mean position error
< 0.6 in, max < 3 in, mean heading error < 0.3°, and the filter recovers (error < 2 in within 3 s)
from a 20 in / 20° kidnap (teleport the true robot).

### Phase 2: a log format for parity testing

Add `tools/export_log.py` that runs a fixed seed scenario and writes, per step, everything the filter
consumed and produced, e.g. JSON lines:
```
{"dt":0.02, "u":[vx,vy,w], "Q":[[...]], "layout":["imu","front","left"],
 "z":[...], "R":[[...]], "x_pred":[...], "P_pred":[[...]], "x":[...], "P":[[...]]}
```
Include at least 2000 steps covering: straight driving, arcs, spin in place, sensors dropping
in and out of range, the mid-field dead zone, the IMU on and off.

**Pass:** the file regenerates identically for the same seed.

### Phase 3: C++ UKF library (host-testable, no PROS)

Files (suggested): `include/localization/ukf.hpp`, `src/localization/ukf.cpp`,
`include/localization/frames.hpp`, `test/test_ukf_parity.cpp`.

- Pure C++17, no PROS headers, so it compiles and runs on a laptop with g++.
- **Matrices:** n = 3, at most 5 measurement channels. Either
  - Eigen with fixed-size types (`Eigen::Matrix<double,3,3>`, `Matrix<double,Eigen::Dynamic,1,0,5,1>`).
    It's header-only and used on V5 by other teams, but adds compile time and binary size; or
  - a small hand-written fixed-size library: 3×3 Cholesky, and an m×m (m ≤ 5) solve via Cholesky
    of Pzz (it's symmetric positive definite). Fewer than ~150 lines, no dependency.
  Pick one and say why in a comment. No heap allocation inside predict/update.
- Use `double` (the V5's Cortex-A9 has a double-precision FPU).
- Mirror the Python API: `predict(u, dt, Q)`, `update(z, layout, R)`, `setState`, `state()`, `P()`.
  Keep the Python names for intermediate values so a reviewer can diff them side by side.
- Measurement model: reuse the same ray-to-box function for h(x). Either port
  `ray_box_intersection` directly (exit parameter, origin clipped into the box, clamp to [0, 4B]) or
  adapt `loc::raycast()` from `wall_math.hpp`, but the parity test decides: results must match Python.
- Angle handling: `normalize_angle` = `std::remainder(a, 2π)`; circular means; wrap residuals on the
  imu channel. Mistakes here show up as filter blow-ups near ±180°, so test headings around ±π.
- Parity test: read the Phase 2 log, feed the same u, Q, z, R into the C++ filter step by step, and
  compare.

**Pass:** over the whole log, max |state difference| < 1e−6 and max |P difference| < 1e−6
(relative to Python), including steps where the layout changes size. Also port at least these
Python tests to C++: sigma weights sum to 1, PD covariance over a long run, circular mean across ±π,
odometry-only drifts, perfect sensors track exactly.

Measure the time of one predict + update with all 5 channels on the laptop and estimate for the
V5 (expect well under 1 ms; the loop is 10 ms).

### Phase 4: PROS integration

- A localization task (`pros::Task`) at a fixed period (10 ms is typical; if the Python sim stays at
  20 ms, rescale per-step noise: σ per step ∝ sqrt(dt) for random-walk noise).
- **Inputs:**
  - Odometry: tracking wheels (rotation sensors) or drive encoders. Convert ticks → inches with the
    measured wheel diameter. Compute `forward`, `lateral`, `dθ` per step in the robot frame of the
    START of the step; apply the pod-offset correction from section 2.2 with the real offsets.
  - Heading change `dθ`: from the IMU (`get_rotation()` differences are unwrapped and smoother than
    `get_heading()`), or from two parallel tracking wheels if the robot has them.
  - IMU absolute heading channel: convert PROS heading (deg, CW from +Y) to the filter convention with
    `frames.hpp`. Add the start-heading offset. Calibrate (`reset(true)`) before the match and don't
    move the robot during calibration.
  - Distance sensors: `pros::Distance::get()` in mm (9999 = nothing, PROS_ERR = failure); drop the
    channel if invalid, above 78.74 in, or `get_confidence()` is low past 200 mm. Convert mm → in.
- **Outputs:** a thread-safe getter (mutex) returning the pose in the filter frame and in the Desmos
  frame; P diagonal for diagnostics. Replace every use of the old 1D filters' x/y with this.
- **Initialization:** at match start, set the known start pose with small P; if unknown, seed from
  `WallReset::globalFix()` (littleX `wall_reset.hpp`) and use a large P.
- **Logging:** write the same JSON-lines format as Phase 2 (inputs + outputs) to the SD card
  (`/usd/ukf_log.txt`) or over the serial terminal. This lets real runs be replayed in Python.
- **LCD:** show x, y, heading, sqrt(Pxx + Pyy), and how many range channels were used.
- Keep the old 1D filter code behind a compile-time switch until the UKF is proven on the robot.

**Pass:** on the robot, push it by hand around a known path (e.g. a tape square on the tiles); the
pose returns to within ~1 in / 1° of the start. Drive autonomous routines that previously used the
1D filters; final position error is equal or better. Replay one robot log in Python and the Python
filter produces the same trajectory (proves the C++ and Python still agree on real data).

### Phase 5: tuning on real data

- Fit Q from odometry-only runs (how fast does error actually grow per meter and per turn?).
- Fit R for the distance sensors from stationary readings at several distances and angles.
- Check consistency: normalized innovations (ν/√Pzz) should be roughly unit variance. Consistently
  > 1 means R or Q is too small (filter overconfident); consistently < 1 means too large.
- Tune the outlier gate and lost-filter recovery with logs from real matches (other robots block beams).
- Later options: add an IMU bias/drift state (state becomes 4D), add velocity states, model field
  elements that sit against the walls.

---

## 5. Pitfalls to watch for

1. **Convention mix-ups** (biggest risk). Python: radians, CCW from +X, body +X forward, +Y left.
   PROS / Desmos: degrees, CW from +Y, +offX right, +offY forward. Convert only in `frames.hpp`,
   unit test round trips, and test at 0°, 90°, 180°, 270°, −179°, 359°.
2. **Per-step vs per-second noise.** σ_odom in the sim is per 20 ms step.
3. **Wrapping:** circular means for θ and the imu channel; wrap every angle difference.
4. **Heading at the start of the step** in both the motion model and the odometry computation; using the
   end or midpoint heading adds a systematic error while turning.
5. **Pod swing:** subtract `offset · dθ` for any tracking wheel not on the center of rotation.
6. **Variable measurement size:** never feed an out-of-range reading (or 9999 mm, or −1) into the update.
7. **Don't cap h(x) at sensor range,** and keep the origin clipping inside `ray_box_intersection`.
8. **Square-field symmetry:** four orthogonal beams can't tell 90° rotations apart; the filter needs the
   IMU (or a good initial heading) to avoid settling into a rotated pose.
9. **Overconfidence after a bad reading:** a wrong-wall or blocked reading can shrink P while moving the
   estimate the wrong way. That is why Phase 1 adds gating with recovery.
10. **Real IMU drifts** (the sim treats it as absolute with white noise). If heading error grows slowly
    over a match, add a bias state or rely more on wall readings for heading.

---

## 6. Measure on the robot (ask the user)

- Real field inner half-size (wall to wall ÷ 2), in inches.
- Each distance sensor: port, position of its FACE relative to the center of rotation
  (forward, left or right), and which way it points.
- Tracking wheels: count, ports, wheel diameter, offsets from the center of rotation, gear ratio.
  (The sim assumes one forward pod at the center and one lateral pod 5 in forward.)
- IMU port and how the robot is oriented at the start of each autonomous routine.
- Drive geometry: track width, max speed (for the no-odometry Q fallback).
- Current loop period and where the current 1D filter's output is used.

---

## 7. Summary of the order

1. Read both handoff files, clone the Python repo, run tests (63 OK).
2. Phase 1: make the sim realistic (mounts, field, noise, blocked beams, gating with recovery).
3. Phase 2: export a parity log.
4. Phase 3: C++ UKF, host-tested, matching Python to 1e−6.
5. Phase 4: PROS task, sensors, logging, replace the 1D filters.
6. Phase 5: tune Q, R, gate on real logs.
