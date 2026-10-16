# Design notes

This file records the decisions behind the project and the trade-offs made, so
each one can be explained and defended. It is updated as the project evolves;
sections marked *(planned)* describe intent that has not been built yet.

## 1. Goal and scope

Demonstrate, in simulation, the two capabilities a line-docking inspection
drone depends on:

1. **Vision-based precision landing**: find a fiducial marker with a
   downward camera, estimate its pose, and use that pose to centre over and
   descend onto a pad with a measured landing error.
2. **A simple inspection mission**: fly a plan of waypoints along a line of
   simplified transmission towers, capture an image at each, return and
   precision-land.

Non-goals: real-hardware deployment, ROS 2 (optional stretch), detecting real
conductors, GPS-denied state estimation.

## 2. Stack

| Layer | Choice | Why |
|---|---|---|
| Flight stack | PX4 SITL, official image `px4io/px4-sitl-gazebo:v1.18.0-rc1`, pinned by digest | Closest to a real autopilot; prebuilt so a reviewer starts in minutes |
| Simulator | Gazebo Harmonic (gz-sim 8), shipped in that image | PX4's supported simulator; has a camera sensor model |
| Vehicle | `x500_vision_down` (this repo): PX4's x500 + a 640×480 @ 15 Hz downward camera | The stock `x500_mono_cam_down` camera is too heavy for a laptop (see §3.1); flown with the stock x500 airframe parameters |
| Vehicle API | MAVSDK-Python (`mavsdk-grpc` package), offboard velocity setpoints | Small, explicit API over MAVLink; same protocol as a real companion computer |
| Camera transport | `gz.transport13` Python bindings (installed in the image) | Reads the camera topic directly, so there's no ROS bridge to set up and debug |
| Vision | OpenCV ArUco + `solvePnP` | Well understood, deterministic, and easy to explain |

### Why v1.18.0-rc1 and not v1.17.0 stable

v1.17.0 is the newest stable release, but PX4 published no prebuilt Gazebo
container or `.deb` for it. Building v1.17 from source would add about 40
minutes and an 8–10 GB image for every reviewer. rc1 is a release candidate;
the parts this project uses (offboard mode, MAVLink, the x500 model) are
mature and unlikely to change before final. The image is pinned by digest, so
builds are reproducible even if the tag moves.

## 3. Runtime architecture

```
 Windows desktop (WSLg)            WSL2 Ubuntu
 ┌──────────────┐        ┌─────────────────────────────────────────────┐
 │ Gazebo GUI   │◄──X11──┤ container "sim"                             │
 │ OpenCV view  │        │   PX4 SITL ◄──► Gazebo server (physics,     │
 └──────────────┘        │                  camera rendering)          │
 ┌──────────────┐ UDP    │     ▲ MAVLink udp 14540     │ gz-transport  │
 │QGroundControl│◄─14550─┤     │                       ▼               │
 └──────────────┘        │ container "app" (shares sim's network)      │
                         │   vehicle.py (MAVSDK)   camera.py (gz sub)  │
                         │        ▲                     │              │
                         │   controller.py ◄── detector.py (ArUco)     │
                         └─────────────────────────────────────────────┘
```

Both services use the same image. `app` runs with `network_mode: service:sim`,
so PX4, Gazebo and the Python code share one network namespace. That keeps
gz-transport's discovery and the MAVLink API link on localhost, with no
multicast or port mapping across Docker networks to get wrong.

### MAVSDK package naming

In 2026 the PyPI name `mavsdk` moved to a new native binding (4.x) with a
different API. The familiar gRPC-based API continues as `mavsdk-grpc`
(`import mavsdk_grpc`). This project uses `mavsdk-grpc==4.0.5`, which keeps
the API most PX4 examples and docs use.

### 3.1 Simulation speed on a laptop

PX4 and Gazebo run in **lockstep**: simulated time only advances when every
sensor, including the camera, has produced its data. So a slow camera
render slows the whole simulation, not just the video. Measured on the
development laptop (i5-1345U, Iris Xe, WSL2), as real-time factor (RTF,
1.0 = real time) or camera frames per wall-clock second:

| Configuration | Result |
|---|---|
| x500, no camera, headless | RTF 1.0 |
| Stock `x500_mono_cam_down` (1280×960 @ 30 Hz), GPU via Mesa D3D12 | RTF 0.01–0.02 |
| Stock camera, CPU rendering (llvmpipe) | ≈0.9 frames/s (RTF ≈0.03) |
| `x500_vision_down` (640×480 @ 15 Hz), GPU via D3D12, headless | 5.9 frames/s (RTF ≈0.4) |
| `x500_vision_down`, llvmpipe, headless | 12.3 frames/s (RTF ≈0.8) |
| `x500_vision_down`, llvmpipe server + GUI on D3D12 | 5.8 frames/s (RTF ≈0.4) |

Conclusions:

- Under WSL the GPU path is slower than CPU rendering for the camera,
  because every frame has to be copied back from the GPU through the D3D12
  translation layer. The Gazebo server therefore renders with llvmpipe,
  while the GUI (which only draws to the screen) uses the GPU. The custom
  entrypoint starts them as separate processes so each gets its own driver.
- The GUI costs about half the speed. Batch experiments run with `GUI=0`.
- Because of lockstep, a slower simulation is still a *correct* simulation:
  PX4's estimator and controllers see consistent simulated time. The control
  code uses simulation time (frame timestamps) for its dt, never wall-clock
  time, so results don't depend on how fast the host is.
- Shadows are disabled in the worlds: shadow maps cost more than the camera
  image itself on these renderers.

## 4. Camera pipeline, pose estimation and coordinate frames

### Camera pipeline

Gazebo renders the camera and publishes each frame as a `gz.msgs.Image` on
`/world/<world>/model/x500_vision_down_0/link/camera_link/sensor/camera/image`.
`vland/camera.py` subscribes through the gz-transport Python bindings,
reshapes the bytes into a 480×640×3 numpy array and flips RGB to BGR for
OpenCV. It keeps only the latest frame: a controller wants the freshest
measurement, and a queue would only add latency. Each frame carries its
simulation timestamp, which the controller uses as its clock.

### Pose estimation

1. **Detect.** OpenCV's ArUco detector finds the marker's four corners and
   decodes its ID. Sub-pixel corner refinement makes the pose steadier.
2. **Solve.** `solvePnP` with `SOLVEPNP_IPPE_SQUARE` (a solver made for
   four corners of a square of known size) finds the marker pose that best
   explains those corners. The translation is the marker centre in the
   camera's optical frame, in metres.
3. **Intrinsics.** The simulated camera is an ideal pinhole: focal length
   `fx = (W/2) / tan(HFOV/2) = 320 / tan(45°) = 320 px`, principal point
   at the image centre, no lens distortion. On real hardware these come
   from a checkerboard calibration instead.

### Frames

| Frame | Axes | Used for |
|---|---|---|
| Camera optical (OpenCV) | x right, y down (in the image), z out of the lens | solvePnP output |
| Body FRD (PX4) | x forward, y right, z down | where the pad is relative to the drone |
| Level | body heading, but horizontal (roll and pitch removed) | the landing controller |
| Local NED (PX4) | x north, y east, z down | logging, mission waypoints |
| Gazebo world ENU | x east, y north, z up | ground truth only |

The camera looks straight down with the top of the image towards the nose,
so the optical-to-body rotation is a fixed permutation: image right = body
right, image down = body backward, lens axis = body down. This was checked
in flight: with the drone facing east and the pad 3 m east and 2 m north,
the marker appears in the top-left of the image (ahead and to the left).

**Tilt compensation.** The camera is fixed to the body, so when the drone
tilts to move, the image tilts too. At 6 m a 10° tilt shifts the apparent
pad position by about 1 m. Rotating the body-frame vector through the
vehicle attitude (`body_to_level`, `body_to_ned`) removes that error.

### Measured accuracy (milestone 2 flight)

`scripts/detect_marker.py` takes off 3.6 m from the pad, hovers at 6 m,
drifts over the pad and compares the vision estimate with Gazebo's ground
truth. Measured against PX4's body origin (`base_link`, 0.24 m above the
Gazebo model origin):

- The marker was detected in 210 of 211 frames, at 6–7 m range.
- Range error is about +1.5% (≈0.10 m at 7 m).
- At hover, 3.6 m from the pad, horizontal error in NED was ≈0.37 m. This
  comes from **heading**: PX4's yaw estimate was ≈96° while the true heading
  was 90°, and 3.6 m × sin 6° ≈ 0.38 m. Directly above the pad the horizontal
  error falls to 2–3 cm.

That heading result shaped the controller design: it works in the level
frame and commands body-frame velocities, so a heading-estimate error
cannot rotate its corrections.

## 5. Precision landing controller

```
            marker seen                  low (< 0.8 m) and centred (< 0.10 m)
  SEARCH ───────────────► TRACK ─────────────────────────────────► FINAL ──► PX4 LAND
    │                    ▲  │ marker lost                               (autopilot)
    │ 20 s               │  ▼
    │           seen     │ HOLD ── 1 s ──► CLIMB (0.5 m/s, ≤ 10 m)
    ▼           again ───┴───────────────────┘   │
  ABORT ◄──────────────── lost 15 s ─────────────┘
```

`vland/lander.py` is pure logic: `update(t, target, altitude) → command`,
where `target` is the pad position in the level frame (or `None`) and `t`
is simulation time. It has no I/O, so `tests/test_lander.py` drives it
through every transition without a simulator. `vland/landing.py` connects
it to the camera, detector and MAVSDK.

**The control law in TRACK**

- Horizontal: `v = kp · e`, with `kp = 0.8 /s`, limited to 1.0 m/s (0.4 m/s
  below 2 m). PX4's own position/velocity loops handle the vehicle
  dynamics; this outer loop only decides where to go.
- Vertical: descend (0.7 m/s, 0.3 m/s below 2 m) **only while inside a
  cone** around the pad axis: allowed error = 0.15 m + 0.12 × height. Far
  off-centre the drone centres first, then descends. The cone keeps the
  marker in view (the field of view shrinks on the ground as the drone
  descends) and stops a lateral error from turning into a landing miss.
- Commands are body-frame velocities, which PX4 interprets in the level
  (heading-aligned) frame, the same frame the target is measured in. Heading
  error doesn't enter the loop (see §4).

**Why hand over to PX4 for the last 0.8 m.** Below about half a metre the
0.8 m marker overflows the 90° camera view and can no longer be tracked.
Once the drone is low *and* centred, PX4's LAND mode holds the current
position and handles touchdown detection and disarm, which it does robustly.
This is also how PX4's built-in precision landing and most fiducial-landing
systems finish.

**A handover bug found in testing.** The first version left offboard mode
(which puts PX4 in HOLD) and then sent the land command. In the 9th run of
the first batch, the command was acknowledged but PX4 stayed in HOLD: the
drone sat on the pad, armed, for six minutes, and the script waited for a
touchdown that PX4 never declared. Two lessons, both now in the code
(`Vehicle.command_land`):

- Go straight from offboard to LAND, with no intermediate mode.
- An accepted command isn't a mode change. Confirm the mode from telemetry
  and retry if it doesn't appear, and put a timeout on every wait.

That batch's results were discarded; every number in the README comes from
runs of the fixed code.

**Losing the marker.** Most losses last a frame or two (glare, motion
blur, a partial occlusion), so the first response is to stop and HOLD.
After 1 s the drone CLIMBs slowly, because a higher camera sees a larger
patch of ground. When the marker reappears it goes straight back to TRACK.
After 15 s lost it ABORTs (in this demo: a normal landing, flagged in the
results). The response is tested in simulation by hiding the marker from
the controller for a configurable window (`--occlusion start,duration`).

**Alternative considered:** PX4's built-in precision landing (and the
newer `vision_target_estimator` module, which runs in this PX4 build). It
takes `LANDING_TARGET` MAVLink messages and fuses them in a Kalman filter
with IMU data, which handles latency and noise better than a raw
P-controller. On a production vehicle that's what I'd build on. Here the
goal is to show and explain the control loop, so it's explicit Python.

## 6. Measuring accuracy

Landing error is measured from Gazebo's ground-truth poses (vehicle vs. pad),
not from the camera, so the measurement is independent of the system being
measured. Every number in the README comes from a logged run in `results/`.

## 7. Mission plan

`missions/powerline.yaml` holds everything about *where*: cruise altitude
and speed, inspection clearance and settle time, the pad position and
marker ID, and the ordered tower list. `vland/mission.py` parses it into
frozen dataclasses and validates it up front: required fields, numeric
types, unique tower IDs, and a safety rule that the cruise altitude must be
at least the highest inspection altitude, so a transit can't clip a tower.
A bad plan fails on the ground with a clear message, not mid-flight.

The same file generates the Gazebo world (`tools/gen_powerline_world.py`):
towers, crossarms, insulators and three conductors per span are placed from
the plan. The simulated world and the plan can't disagree.

Flight (`scripts/run_mission.py`):

1. Take off and climb to cruise altitude.
2. For each tower: transit at cruise speed, descend to tower height plus
   clearance, settle for `hover_s` (simulation time), save an annotated
   image, climb back to cruise.
3. Return to the pad, descend to the approach altitude and run the same
   precision-landing routine as milestone 3.
4. Measure landing error against ground truth and write `summary.json`.

`Vehicle.fly_to` uses velocity setpoints along the straight line at the
planned speed, slowing over the last metres. Bare position setpoints would
let PX4 fly at its maximum speed.

**Simplifications.** The towers are simple box geometry and the conductors
are straight (no catenary sag). The camera points down, so inspection images
are taken from above the tower top, not from the side as a real inspection
camera would.

## 8. Limitations and the gap to docking on a live line

This is a simulation demo of the *pattern*: perception → relative pose →
closed-loop guidance → handover to the autopilot. Docking on an energised
conductor is a much harder version of the same problem:

- **Wind and turbulence.** The sim has no wind. Gusts at conductor height,
  plus the line itself swinging, mean the target moves. You'd estimate the
  target's motion (a Kalman filter over target position and velocity, as
  PX4's `vision_target_estimator` does) and add feed-forward, not just a
  P-controller on the current error.
- **GPS-denied, relative navigation.** Near a line the job is *relative*
  positioning to the conductor at centimetre level, which GNSS can't give
  (multipath near steel structures, no RTK in remote areas). Relative
  sensing, from the camera, lidar or the line's own magnetic field, becomes
  the primary navigation source, with GNSS only for transit.
- **EMI near conductors.** High-voltage lines produce strong, varying
  magnetic fields, which corrupt the magnetometer and with it the heading.
  This demo already measured a 6° heading-estimate error with *no* EMI, and
  the controller was designed to be independent of heading for exactly that
  reason. Near a line you'd go further: no magnetometer fusion close in,
  heading from vision or a dual-antenna GNSS.
- **No fiducials on the line.** A marker on a pad is the easy case. A real
  docking system has to detect the conductor itself, a thin, low-contrast
  object against sky or terrain: learned detectors, line fitting, or lidar.
- **Edge compute.** All perception here runs on a desktop CPU at 15 Hz.
  On the aircraft it has to fit a companion computer's power and thermal
  budget alongside logging and comms, which pushes towards fixed-latency
  pipelines and hardware acceleration.
- **Latency and timing.** The camera frame and the attitude used to rotate
  it are paired by "latest available", not by timestamp. That's fine at
  these speeds but should be interpolated properly for aggressive motion.
- **Real cameras.** Intrinsics from calibration, lens distortion, rolling
  shutter, exposure changes and glare off the pad.
- **Safety architecture.** Geofences, a hard minimum distance to
  conductors, link-loss behaviour and an independent abort path. Here,
  ABORT just lands normally.

## 9. Investigation: intermittent PX4 SITL crash

**Symptom.** Occasionally the `sim` container died mid-flight with exit
code 139 (SIGSEGV in PX4, which is the container's main process). Nothing
was logged; telemetry froze at its last value and the camera showed the
drone tumbling. It happened 3 times in roughly 60 flights: once after a
landing in milestone 1, once in landing run seed 25, and once 1.5 minutes
into a mission. Docker reported no out-of-memory kill.

**Gathering evidence.** A crash that rare can't be debugged by retrying.
I built a throwaway debug image (the project image plus `gdb`), ran PX4
under `gdb -batch` so a SIGSEGV prints every thread's stack, and made
`tools/batch_landings.sh` keep each run's simulator log. Three missions and
19 landings under gdb ran clean; run 114 crashed and the backtrace was
captured ([`docs/px4_logger_crash.txt`](docs/px4_logger_crash.txt)):

```
Thread 187 "logger" received signal SIGSEGV, Segmentation fault.
#0  pthread_mutex_lock () from libc.so.6
#1  px4_sem_post ()
#2  ?? ...
```

The fault is in PX4's own onboard ULog **logger** thread: posting a
semaphore whose mutex is invalid. That's consistent with a lifetime or
initialisation bug around the semaphore, but that is inference, not
something the trace proves. The binary is
stripped, so the trail ends there. Pinning it down further would mean a
PX4 debug build from source; reporting it upstream with this trace is the
right next step.

**Mitigation.** This project doesn't use PX4's ULog files, so the logger
is disabled in SITL via PX4's environment parameter override
(`PX4_PARAM_SDLOG_BACKEND=0` in `compose.yaml`; `ULOG=1` turns it back on).
That removes the crashing thread entirely, rather than masking the symptom.
Every landing result in the README was re-run with this configuration.
Separately, scripts now record a crashed run as `ERROR` instead of hanging
or vanishing from the statistics.
