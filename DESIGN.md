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

## 5. Precision landing controller *(planned)*

A state machine in Python that sends velocity setpoints:
`SEARCH → ALIGN → DESCEND → LAND`, plus `LOST → HOLD → CLIMB → re-acquire`.
Kept as a pure function of (state, measurement, time) so it can be unit-tested
without the simulator.

**Alternative considered:** PX4's built-in precision landing, which takes
`LANDING_TARGET` MAVLink messages and runs its own estimator. That's closer to
production but hides the control logic behind PX4 parameters, and the goal
here is to show the control loop explicitly.

## 6. Measuring accuracy

Landing error is measured from Gazebo's ground-truth poses (vehicle vs. pad),
not from the camera, so the measurement is independent of the system being
measured. Every number in the README comes from a logged run in `results/`.

## 7. Mission plan *(planned)*

YAML file with the tower positions, cruise altitude, standoff offset and
capture settings, parsed into validated dataclasses. Control code never
hard-codes waypoints.

## 8. Limitations and real-world gaps *(planned)*

Wind and gusts, GPS-denied relative navigation near conductors, EMI effects on
the magnetometer and links, edge compute budget, and fiducial markers vs.
detecting the line itself.
