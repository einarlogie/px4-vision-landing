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
| Vehicle | `x500_mono_cam_down` | Stock PX4 quad with a downward camera; no custom airframe needed |
| Vehicle API | MAVSDK-Python, offboard velocity setpoints | Small, explicit API over MAVLink; same protocol as a real companion computer |
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

## 4. Coordinate frames *(planned, verified in milestone 2)*

To be filled in with the verified conventions.

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
