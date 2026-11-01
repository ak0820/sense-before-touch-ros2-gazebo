# Sense Before Touch

**Uncertainty-aware autonomous warehouse inspection with ROS 2 Jazzy + Gazebo Harmonic.**

An autonomous differential-drive rover navigates between three simulated packages, uses a Gaussian classifier over **synthetic multispectral sensor signals**, re-inspects low-confidence readings by moving to a closer observation pose, and returns to its start position. The data are **not Gazebo camera images**. The robot does not grasp, transport, or physically sort objects.

## Demonstration and confirmed results

- Gazebo physics simulation and ROS 2 navigation ran on an **Apple Silicon Mac using Docker Desktop (linux/arm64)**.
- Observed result: `MISSION COMPLETE: 3/3 correct, 6 active inspections` (correct versus the internally defined synthetic labels).
- `active inspections` means **6 additional inspections**, on top of the initial observation of each item (9 observations in total).
- Confidence threshold: **0.77**; up to **2 additional inspections per package**.
- Published dashboard telemetry: `/rescue_ai/telemetry` (`std_msgs/String` JSON).
- For the final video and explanations, see the publication links added by the submitter.

## System overview

```
Gazebo Harmonic (world/rover/diff-drive)
    | ground truth pose                 ^ velocity command
    v                                   |
ros_gz_bridge <------------------------> ROS 2 controller
                                          |  A* + clearance-aware waypoints
                                          |  Gaussian material classifier
                                          |  confidence-based re-inspection
                                          v
                                    /rescue_ai/telemetry
                                          |
                                          v
                                 Browser AI Dashboard (:6081)
```

## Run on Apple Silicon Mac

Prerequisites: Docker Desktop with Linux containers; enough memory for the software-rendered Gazebo GUI.

```bash
cd Sense_Before_Touch_GitHub
docker compose -p sense_before_touch_mac_docker up --build -d
```

Open:

- Gazebo GUI (noVNC): http://localhost:6080/vnc.html?autoconnect=true&resize=scale
- AI decision dashboard: http://localhost:6081

To restart the scenario from the initial state:

```bash
docker compose -p sense_before_touch_mac_docker restart pai
```

To read the mission logs:

```bash
docker compose -p sense_before_touch_mac_docker logs -f pai
```

Stop the environment:

```bash
docker compose -p sense_before_touch_mac_docker down
```

## Test

Pure Python planner/classifier and telemetry contract tests (no Gazebo required):

```bash
python -m pytest -q tests
```

The included `gazebo_world_layout.png` is a **world-layout illustration derived from SDF**, not a screenshot of Gazebo execution.

## Project structure

| Path | Role |
|---|---|
| `sense_before_touch_ros2/worlds/rescue_warehouse.sdf` | Simulated warehouse, rover, packages, walls and collision models |
| `sense_before_touch_ros2/sense_before_touch_ros2/controller.py` | Mission state machine, speed commands, ROS 2 telemetry |
| `sense_before_touch_ros2/sense_before_touch_ros2/core.py` | Synthetic Gaussian classifier and A* planner |
| `sense_before_touch_ros2/launch/rescue_demo.launch.py` | ROS 2 and Gazebo launch / bridge |
| `scripts/ai_dashboard.py` | HTTP server for live AI dashboard |
| `scripts/dashboard/index.html` | Browser dashboard UI |
| `compose.yaml`, `Dockerfile` | Linux/arm64 container environment |
| `tests/` | Automated unit tests |

## Technical design

**AI model:** The Gaussian classifier fits mean and diagonal variance for each of the three synthetic material signatures using generated training samples. Inference evaluates a distance- and noise-dependent observation and computes a confidence-like normalized score. It is a demonstration of uncertainty-aware action selection, **not a validated calibrated confidence estimate for real-world sensor data**.

**Navigation:** 8-connected A* on a 0.2 m grid; the occupancy map includes warehouse obstacles and all three boxes with 0.55 m collision clearances. Line-of-sight smoothing generates waypoints. Velocity commands track the planned waypoints using world-frame ground truth pose from Gazebo (`/model/rescue_rover/ground_truth`).

**Safety:** Outer collision walls; conservative velocity limit; emergency stops for out-of-bounds/invalid/stale pose or planning failure.

**Limitations:** No RGB/image recognition, no manipulation, no dynamic obstacle avoidance, no real-world validation, no calibrated uncertainty assessment. Ground-truth pose and synthetic labels simplify the scenario.

## Privacy and publication

The code uses only generated simulation content. The ROS package includes the non-real maintainer address `maintainer@example.invalid`, required in the package manifest; it is not a personal email address. Before publishing, review the GitHub repository and recorded video for any identifying information.

## License

MIT (see `LICENSE`).
