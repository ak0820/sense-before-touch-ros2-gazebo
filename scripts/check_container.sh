#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")/.."
docker compose exec -T pai bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/pai_ws/install/setup.bash && \
   echo "[Architecture]" && uname -m && \
   echo "[Gazebo]" && gz sim --versions && \
   echo "[ROS package]" && ros2 pkg prefix sense_before_touch_ros2 && \
   echo "[Unit test]" && python3 /opt/pai_ws/tests/test_core.py && \
   echo "[ROS topic list]" && ros2 topic list'
