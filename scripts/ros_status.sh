#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")/.."
docker compose exec pai bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/pai_ws/install/setup.bash && ros2 topic echo /rescue_ai/status'
