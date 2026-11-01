#!/usr/bin/env bash
set -Eeo pipefail

# ROS setup scripts may reference unset AMENT variables; disable nounset while sourcing.
source /opt/ros/jazzy/setup.bash
source /opt/pai_ws/install/setup.bash
set -u
mkdir -p /tmp/runtime-root /recordings
chmod 700 /tmp/runtime-root

pids=()
cleanup() {
  echo "[PAI] Stopping processes..."
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[PAI] Starting virtual X display..."
Xvfb :99 -screen 0 1600x900x24 -ac +extension GLX +render -noreset &
pids+=("$!")

for _ in $(seq 1 50); do
  if xdpyinfo -display :99 >/dev/null 2>&1; then break; fi
  sleep 0.2
done
xdpyinfo -display :99 >/dev/null

openbox --display :99 >/tmp/openbox.log 2>&1 &
pids+=("$!")
x11vnc -display :99 -rfbport 5900 -forever -shared -localhost -nopw \
  -xkb -quiet >/tmp/x11vnc.log 2>&1 &
pids+=("$!")
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 &
pids+=("$!")

echo "[PAI] Starting telemetry dashboard at http://localhost:6081 ..."
python3 /opt/pai_ws/ai_dashboard.py &
pids+=("$!")

echo "[PAI] Starting Gazebo server, ROS/Gazebo bridge, and AI controller..."
ros2 launch sense_before_touch_ros2 rescue_demo.launch.py &
ros_pid="$!"
pids+=("$ros_pid")

# The Gazebo GUI is separate so that rendering problems do not stop physics.
sleep 6
echo "[PAI] Starting Gazebo GUI in software rendering mode (OGRE 1)..."
gz sim -g -v 3 --render-engine ogre >/tmp/gazebo_gui.log 2>&1 &
pids+=("$!")

echo "[PAI] Gazebo: http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
echo "[PAI] AI dashboard: http://localhost:6081"
echo "[PAI] Inspect ROS status: docker compose exec pai bash -lc 'source /opt/ros/jazzy/setup.bash; source /opt/pai_ws/install/setup.bash; ros2 topic echo /rescue_ai/status'"
wait "$ros_pid"
