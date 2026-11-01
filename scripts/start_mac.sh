#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")/.."
if ! command -v docker >/dev/null; then
  echo "Docker not found. Please install and start Docker Desktop for Apple Silicon." >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  echo "Docker Desktop is not running. Please start it first." >&2
  exit 1
fi
if [[ "$(uname -m)" != "arm64" ]]; then
  echo "Warning: This package targets Apple Silicon (arm64)." >&2
fi
docker compose up --build -d
echo
echo "Open in your Mac browser:"
echo "http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
echo "View logs: docker compose logs -f pai"
echo "ROS status: ./scripts/ros_status.sh"
if command -v open >/dev/null; then
  open 'http://localhost:6080/vnc.html?autoconnect=true&resize=scale' || true
fi
