#!/usr/bin/env bash
set -u

echo "[INFO] Stopping ROS nodes (if roscore is reachable)..."
if command -v rosnode >/dev/null 2>&1; then
  rosnode kill -a >/dev/null 2>&1 || true
fi

echo "[INFO] Terminating Gazebo and ROS processes..."
patterns=(
  "gzserver"
  "gzclient"
  "gazebo"
  "roslaunch"
  "roscore"
  "rosmaster"
  "rosout"
  "rosrun"
)

for p in "${patterns[@]}"; do
  pkill -f "$p" >/dev/null 2>&1 || true
done

sleep 1

echo "[INFO] Forcing remaining Gazebo/ROS processes (if any)..."
for p in "${patterns[@]}"; do
  pgrep -f "$p" >/dev/null 2>&1 && pkill -9 -f "$p" >/dev/null 2>&1 || true
done

echo "[DONE] Gazebo and ROS processes have been stopped."