#!/bin/bash
# start_floorscan.sh
# Launches Gazebo (with the room world), spawns FloorScan Bot, and starts
# the ROS 2 <-> Gazebo bridge, all in one command. Ctrl+C stops everything.

set -e

# ---- Paths: automatically use the directory this script lives in ----
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="$SIM_DIR/empty_world.sdf"
ROBOT_FILE="$SIM_DIR/floorscan_bot.sdf"
WORLD_NAME="floorscan_room"
ROBOT_NAME="floorscan_bot"

# ---- Sanity checks ----
if [ ! -f "$WORLD_FILE" ]; then
  echo "ERROR: world file not found at $WORLD_FILE"
  exit 1
fi
if [ ! -f "$ROBOT_FILE" ]; then
  echo "ERROR: robot file not found at $ROBOT_FILE"
  exit 1
fi

source /opt/ros/jazzy/setup.bash

# ---- Software rendering isn't needed for the real bug we found (a
#      duplicate plugin registration), but leaving this here, commented
#      out, in case WSLg rendering issues show up again later. ----
# export LIBGL_ALWAYS_SOFTWARE=1

# ---- Clean up any stray Gazebo/bridge processes from a previous run,
#      and actually WAIT until they're gone before continuing. ----
echo "Cleaning up any previous run..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "ros2 launch ros_gz_sim" 2>/dev/null || true

for i in $(seq 1 15); do
  if ! pgrep -f "gz sim" > /dev/null 2>&1; then
    echo "Old processes cleared."
    break
  fi
  sleep 1
  if [ "$i" -eq 15 ]; then
    echo "ERROR: old Gazebo process(es) won't die. Run 'pkill -9 -f \"gz sim\"' manually and try again."
    exit 1
  fi
done

# ---- Track background process PIDs so we can kill them all on exit ----
PIDS=()
cleanup() {
  echo ""
  echo "Shutting down..."
  for pid in "${PIDS[@]}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
  pkill -9 -f "ros2 launch ros_gz_sim" 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM EXIT

# ---- 1. Launch Gazebo with the room world (combined server+GUI, -r = run immediately) ----
echo "Starting Gazebo with $WORLD_NAME..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r $WORLD_FILE" &
PIDS+=($!)

echo "Waiting for Gazebo to come up..."
for i in $(seq 1 30); do
  if gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/"; then
    echo "Gazebo is ready."
    break
  fi
  sleep 1
  if [ "$i" -eq 30 ]; then
    echo "ERROR: Gazebo did not come up within 30 seconds."
    exit 1
  fi
done

# ---- 2. Spawn the robot ----
echo "Spawning $ROBOT_NAME..."
ros2 run ros_gz_sim create -world "$WORLD_NAME" -file "$ROBOT_FILE" -name "$ROBOT_NAME" -x 0 -y 0 -z 0.1

# ---- 3. Start the ROS 2 <-> Gazebo bridge ----
echo "Starting ROS 2 bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
PIDS+=($!)

echo ""
echo "=========================================="
echo " FloorScan Bot simulation is running."
echo " Gazebo window should be visible now."
echo " Try in a NEW terminal:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo " Press Ctrl+C here to stop everything."
echo "=========================================="

wait