# Getting started: FloorScan Bot in Gazebo + ROS 2

You already have ROS 2 Jazzy + Gazebo Harmonic working (confirmed earlier with
the talker/listener test and the TurtleBot3 world). This picks up from there.

## 1. Put the model somewhere Gazebo can find it

```bash
mkdir -p ~/floorscan_sim
cp floorscan_bot.sdf ~/floorscan_sim/
```

## 2. Make sure slam_toolbox is installed

```bash
sudo apt install ros-jazzy-slam-toolbox -y
```

## 3. Launch an empty world (terminal 1)

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

## 4. Spawn FloorScan Bot into it (terminal 2)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_sim create -file ~/floorscan_sim/floorscan_bot.sdf -name floorscan_bot -x 0 -y 0 -z 0.1
```

You should see the little orange sensor puck on top of the chassis in Gazebo.

## 5. Bridge topics into ROS 2 (terminal 3)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

## 6. Confirm the scan is coming through (terminal 4)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /scan --once
```

You should see a `LaserScan` message with 360 range values. If `ranges` is
full of `inf`, that's correct in an empty world — nothing to hit yet.

## 7. Add something to actually map

Easiest test: in Gazebo, use the "Insert shapes" panel (or just re-launch
step 3 with a world that already has walls, e.g. the TurtleBot3 world you
installed earlier) so the scan has real obstacles to detect.

## 8. Start SLAM (terminal 5)

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

## 9. Drive it around and watch the map build (terminal 6)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 10. Visualize (terminal 7)

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

Set **Fixed Frame** to `map`, add a **LaserScan** display on `/scan`, and
add a **Map** display on `/map`. Drive the robot around with teleop and
watch the floor plan fill in.

## 11. Save the map once you're happy with it

```bash
ros2 run nav2_map_server map_saver_cli -f ~/floorscan_map
```

## Troubleshooting notes

- If Gazebo won't open a window, that's the WSLg GPU issue from earlier —
  try `export LIBGL_ALWAYS_SOFTWARE=1` before launching.
- If `ros2 run ros_gz_sim create` fails saying it can't find the file, double
  check the path is absolute (`~/floorscan_sim/floorscan_bot.sdf`, not a
  relative path from some other directory).
- If `/scan` never shows up in `ros2 topic list`, the bridge terminal is the
  one to check first — a typo in the topic type string is the most common
  cause of a bridge silently not connecting.

# Tutorial
https://gazebosim.org/docs/latest/building_robot/
https://gazebosim.org/api/transport/15/messages.html

# Run world
gz sim building_robot.sdf

# Gz setup - this exposes gz command & make it permanent
source /opt/ros/jazzy/setup.bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

# Move bot
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"

# Lidar messages
gz topic -e -t /lidar

# Fix build
export CMAKE_PREFIX_PATH="/opt/ros/jazzy/opt/gz_transport_vendor/extra_cmake:$CMAKE_PREFIX_PATH"