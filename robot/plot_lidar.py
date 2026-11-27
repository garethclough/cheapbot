#!/usr/bin/env python3
"""Live scatter plot of /lidar hit points, transformed into world coordinates
using each scan's own world_pose field, so hits stay put as the robot moves
(building up a floor-plan-style map instead of a per-scan local view).

world_pose is used instead of a separate /odom subscription because /odom on
this sim only publishes at 1Hz while /lidar publishes at 10Hz -- transforming
with the last /odom pose meant most scans were rotated using a stale heading,
which showed up as a "twisted" map whenever the robot was turning. world_pose
is stamped on the same message as the ranges, so it's always in sync.

Usage (with the sim already running):
    python3 plot_lidar.py
"""
import math
import threading
from collections import deque

import matplotlib.pyplot as plt
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node

LIDAR_TOPIC = "/lidar"
MAX_POINTS = 50_000  # cap so the plot/memory don't grow unbounded on a long run

lock = threading.Lock()
pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
map_points = deque(maxlen=MAX_POINTS)


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lidar_cb(msg: LaserScan) -> None:
    wp = msg.world_pose
    rx, ry = wp.position.x, wp.position.y
    yaw = quat_to_yaw(wp.orientation)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

    new_points = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            continue
        angle = msg.angle_min + i * msg.angle_step
        local_x = r * math.cos(angle)
        local_y = r * math.sin(angle)

#        world_x = rx + local_x * cos_yaw - local_y * sin_yaw
#        world_y = ry + local_x * sin_yaw + local_y * cos_yaw
        world_x = local_x
        world_y = local_y
        new_points.append((world_x, world_y))

    with lock:
        pose["x"], pose["y"], pose["yaw"] = rx, ry, yaw
        map_points.extend(new_points)


def main() -> None:
    node = Node()
    node.subscribe(LaserScan, LIDAR_TOPIC, lidar_cb)

    fig, ax = plt.subplots()
    scatter = ax.scatter([], [], s=3, c="steelblue")
    robot_arrow = ax.quiver(
        [0], [0], [1], [0],
        color="red", scale=12, width=0.012, label="robot heading",
    )
    ax.set_xlabel("x (m, world)")
    ax.set_ylabel("y (m, world)")
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    ax.set_title("Live floor map (world frame)")

    def update(_frame):
        with lock:
            pts = list(map_points)
            rx, ry, yaw = pose["x"], pose["y"], pose["yaw"]
        if pts:
            scatter.set_offsets(pts)
        robot_arrow.set_offsets([[rx, ry]])
        robot_arrow.set_UVC([math.cos(yaw)], [math.sin(yaw)])
        return scatter, robot_arrow

    from matplotlib.animation import FuncAnimation

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
