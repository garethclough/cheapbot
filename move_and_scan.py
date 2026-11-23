#!/usr/bin/env python3
"""
move_and_scan.py - turn the robot to a heading in Gazebo, then take a lidar scan.

Talks to Gazebo directly (gz-transport), no lidar_node needed:
  - reads the robot's heading from /odom
  - turns it with /cmd_vel until it faces the requested angle
  - waits for the robot to settle, then grabs one /lidar scan
  - prints a summary and draws the points, in world coordinates, on a bitmap
    (scan_map.png by default); optionally saves CSV and/or shows a plot

Run with the system python3 (the .venv lacks protobuf, which gz needs),
with ROS sourced and the sim running (press play in Gazebo):
    source /opt/ros/jazzy/setup.bash
    python3 move_and_scan.py                     # 4 scans: 0, 90, 180, 270 deg
    python3 move_and_scan.py 90                  # one scan facing 90 deg
    python3 move_and_scan.py 90 --bitmap a.png   # bitmap to another file
    python3 move_and_scan.py -45 --out scan.csv  # also save points to CSV
    python3 move_and_scan.py 180 --plot          # also show the points

Angles are the odometry heading in degrees: 0 = the direction the robot faced
when the sim started, positive = anticlockwise (left).

Don't run this while lidar_node is driving the robot: both would send
/cmd_vel and fight over it.
"""

import argparse
import csv
import math
import sys
import threading
import time

from PIL import Image, ImageDraw

from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V

CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
LIDAR_TOPIC = "/lidar"
WORLD_POSE_TOPIC = "/world/car_world/pose/info"

# Lidar position relative to the point odometry tracks (building_robot.sdf:
# chassis at x=0.5 in the model, lidar_frame at x=0.8 on the chassis).
LIDAR_OFFSET_X = 1.3

# Bitmap: a fixed square of the world centred on (0, 0), so every run is
# drawn at the same scale and position. building_robot.sdf's walls are at
# +/-5 m, so +/-6 m shows the whole room with a margin.
BITMAP_EXTENT = 6.0     # metres from the centre to each edge
PIXELS_PER_METRE = 50   # 12 m * 50 = 600 x 600 pixels

# Turn control, same as lidar_node: speed proportional to the angle left,
# capped, with a minimum so it doesn't stall just short of the target.
TOLERANCE = math.radians(0.5)   # stop when within this of the target
MAX_TURN_SPEED = 0.5            # rad/s
MIN_TURN_SPEED = 0.03           # rad/s
GAIN = 1.5
LOOP_PERIOD = 0.05              # s (20 Hz)
SETTLE_TIME = 1               # s to wait after stopping, before scanning


def normalize_angle(a):
    """Wrap an angle into -pi..pi."""
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Robot:
    """Latest odometry and lidar data from Gazebo, plus a way to drive."""

    def __init__(self):
        # gz-transport calls the callbacks on its own threads, so everything
        # they write is protected by this lock.
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = None   # None until the first /odom
        self.worldPoseX = self.worldPoseY = self.worldPoseYaw = None   # None until the first /world/pose
        self.scan = None
        self.scan_time = 0.0                # when the latest scan arrived

        self.node = Node()
        self.cmd_pub = self.node.advertise(CMD_VEL_TOPIC, Twist)
        if not self.node.subscribe(Odometry, ODOM_TOPIC, self.on_odom):
            raise RuntimeError(f"could not subscribe to {ODOM_TOPIC}")
        if not self.node.subscribe(LaserScan, LIDAR_TOPIC, self.on_scan):
            raise RuntimeError(f"could not subscribe to {LIDAR_TOPIC}")
        if not self.node.subscribe(Pose_V, WORLD_POSE_TOPIC, self.on_world_pose):
            raise RuntimeError(f"could not subscribe to {WORLD_POSE_TOPIC}")

    def on_odom(self, msg):
        with self.lock:
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.yaw = quat_to_yaw(msg.pose.orientation)

    def on_world_pose(self, msg):
        with self.lock:
            for pose in msg.pose:
                if pose.name == "vehicle_blue":
                    self.worldPoseX = pose.position.x
                    self.worldPoseY = pose.position.y
                    self.worldPoseYaw = quat_to_yaw(pose.orientation)
                    self.haveWorldPose = True
                    return

    def on_scan(self, msg):
        with self.lock:
            self.scan = msg
            self.scan_time = time.monotonic()

    def pose(self):
        with self.lock:
            return self.worldPoseX, self.worldPoseY, self.worldPoseYaw
#            return self.x, self.y, self.yaw

    def set_speed(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.set_speed(0.0, 0.0)

    def wait_for_odom(self, timeout):
        deadline = time.monotonic() + timeout
        while self.pose()[2] is None:
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def turn_to(self, target, timeout):
        """Turn on the spot until facing `target` (radians). True if reached."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, _, yaw = self.pose()
            remaining = normalize_angle(target - yaw)
            if abs(remaining) < TOLERANCE:
                self.stop()
                return True
            speed = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, GAIN * remaining))
            if abs(speed) < MIN_TURN_SPEED:
                speed = math.copysign(MIN_TURN_SPEED, speed)
            self.set_speed(0.0, speed)
            time.sleep(LOOP_PERIOD)
        self.stop()
        return False

    def next_scan(self, timeout):
        """Wait for a scan that arrives after now (i.e. taken while stopped)."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            with self.lock:
                if self.scan is not None and self.scan_time > start:
                    return self.scan
            time.sleep(0.02)
        return None


def scan_to_points(scan, x, y, yaw):
    """Valid hits as (angle_deg, range_m, world_x, world_y) tuples."""
    points = []
    lx = x + LIDAR_OFFSET_X * math.cos(yaw)   # lidar position in the world
    ly = y + LIDAR_OFFSET_X * math.sin(yaw)
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_step   # relative to the robot
        world_angle = yaw + angle
        points.append((math.degrees(angle), r,
                       lx + r * math.cos(world_angle),
                       ly + r * math.sin(world_angle)))
    return points


# One colour per reading, so you can see which scan each wall came from.
SCAN_COLOURS = [(31, 119, 180), (230, 120, 20), (40, 160, 40), (150, 60, 190),
                (200, 40, 40), (120, 80, 40)]


def draw_bitmap(path, scans, extent, ppm):
    """Draw every scan's hits in world coordinates on one PNG, with a 1 m grid.

    scans: list of dicts with keys points, x, y, yaw.
    """
    size = int(2 * extent * ppm)
    image = Image.new("RGB", (size, size), "white")
    draw = ImageDraw.Draw(image)

    def to_pixel(wx, wy):
        # World y points up, but pixel y points down, so flip it.
        return ((wx + extent) * ppm, (extent - wy) * ppm)

    # Grid: a light line every metre, darker axes through (0, 0), labelled.
    for m in range(-int(extent), int(extent) + 1):
        colour = (150, 150, 150) if m == 0 else (225, 225, 225)
        px, _ = to_pixel(m, 0)
        _, py = to_pixel(0, m)
        draw.line([(px, 0), (px, size)], fill=colour)
        draw.line([(0, py), (size, py)], fill=colour)
        if m != 0 and m % 2 == 0 and abs(m) < extent:
            draw.text((px + 2, to_pixel(0, 0)[1] + 2), f"{m}", fill=(120, 120, 120))
            draw.text((to_pixel(0, 0)[0] + 2, py + 2), f"{m}", fill=(120, 120, 120))
    draw.text((size - 14, to_pixel(0, 0)[1] - 14), "x", fill=(120, 120, 120))
    draw.text((to_pixel(0, 0)[0] + 4, 2), "y", fill=(120, 120, 120))

    for i, scan in enumerate(scans):
        colour = SCAN_COLOURS[i % len(SCAN_COLOURS)]
        x, y, yaw = scan["x"], scan["y"], scan["yaw"]

        # Hits: small dots in this scan's colour (outside the area: skipped).
        for _, _, wx, wy in scan["points"]:
            if abs(wx) <= extent and abs(wy) <= extent:
                px, py = to_pixel(wx, wy)
                draw.rectangle([px - 1, py - 1, px + 1, py + 1], fill=colour)

        # Heading for this scan: a 1 m line from the robot, same colour.
        rx, ry = to_pixel(x, y)
        hx, hy = to_pixel(x + math.cos(yaw), y + math.sin(yaw))
        draw.line([(rx, ry), (hx, hy)], fill=colour, width=2)

        # Legend line at the top.
        draw.text((4, 4 + 12 * i),
                  f"scan {i + 1}: {math.degrees(yaw):.1f} deg, "
                  f"{len(scan['points'])} hits", fill=colour)

    # Robot position(s): red dots, drawn last so they're on top.
    for scan in scans:
        rx, ry = to_pixel(scan["x"], scan["y"])
        draw.ellipse([rx - 4, ry - 4, rx + 4, ry + 4], fill="red")

    draw.text((4, size - 16), "1 grid square = 1 m", fill="black")
    image.save(path)


def take_reading(robot, angle_deg, timeout):
    """Turn to angle_deg and scan. Returns a scan dict, or None on failure."""
    x, y, yaw = robot.pose()
    print(f"Position: x={x:.4f}, y={y:.4f}, yaw={math.degrees(yaw):.3f}")
    target = normalize_angle(math.radians(angle_deg))
    print(f"Facing {math.degrees(yaw):.1f} deg, turning to {math.degrees(target):.1f} deg...")

    if not robot.turn_to(target, timeout=timeout):
        print(f"  Didn't reach the target within {timeout:.0f} s")
        return None

    time.sleep(SETTLE_TIME)
    x, y, yaw = robot.pose()
    print(f"Position after turn: x={x:.4f}, y={y:.4f}, yaw={math.degrees(yaw):.3f}")
    scan = robot.next_scan(timeout=5.0)
    if scan is None:
        print(f"  No {LIDAR_TOPIC} messages - is the lidar publishing?")
        return None

    points = scan_to_points(scan, x, y, yaw)
    print(f"  Scanned at {math.degrees(yaw):.1f} deg "
          f"({math.degrees(normalize_angle(target - yaw)):+.2f} from target): "
          f"{len(points)} hits", end="")
    if points:
        nearest = min(points, key=lambda p: p[1])
        print(f", nearest {nearest[1]:.2f} m at {nearest[0]:+.1f} deg", end="")
    print()
    return {"points": points, "x": x, "y": y, "yaw": yaw}


def main():
    parser = argparse.ArgumentParser(
        description="Turn the robot to one or more headings in Gazebo and take "
                    "a lidar scan at each.")
    parser.add_argument("angles", type=float, nargs="*", default=[0, 90, 180, 270],
                        help="headings to scan at, in degrees (0 = start "
                             "direction, + = left). Default: 0 90 180 270")
    parser.add_argument("--bitmap", metavar="FILE", default="scan_map.png",
                        help="PNG to draw the points on (default scan_map.png)")
    parser.add_argument("--extent", type=float, default=BITMAP_EXTENT,
                        help=f"bitmap shows +/- this many metres around (0, 0) "
                             f"(default {BITMAP_EXTENT:g})")
    parser.add_argument("--out", metavar="FILE",
                        help="save the scan points to this CSV file")
    parser.add_argument("--plot", action="store_true",
                        help="show the points in a matplotlib window")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="seconds allowed for each turn (default 30)")
    args = parser.parse_args()

    robot = Robot()
    scans = []
    try:
        print("Waiting for odometry...")
        if not robot.wait_for_odom(timeout=5.0):
            print(f"No {ODOM_TOPIC} messages - is the sim running and playing?")
            return 1

        for angle in args.angles:
            reading = take_reading(robot, angle, args.timeout)
            if reading is None:
                break
            scans.append(reading)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        robot.stop()   # never leave the robot spinning

    if not scans:
        return 1

    # Save whatever was collected, even if a later reading failed.
    draw_bitmap(args.bitmap, scans, args.extent, PIXELS_PER_METRE)
    total = sum(len(s["points"]) for s in scans)
    print(f"Drew {len(scans)} scan(s), {total} points, on {args.bitmap}")

    if args.out:
        with open(args.out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["scan", "heading_deg", "angle_deg", "range_m",
                             "world_x", "world_y"])
            for i, s in enumerate(scans):
                heading = math.degrees(s["yaw"])
                for p in s["points"]:
                    writer.writerow([i + 1, f"{heading:.2f}", *p])
        print(f"Saved {total} points to {args.out}")

    if args.plot:
        import matplotlib.pyplot as plt
        for i, s in enumerate(scans):
            plt.scatter([p[2] for p in s["points"]], [p[3] for p in s["points"]],
                        s=2, label=f"scan {i + 1} ({math.degrees(s['yaw']):.0f} deg)")
        plt.plot([s["x"] for s in scans], [s["y"] for s in scans], "r+",
                 markersize=12, label="robot")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.title(f"{len(scans)} scan(s)")
        plt.show()

    return 0 if len(scans) == len(args.angles) else 1


if __name__ == "__main__":
    sys.exit(main())
