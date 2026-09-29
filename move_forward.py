#!/usr/bin/env python3
"""
move_forward.py — drives the robot forward for a fixed duration, then stops.

Run (after sourcing ROS 2 and while the simulation is running):
    python3 move_forward.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


FORWARD_SPEED = 0.2   # meters per second
DRIVE_DURATION = 3.0  # seconds
PUBLISH_RATE_HZ = 10  # how often to re-send the command


class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.elapsed = 0.0
        self.period = 1.0 / PUBLISH_RATE_HZ
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(
            f'Driving forward at {FORWARD_SPEED} m/s for {DRIVE_DURATION}s...'
        )

    def tick(self):
        msg = Twist()

        if self.elapsed < DRIVE_DURATION:
            # Still driving: keep sending the forward command
            msg.linear.x = FORWARD_SPEED
            self.publisher.publish(msg)
            self.elapsed += self.period
        else:
            # Time's up: send one stop command, then shut down
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.get_logger().info('Done. Stopped.')
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()