import sys
import math
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox
)
from PySide6.QtCore import QTimer, Qt, QPointF
from PySide6.QtGui import QPixmap, QPainter, QColor, QPen
from datetime import datetime

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import Trigger
# Used for /robot/move_and_scan only because it's a standard service with a
# single string request (load_namespace) and a success + message reply.
from diagnostic_msgs.srv import AddDiagnostics as MoveAndScan
from std_msgs.msg import String, Float64
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

RESET_SERVICE = "/robot/reset"
START_SCAN_SERVICE = "/robot/start_scan"
COORDINATES_TOPIC = "/robot/coordinates"
LOG_TOPIC = "/robot/log"
YAW_TOPIC = "/robot/yaw"
# Service: turn to a heading (degrees, sent as text) and take a scan.
MOVE_AND_SCAN_SERVICE = "/robot/move_and_scan"

MAP_SIZE = 400      # bitmap width and height in pixels
MAP_MARGIN = 20     # empty border around the points, in pixels


class RobotGUI(QWidget):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.reset_client = ros_node.create_client(Trigger, RESET_SERVICE)
        self.start_scan_client = ros_node.create_client(Trigger, START_SCAN_SERVICE)

        # Newest point cloud received, or None if nothing new since last draw.
        self.latest_cloud = None
        self.coordinates_sub = ros_node.create_subscription(
            PointCloud2, COORDINATES_TOPIC, self.on_coordinates, 10)
        self.messages_sub = ros_node.create_subscription(
            String, LOG_TOPIC, self.on_log_message, 10)
        # transient_local to match the node: get the last yaw it sent as soon
        # as we connect, even if the robot is sitting still.
        latest_only = QoSProfile(depth=1,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.yaw_sub = ros_node.create_subscription(
            Float64, YAW_TOPIC, self.on_yaw, latest_only)
        self.move_and_scan_client = ros_node.create_client(
            MoveAndScan, MOVE_AND_SCAN_SERVICE)

        self.setWindowTitle("Robot Controller")
        self.resize(500, 700)

        self.reset_button = QPushButton("Reset Robot")
        self.reset_button.clicked.connect(self.reset_robot)

        self.start_scan_button = QPushButton("Start Scan")
        self.start_scan_button.clicked.connect(self.start_scan)

        self.yaw_label = QLabel("Yaw: --")

        # Angle to turn to (degrees, same 0 as the Yaw display) + go button.
        self.angle_input = QDoubleSpinBox()
        self.angle_input.setRange(-180.0, 180.0)
        self.angle_input.setDecimals(1)
        self.angle_input.setSingleStep(15.0)
        self.angle_input.setSuffix("°")
        self.scan_at_angle_button = QPushButton("Move && Scan")
        self.scan_at_angle_button.clicked.connect(self.scan_at_angle)

        # The map bitmap: a pixmap shown in a label.
        self.map_label = QLabel()
        self.map_label.setFixedSize(MAP_SIZE, MAP_SIZE)
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_info = QLabel(f"Waiting for {COORDINATES_TOPIC}...")
        self.draw_map([])

        self.messages = QTextEdit()
        self.messages.setReadOnly(True)

        layout = QVBoxLayout()
        button_row = QHBoxLayout()
        button_row.addWidget(self.reset_button)
        button_row.addWidget(self.start_scan_button)
        layout.addLayout(button_row)
        layout.addWidget(self.yaw_label)

        angle_row = QHBoxLayout()
        angle_row.addWidget(QLabel("Angle:"))
        angle_row.addWidget(self.angle_input)
        angle_row.addWidget(self.scan_at_angle_button)
        angle_row.addStretch()
        layout.addLayout(angle_row)
        layout.addWidget(self.map_label, alignment=Qt.AlignHCenter)
        layout.addWidget(self.map_info)
        layout.addWidget(self.messages)

        self.setLayout(layout)

        # Let ROS 2 process replies from inside the Qt event loop: every 50 ms,
        # handle anything that has arrived, without blocking the window.
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)

        # Once a second, redraw the map if new coordinates have arrived.
        self.map_timer = QTimer(self)
        self.map_timer.timeout.connect(self.check_for_new_coordinates)
        self.map_timer.start(1000)

        self.log("GUI started")

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.messages.append(f"{timestamp} - {message}")

    def on_log_message(self, msg):
        # Runs inside spin_once. Just append the message to the text box.
        self.log(msg.data)

    def on_yaw(self, msg):
        # Runs inside spin_once (Qt thread), so it can update the label directly.
        # The node sends radians; show degrees.
        self.yaw_label.setText(f"Yaw: {math.degrees(msg.data):.1f}°")

    # ------------------------------------------------------------ map

    def on_coordinates(self, msg):
        # Runs inside spin_once. Just keep the newest message; drawing happens
        # in check_for_new_coordinates, at most once a second.
        self.latest_cloud = msg

    def check_for_new_coordinates(self):
        if self.latest_cloud is None:
            return
        msg = self.latest_cloud
        self.latest_cloud = None

        xy = point_cloud2.read_points_numpy(msg, field_names=("x", "y"))
        points = [(float(x), float(y)) for x, y in xy]
        self.draw_map(points)

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.map_info.setText(f"{len(points)} points, updated {timestamp}")

    def draw_map(self, points):
        pixmap = QPixmap(MAP_SIZE, MAP_SIZE)
        pixmap.fill(QColor("white"))
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Fit everything in view: all the points, plus the origin (0, 0) so
        # you can see where the robot started relative to the walls.
        xs = [x for x, _ in points] + [0.0]
        ys = [y for _, y in points] + [0.0]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span = max(max_x - min_x, max_y - min_y, 1.0)   # metres; never 0
        scale = (MAP_SIZE - 2 * MAP_MARGIN) / span      # pixels per metre
        centre_x = (min_x + max_x) / 2
        centre_y = (min_y + max_y) / 2

        def to_pixel(x, y):
            # World y points up, but pixel y points down, so flip it.
            px = MAP_SIZE / 2 + (x - centre_x) * scale
            py = MAP_SIZE / 2 - (y - centre_y) * scale
            return QPointF(px, py)

        # Points: small black dots.
        painter.setPen(QPen(QColor("black"), 2))
        for x, y in points:
            painter.drawPoint(to_pixel(x, y))

        # Origin: red cross.
        origin = to_pixel(0.0, 0.0)
        painter.setPen(QPen(QColor("red"), 2))
        painter.drawLine(origin + QPointF(-6, 0), origin + QPointF(6, 0))
        painter.drawLine(origin + QPointF(0, -6), origin + QPointF(0, 6))

        # Scale bar: 1 metre, bottom left.
        painter.setPen(QPen(QColor("gray"), 2))
        bar_start = QPointF(MAP_MARGIN, MAP_SIZE - 8)
        painter.drawLine(bar_start, bar_start + QPointF(scale, 0))
        painter.drawText(bar_start + QPointF(0, -4), "1 m")

        painter.end()
        self.map_label.setPixmap(pixmap)

    # ------------------------------------------------------------ reset

    def clear_map(self):
        # Drop any cloud not drawn yet, so the 1 s redraw can't bring old
        # points back, then draw an empty map.
        self.latest_cloud = None
        self.draw_map([])
        self.map_info.setText(f"Map cleared, waiting for {COORDINATES_TOPIC}...")

    def scan_at_angle(self):
        if not self.move_and_scan_client.service_is_ready():
            self.log(f"Move & scan failed: {MOVE_AND_SCAN_SERVICE} not available "
                     "(is lidar_node running?)")
            return

        degrees = self.angle_input.value()
        request = MoveAndScan.Request()
        request.load_namespace = str(degrees)   # the angle, in degrees, as text
        self.log(f"Move & scan at {degrees:.1f}° requested")
        future = self.move_and_scan_client.call_async(request)
        future.add_done_callback(self.on_move_and_scan_done)

    def on_move_and_scan_done(self, future):
        # Reply only says whether it was accepted; progress arrives on /robot/log.
        try:
            response = future.result()
        except Exception as e:
            self.log(f"Move & scan failed: {e}")
            return
        status = "accepted" if response.success else "REFUSED"
        self.log(f"Move & scan {status}: {response.message}")

    def start_scan(self):
        if not self.start_scan_client.service_is_ready():
            self.log(f"Start scan failed: {START_SCAN_SERVICE} not available "
                     "(is lidar_node running?)")
            return

        self.log("Start scan requested")
        future = self.start_scan_client.call_async(Trigger.Request())
        future.add_done_callback(self.on_start_scan_done)

    def on_start_scan_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.log(f"Start scan failed: {e}")
            return
        status = "OK" if response.success else "REFUSED"
        self.log(f"Start scan {status}: {response.message}")

    def reset_robot(self):
        self.clear_map()

        if not self.reset_client.service_is_ready():
            self.log(f"Reset failed: {RESET_SERVICE} not available "
                     "(is lidar_node running?)")
            return

        self.log("Reset requested")
        # Asynchronous, so the window stays responsive while waiting.
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(self.on_reset_done)

    def on_reset_done(self, future):
        # Runs inside spin_once, i.e. on the Qt thread, so it can update the UI.
        try:
            response = future.result()
        except Exception as e:
            self.log(f"Reset failed: {e}")
            return
        status = "OK" if response.success else "FAILED"
        self.log(f"Reset {status}: {response.message}")


rclpy.init()
ros_node = rclpy.create_node("robot_gui")

app = QApplication(sys.argv)
window = RobotGUI(ros_node)
window.show()

exit_code = app.exec()

ros_node.destroy_node()
rclpy.shutdown()
sys.exit(exit_code)
