import sys
import math
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QRadioButton, QButtonGroup
)
from PySide6.QtCore import QTimer, Qt, QPointF
from PySide6.QtGui import QPixmap, QPainter, QColor, QPen
from datetime import datetime

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
import json
# /robot/command uses AddDiagnostics only because it's a standard service with
# a single string request (load_namespace) and a success + message reply.
# The string carries a JSON object, e.g. {"cmd": "move_and_scan", "angle": 90}.
from diagnostic_msgs.srv import AddDiagnostics as Command
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# Every command to lidar_node goes through this one service.
COMMAND_SERVICE = "/robot/command"
COORDINATES_TOPIC = "/robot/coordinates"
LOG_TOPIC = "/robot/log"
YAW_TOPIC = "/robot/yaw"
# Where the robot's position/heading should come from:
# "sensors" = odometry and other robot sensors, "world" = Gazebo's true values.
POSE_SOURCE_SENSORS = "sensors"
POSE_SOURCE_WORLD = "world"

MAP_SIZE = 400      # bitmap width and height in pixels
MAP_MARGIN = 20     # empty border around the points, in pixels


class RobotGUI(QWidget):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.command_client = ros_node.create_client(Command, COMMAND_SERVICE)
        # Used to notice lidar_node (re)appearing, so the pose source can be
        # sent to it again.
        self.node_was_available = False

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
        # /robot/yaw is [sensor yaw, world yaw] in radians; world is NaN until
        # the node has Gazebo's pose.
        self.yaw_sub = ros_node.create_subscription(
            Float64MultiArray, YAW_TOPIC, self.on_yaw, latest_only)

        self.setWindowTitle("Robot Controller")
        self.resize(500, 700)

        self.reset_button = QPushButton("Reset Robot")
        self.reset_button.clicked.connect(self.reset_robot)

        self.start_scan_button = QPushButton("Start Scan")
        self.start_scan_button.clicked.connect(self.start_scan)

        self.yaw_label = QLabel("Yaw: --")
        self.world_yaw_label = QLabel("World yaw: --")

        # Pose source: one of two radio buttons, sensors selected to start.
        self.sensors_radio = QRadioButton("Sensors")
        self.sim_radio = QRadioButton("World sim")
        self.sim_radio.setProperty("value", "world")

        self.sensors_radio.setChecked(True)
        self.pose_source_group = QButtonGroup(self)   # makes them exclusive
        self.pose_source_group.addButton(self.sensors_radio)
        self.pose_source_group.addButton(self.sim_radio)
        # toggled fires for both buttons on a change; only act on the new one.
        self.sensors_radio.toggled.connect(
            lambda checked: checked and self.set_pose_source(POSE_SOURCE_SENSORS))
        self.sim_radio.toggled.connect(
            lambda checked: checked and self.set_pose_source(POSE_SOURCE_WORLD))

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

        source_row = QHBoxLayout()
        source_row.addWidget(QLabel("Use:"))
        source_row.addWidget(self.sensors_radio)
        source_row.addWidget(self.sim_radio)
        source_row.addStretch()
        layout.addLayout(source_row)

        yaw_row = QHBoxLayout()
        yaw_row.addWidget(self.yaw_label)
        yaw_row.addSpacing(30)
        yaw_row.addWidget(self.world_yaw_label)
        yaw_row.addStretch()
        layout.addLayout(yaw_row)

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

        # Once a second, check whether lidar_node has (re)appeared; when it
        # does, send it the current pose source so the two agree.
        self.node_timer = QTimer(self)
        self.node_timer.timeout.connect(self.check_node_available)
        self.node_timer.start(1000)

        self.log("GUI started")

    # ------------------------------------------------------------ commands

    def send_command(self, command, label):
        """Send a command dict to lidar_node as JSON; log the reply.

        command: e.g. {"cmd": "move_and_scan", "angle": 90}
        label:   name to show in the log, e.g. "Move & scan"
        """
        if not self.command_client.service_is_ready():
            self.log(f"{label} failed: {COMMAND_SERVICE} not available "
                     "(is lidar_node running?)")
            return

        request = Command.Request()
        request.load_namespace = json.dumps(command)
        self.log(f"{label} requested")
        # Asynchronous, so the window stays responsive while waiting.
        future = self.command_client.call_async(request)
        future.add_done_callback(lambda f: self.on_command_done(f, label))

    def on_command_done(self, future, label):
        # Runs inside spin_once, i.e. on the Qt thread, so it can update the UI.
        try:
            response = future.result()
        except Exception as e:
            self.log(f"{label} failed: {e}")
            return
        status = "OK" if response.success else "REFUSED"
        self.log(f"{label} {status}: {response.message}")

    def current_pose_source(self):
        return POSE_SOURCE_WORLD if self.sim_radio.isChecked() else POSE_SOURCE_SENSORS

    def set_pose_source(self, source):
        self.send_command({"cmd": "pose_source", "value": source},
                          f"Use {'sensors' if source == POSE_SOURCE_SENSORS else 'world'}")

    def check_node_available(self):
        available = self.command_client.service_is_ready()
        if available and not self.node_was_available:
            self.log("lidar_node connected")
            self.set_pose_source(self.current_pose_source())
        self.node_was_available = available

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.messages.append(f"{timestamp} - {message}")

    def on_log_message(self, msg):
        # Runs inside spin_once. Just append the message to the text box.
        self.log(msg.data)

    def on_yaw(self, msg):
        # Runs inside spin_once (Qt thread), so it can update the labels directly.
        # msg.data = [sensor yaw, world yaw] in radians; show degrees.
        def fmt(radians):
            return "--" if math.isnan(radians) else f"{math.degrees(radians):.1f}°"

        if len(msg.data) >= 1:
            self.yaw_label.setText(f"Yaw: {fmt(msg.data[0])}")
        if len(msg.data) >= 2:
            self.world_yaw_label.setText(f"World yaw: {fmt(msg.data[1])}")

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
        # The reply only says whether it was accepted; progress arrives on
        # /robot/log.
        degrees = self.angle_input.value()
        self.send_command({"cmd": "move_and_scan", "angle": degrees},
                          f"Move & scan at {degrees:.1f}°")

    def start_scan(self):
        self.send_command({"cmd": "start_scan"}, "Start scan")

    def reset_robot(self):
        self.clear_map()
        self.send_command({"cmd": "reset"}, "Reset")


rclpy.init()
ros_node = rclpy.create_node("robot_gui")

app = QApplication(sys.argv)
window = RobotGUI(ros_node)
window.show()

exit_code = app.exec()

ros_node.destroy_node()
rclpy.shutdown()
sys.exit(exit_code)
