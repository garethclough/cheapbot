import sys
import signal
import math
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton,
    QTextEdit, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QRadioButton, QButtonGroup, QGridLayout
)
from PySide6.QtCore import QTimer, Qt, QPointF
from PySide6.QtGui import QPixmap, QPainter, QColor, QPen
from datetime import datetime

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
import json
# /robot/command uses AddDiagnostics only because it's a standard service with
# a single string request (load_namespace) and a success + message reply.
# The string carries a JSON object, e.g. {"cmd": "move_and_scan", "angle": 90}.
from diagnostic_msgs.srv import AddDiagnostics as Command
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# Every command to the robot program goes through this one service.
COMMAND_SERVICE = "/robot/command"
COORDINATES_TOPIC = "/robot/coordinates"
# Walls found by the robot: [start x, start y, end x, end y] per wall, metres.
WALLS_TOPIC = "/robot/walls"
# Room size and corners: [area, perimeter, corner1 x, corner1 y, ...].
ROOM_TOPIC = "/robot/room"
LOG_TOPIC = "/robot/log"
POSE_TOPIC = "/robot/pose"
# Where the robot's position/heading should come from:
# "sensors" = odometry and other robot sensors, "world" = Gazebo's true values.
POSE_SOURCE_SENSORS = "sensors"
POSE_SOURCE_WORLD = "world"

MAP_SIZE = 400      # bitmap width and height in pixels
MAP_MARGIN = 20     # empty border around the points, in pixels
WALL_COLOUR = QColor(230, 120, 20)   # orange
CORNER_COLOUR = QColor(150, 60, 190)  # purple

# Colours for the two pose sources, used by the heading dial and the table.
SENSORS_COLOUR = QColor(31, 119, 180)   # blue
WORLD_COLOUR = QColor(40, 160, 40)      # green


def side_lengths(corners):
    """Lengths of the room's sides, corner 1->2, 2->3, ..., last->1 (metres)."""
    n = len(corners)
    return [math.hypot(corners[(i + 1) % n][0] - corners[i][0],
                       corners[(i + 1) % n][1] - corners[i][1])
            for i in range(n)]


def describe_dimensions(corners):
    """'9.50 m × 9.50 m' for a four-cornered room (each pair of opposite
    sides averaged), otherwise the list of side lengths."""
    sides = side_lengths(corners)
    if len(sides) == 4:
        width = (sides[0] + sides[2]) / 2
        length = (sides[1] + sides[3]) / 2
        return f"{width:.2f} m × {length:.2f} m"
    return "sides " + ", ".join(f"{side:.2f}" for side in sides) + " m"


class HeadingDial(QWidget):
    """A small compass showing the sensor and world headings as arrows.

    Same directions as the map: 0 = pointing right (+x), angles increase
    anticlockwise, so 90 degrees points up (+y).
    """

    SIZE = 90

    def __init__(self):
        super().__init__()
        self.setFixedSize(self.SIZE, self.SIZE)
        self.sensor_yaw = math.nan   # radians; NaN = unknown, no arrow
        self.world_yaw = math.nan

    def set_yaws(self, sensor_yaw, world_yaw):
        self.sensor_yaw = sensor_yaw
        self.world_yaw = world_yaw
        self.update()   # ask Qt to repaint

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        centre = QPointF(self.SIZE / 2, self.SIZE / 2)
        radius = self.SIZE / 2 - 4

        # Dial: circle, and ticks every 90 degrees.
        painter.setPen(QPen(QColor(180, 180, 180), 1))
        painter.setBrush(QColor("white"))
        painter.drawEllipse(centre, radius, radius)
        for i in range(4):
            a = i * math.pi / 2
            d = QPointF(math.cos(a), -math.sin(a))
            painter.drawLine(centre + d * (radius - 6), centre + d * radius)

        # World first, so the sensor arrow is drawn on top when they overlap.
        self.draw_arrow(painter, centre, radius - 8, self.world_yaw, WORLD_COLOUR)
        self.draw_arrow(painter, centre, radius - 8, self.sensor_yaw, SENSORS_COLOUR)
        painter.end()

    @staticmethod
    def draw_arrow(painter, centre, length, yaw, colour):
        if math.isnan(yaw):
            return
        # Pixel y points down, so flip the sine to make positive angles go up.
        d = QPointF(math.cos(yaw), -math.sin(yaw))
        tip = centre + d * length
        painter.setPen(QPen(colour, 3, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(centre, tip)
        # Arrow head: two short lines angled back from the tip.
        for side in (+1, -1):
            a = yaw + math.pi + side * math.radians(25)
            painter.drawLine(tip, tip + QPointF(math.cos(a), -math.sin(a)) * 10)


class RobotGUI(QWidget):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.command_client = ros_node.create_client(Command, COMMAND_SERVICE)
        # Used to notice the robot program (re)appearing, so the pose source can be
        # sent to it again.
        self.node_was_available = False

        # Newest point cloud / walls received, or None if nothing new since
        # the last draw.
        self.latest_cloud = None
        self.latest_walls = None
        self.latest_room = None
        # What's currently drawn, so each can be redrawn with the others.
        self.map_points = []   # [(x, y), ...]
        self.map_walls = []    # [((x1, y1), (x2, y2)), ...]
        self.map_corners = []  # [(x, y), ...] in order round the room
        self.room_sub = ros_node.create_subscription(
            Float64MultiArray, ROOM_TOPIC, self.on_room, 10)
        self.walls_sub = ros_node.create_subscription(
            Float64MultiArray, WALLS_TOPIC, self.on_walls, 10)
        self.coordinates_sub = ros_node.create_subscription(
            PointCloud2, COORDINATES_TOPIC, self.on_coordinates, 10)
        self.messages_sub = ros_node.create_subscription(
            String, LOG_TOPIC, self.on_log_message, 10)
        # transient_local to match the node: get the last yaw it sent as soon
        # as we connect, even if the robot is sitting still.
        latest_only = QoSProfile(depth=1,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # /robot/pose is [sensor x, y, yaw, world x, y, yaw] (metres, radians);
        # the world values are NaN until the node has Gazebo's pose.
        self.pose_sub = ros_node.create_subscription(
            Float64MultiArray, POSE_TOPIC, self.on_pose, latest_only)

        self.setWindowTitle("Robot Controller")
        self.resize(500, 700)

        self.reset_button = QPushButton("Reset Robot")
        self.reset_button.clicked.connect(self.reset_robot)

        self.start_scan_button = QPushButton("Start Scan")
        self.start_scan_button.clicked.connect(self.start_scan)

        # Pose table: one row per source, columns x, y, yaw.
        #            x        y        yaw
        #   Sensors  0.00 m   0.00 m   0.0°
        #   World    0.00 m   0.00 m   0.0°
        self.pose_grid = QGridLayout()
        for col, heading in enumerate(["x", "y", "yaw"], start=1):
            self.pose_grid.addWidget(QLabel(heading), 0, col, Qt.AlignRight)
        # pose_labels["sensors"] = [x label, y label, yaw label], same for "world"
        self.pose_labels = {}
        for row, (key, name, colour) in enumerate(
                [("sensors", "Sensors", SENSORS_COLOUR), ("world", "World", WORLD_COLOUR)],
                start=1):
            # Row name in the same colour as its arrow on the heading dial.
            name_label = QLabel(name)
            name_label.setStyleSheet(f"color: {colour.name()}; font-weight: bold;")
            self.pose_grid.addWidget(name_label, row, 0)
            self.pose_labels[key] = []
            for col in range(1, 4):
                label = QLabel("--")
                label.setMinimumWidth(70)
                label.setAlignment(Qt.AlignRight)
                self.pose_grid.addWidget(label, row, col)
                self.pose_labels[key].append(label)
        self.pose_grid.setColumnStretch(4, 1)   # keep the table to the left

        self.heading_dial = HeadingDial()

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
        self.room_info = QLabel("Room: --")
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

        pose_row = QHBoxLayout()
        pose_row.addLayout(self.pose_grid)
        pose_row.addWidget(self.heading_dial)
        layout.addLayout(pose_row)

        angle_row = QHBoxLayout()
        angle_row.addWidget(QLabel("Angle:"))
        angle_row.addWidget(self.angle_input)
        angle_row.addWidget(self.scan_at_angle_button)
        angle_row.addStretch()
        layout.addLayout(angle_row)
        layout.addWidget(self.map_label, alignment=Qt.AlignHCenter)
        layout.addWidget(self.map_info)
        layout.addWidget(self.room_info)
        layout.addWidget(self.messages)

        self.setLayout(layout)

        # Let ROS 2 process replies from inside the Qt event loop: every 50 ms,
        # handle anything that has arrived, without blocking the window.
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(50)

        # Once a second, redraw the map if new coordinates have arrived.
        self.map_timer = QTimer(self)
        self.map_timer.timeout.connect(self.check_for_new_coordinates)
        self.map_timer.start(1000)

        # Once a second, check whether the robot program has (re)appeared; when it
        # does, send it the current pose source so the two agree.
        self.node_timer = QTimer(self)
        self.node_timer.timeout.connect(self.check_node_available)
        self.node_timer.start(1000)

        self.log("GUI started")

    # ------------------------------------------------------------ commands

    def send_command(self, command, label):
        """Send a command dict to the robot program as JSON; log the reply.

        command: e.g. {"cmd": "move_and_scan", "angle": 90}
        label:   name to show in the log, e.g. "Move & scan"
        """
        if not self.command_client.service_is_ready():
            self.log(f"{label} failed: {COMMAND_SERVICE} not available "
                     "(is robot running?)")
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
            self.log("robot connected")
            self.set_pose_source(self.current_pose_source())
        self.node_was_available = available

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.messages.append(f"{timestamp} - {message}")

    def on_log_message(self, msg):
        # Runs inside spin_once. Just append the message to the text box.
        self.log(msg.data)

    def on_pose(self, msg):
        # Runs inside spin_once (Qt thread), so it can update the labels directly.
        # msg.data = [sensor x, y, yaw, world x, y, yaw]: metres and radians.
        if len(msg.data) < 6:
            return   # old message format (yaw only), ignore

        def fmt_metres(m):
            return "--" if math.isnan(m) else f"{m:.2f} m"

        def fmt_degrees(radians):
            return "--" if math.isnan(radians) else f"{math.degrees(radians):.1f}°"

        for key, start in (("sensors", 0), ("world", 3)):
            x, y, yaw = msg.data[start:start + 3]
            labels = self.pose_labels[key]
            labels[0].setText(fmt_metres(x))
            labels[1].setText(fmt_metres(y))
            labels[2].setText(fmt_degrees(yaw))

        self.heading_dial.set_yaws(sensor_yaw=msg.data[2], world_yaw=msg.data[5])

    # ------------------------------------------------------------ map

    def spin_ros(self):
        # Skip if ROS has already shut down (e.g. while the app is closing).
        if rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0)

    def on_walls(self, msg):
        # Runs inside spin_once. Keep the newest; drawn with the points below.
        d = msg.data
        self.latest_walls = [((d[i], d[i + 1]), (d[i + 2], d[i + 3]))
                             for i in range(0, len(d) - 3, 4)]

    def on_room(self, msg):
        # Runs inside spin_once. [area, perimeter, x1, y1, x2, y2, ...].
        d = msg.data
        if len(d) < 2:
            return
        corners = [(d[i], d[i + 1]) for i in range(2, len(d) - 1, 2)]
        self.latest_room = (d[0], d[1], corners)

    def on_coordinates(self, msg):
        # Runs inside spin_once. Just keep the newest message; drawing happens
        # in check_for_new_coordinates, at most once a second.
        self.latest_cloud = msg

    def check_for_new_coordinates(self):
        if (self.latest_cloud is None and self.latest_walls is None
                and self.latest_room is None):
            return
        if self.latest_cloud is not None:
            xy = point_cloud2.read_points_numpy(self.latest_cloud, field_names=("x", "y"))
            self.map_points = [(float(x), float(y)) for x, y in xy]
            self.latest_cloud = None
        if self.latest_walls is not None:
            self.map_walls = self.latest_walls
            self.latest_walls = None
        if self.latest_room is not None:
            area, perimeter, self.map_corners = self.latest_room
            self.latest_room = None
            if len(self.map_corners) >= 3:
                self.room_info.setText(
                    f"Room: {describe_dimensions(self.map_corners)}, "
                    f"area {area:.2f} m², perimeter {perimeter:.2f} m, "
                    f"{len(self.map_corners)} corners")
            else:
                self.room_info.setText(
                    f"Room: not enough corners yet ({len(self.map_corners)} found)")
        self.draw_map(self.map_points, self.map_walls, self.map_corners)

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.map_info.setText(f"{len(self.map_points)} points, "
                              f"{len(self.map_walls)} walls, updated {timestamp}")

    def draw_map(self, points, walls=(), corners=()):
        pixmap = QPixmap(MAP_SIZE, MAP_SIZE)
        pixmap.fill(QColor("white"))
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Fit everything in view: all the points, plus the origin (0, 0) so
        # you can see where the robot started relative to the walls.
        wall_ends = [end for wall in walls for end in wall]
        xs = [x for x, _ in points] + [x for x, _ in wall_ends] + [x for x, _ in corners] + [0.0]
        ys = [y for _, y in points] + [y for _, y in wall_ends] + [y for _, y in corners] + [0.0]
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

        # Walls: thick orange lines over the points.
        painter.setPen(QPen(WALL_COLOUR, 3, Qt.SolidLine, Qt.RoundCap))
        for (x1, y1), (x2, y2) in walls:
            painter.drawLine(to_pixel(x1, y1), to_pixel(x2, y2))

        # Corners: purple dots, numbered in the order round the room.
        painter.setPen(QPen(CORNER_COLOUR, 1))
        painter.setBrush(CORNER_COLOUR)
        for number, (x, y) in enumerate(corners, start=1):
            centre = to_pixel(x, y)
            painter.drawEllipse(centre, 5, 5)
            painter.drawText(centre + QPointF(7, -7), str(number))
        painter.setBrush(Qt.NoBrush)

        # Side lengths: written at the middle of each side, just inside the room.
        if len(corners) >= 3:
            # Which way round the corners go decides which side of each wall is
            # inside: anticlockwise (positive area) = inside is on the left.
            signed_area = sum(corners[i][0] * corners[(i + 1) % len(corners)][1]
                              - corners[(i + 1) % len(corners)][0] * corners[i][1]
                              for i in range(len(corners)))
            inside = 1.0 if signed_area > 0 else -1.0
            metrics = painter.fontMetrics()
            painter.setPen(QPen(CORNER_COLOUR, 1))
            for i, side in enumerate(side_lengths(corners)):
                (x1, y1), (x2, y2) = corners[i], corners[(i + 1) % len(corners)]
                if side < 1e-9:
                    continue
                # Unit vector at right angles to the wall, pointing into the room
                # (world coordinates; y is flipped below for pixels).
                nx = -(y2 - y1) / side * inside
                ny = (x2 - x1) / side * inside
                text = f"{side:.2f} m"
                w, h = metrics.horizontalAdvance(text), metrics.ascent()
                # Far enough in that the whole text clears the wall: half the
                # text's width/height in the direction we move, plus a gap.
                gap = 6 + abs(nx) * w / 2 + abs(ny) * h / 2
                middle = to_pixel((x1 + x2) / 2, (y1 + y2) / 2)
                centre = middle + QPointF(nx, -ny) * gap
                painter.drawText(centre + QPointF(-w / 2, h / 2 - 1), text)

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
        self.latest_walls = None
        self.latest_room = None
        self.map_points = []
        self.map_walls = []
        self.map_corners = []
        self.room_info.setText("Room: --")
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


# Don't let rclpy handle Ctrl+C: it would shut ROS down while the window
# keeps running. Instead Ctrl+C quits the Qt app, and ROS is shut down
# cleanly below once the event loop has stopped.
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
ros_node = rclpy.create_node("robot_gui")

app = QApplication(sys.argv)
# Python only runs this handler between bits of Python code; the 50 ms ROS
# timer makes sure that happens often, so Ctrl+C takes effect quickly.
signal.signal(signal.SIGINT, lambda *args: app.quit())
window = RobotGUI(ros_node)
window.show()

exit_code = app.exec()

ros_node.destroy_node()
rclpy.try_shutdown()
sys.exit(exit_code)
