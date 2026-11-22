#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <csignal>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/srv/add_diagnostics.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "min_max_xy.hh"
#include "lidar_message.hh"

void loop();
void handleInterrupt(int signal);
void setUpGuiCommunication();
void resetMode();
std::string radiansToDegreesString(double radians);

struct Coordinate {
    double x;
    double y;
};

enum class Mode
{
    STOP = 0,
    SCAN_ROOM = 1,
    MOVE_AND_SCAN = 2,
    MOVE_FORWARD = 3
};

// Steps of a move-and-scan request from the GUI.
enum class MoveAndScanStep
{
    TURN = 0,
    WAIT_FOR_TURN = 1,
    WAIT_FOR_SCAN = 2
};

enum class RoomScanMode
{
    INIT = 0,
    SCAN_ROOM = 1,
    WAIT_FOR_ROOM_SCAN = 2,
    TURN = 3,
    WAIT_FOR_TURN = 4,
};

// Must come after RoomScanMode, which it uses.
struct RoomScan {
  RoomScanMode mode = RoomScanMode::INIT;
  const int numScans = 4;
  int currentScan = 0;

  void reset() {
    mode = RoomScanMode::INIT;
    currentScan = 0;
  }

  float getTurnAngle() const {
    double step = 2 * M_PI / numScans;
    double angle = currentScan * step;
    return angle;
  }
};

std::string topic_pub = "/cmd_vel";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);
std::optional<float> maxX;
MinMaxXY limits;
Mode mode = Mode::STOP;
std::mutex dataMutex;
bool waitingForLidarData = false;
LidarMessage latestLidarData;
std::atomic<bool> running{true};
std::vector<Coordinate> coordinates;
RoomScan roomScan;
std::optional<double> lastPublishedYaw;

float worldX = 0.0;
float worldY = 0.0;

// Robot heading in radians (-pi..pi, 0 = facing +x), from /odom.
// Written by odomCb, read by loop(): both under dataMutex.
double currentYaw = 0.0;
bool haveOdom = false;
// Odometry's own heading, before the offset is applied, and the offset
// itself: the raw heading at the last reset. currentYaw = rawYaw - yawOffset,
// so whichever way the robot faced at reset counts as 0.
double rawYaw = 0.0;
double yawOffset = 0.0;
// Heading to stop at during WAIT_FOR_TURN.
double turnTargetYaw = 0.0;
// Current step while mode == MOVE_AND_SCAN.
MoveAndScanStep moveAndScanStep = MoveAndScanStep::TURN;

// Wrap any angle into -pi..pi, so e.g. 190 deg becomes -170 deg.
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Send a drive command: linear = forward m/s, angular = turn rad/s (+ = left).
void setSpeed(double linear, double angular)
{
  // Don't send a move command once shutdown has started, or it could
  // arrive after the stop command and leave the robot driving.
  if (!running && (linear != 0.0 || angular != 0.0))
    return;
  gz::msgs::Twist cmd;
  cmd.mutable_linear()->set_x(linear);
  cmd.mutable_angular()->set_z(angular);
  pub.Publish(cmd);
}


// ROS 2 side, for talking to robot_gui.py. Separate from the Gazebo `node`
// above. These must live as long as the program: if the service object is
// destroyed, the service disappears.
rclcpp::Node::SharedPtr rosNode;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;
// /robot/move_and_scan. Uses AddDiagnostics only because it's a standard
// service type with a single string request and a success + message reply;
// its `load_namespace` field carries the angle in degrees, as text.
rclcpp::Service<diagnostic_msgs::srv::AddDiagnostics>::SharedPtr moveAndScanService;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logPublisher;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr coordinatesPublisher;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yawPublisher;
// Only publish the yaw when it has moved at least this much (0.5 degrees),
// so a robot sitting still doesn't send the same value 50 times a second.
constexpr double kYawPublishThreshold = 0.5 * M_PI / 180.0;

// Send a line of text on /robot/log. The publisher takes a std_msgs String
// message, not a plain std::string, so wrap the text in one first.
void publishLog(const std::string &text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  logPublisher->publish(msg);
  std::cout << text << std::endl;
}

void odomCb(const gz::msgs::Odometry &msg)
{
  // Heading from the orientation quaternion (rotation about z).
  const auto &q = msg.pose().orientation();

  double odomYaw = std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())
);

  std::lock_guard<std::mutex> lock(dataMutex);
  currentYaw = odomYaw;
  haveOdom = true;

  // Publish when it has changed enough (or for the very first reading).
  if (yawPublisher &&
      (!lastPublishedYaw.has_value() ||
       std::fabs(odomYaw - lastPublishedYaw.value()) >= kYawPublishThreshold))
  {
    std_msgs::msg::Float64 yawMsg;
    yawMsg.data = odomYaw;
    yawPublisher->publish(yawMsg);
    lastPublishedYaw = odomYaw;
  }
  return;
  /*
    const auto &pose = msg.pose();

    if (limits.update(
        pose.position().x(),
        pose.position().y()))
    {
        std::cout << "Min X: " << limits.getMinX().value()
                  << " Max X: " << limits.getMaxX().value()
                  << " Min Y: " << limits.getMinY().value()
                  << " Max Y: " << limits.getMaxY().value()
                  << std::endl;
    }
                  */
}

void imuCb(const gz::msgs::IMU &_msg)
{
  return;
    const auto &orientation = _msg.orientation();
/*
    std::cout
        << "Orientation: "
        << "x=" << orientation.x()
        << " y=" << orientation.y()
        << " z=" << orientation.z()
        << " w=" << orientation.w()
        << std::endl;

    const auto &angularVelocity = _msg.angular_velocity();

    std::cout
        << "Angular velocity: "
        << "x=" << angularVelocity.x()
        << " y=" << angularVelocity.y()
        << " z=" << angularVelocity.z()
        << std::endl;

    const auto &linearAcceleration = _msg.linear_acceleration();

    std::cout
        << "Acceleration: "
        << "x=" << linearAcceleration.x()
        << " y=" << linearAcceleration.y()
        << " z=" << linearAcceleration.z()
        << std::endl;
*/
}

void lidarCb(const gz::msgs::LaserScan &_msg)
{
  std::lock_guard<std::mutex> lock(dataMutex);
  if (!waitingForLidarData)
    return;

  std::cout << "Got lidar data...\n";
  latestLidarData.setMessage(_msg);

  waitingForLidarData = false;
  return;

  gz::msgs::Twist data;

  bool allMore = true;
  int numRanges = _msg.ranges_size();
  int steps = numRanges / 2;

  double angleMin = _msg.angle_min();
  double angleMax = _msg.angle_max();
  double angleStep = _msg.angle_step();

  double angleMinDeg = angleMin * 180.0 / M_PI;
  double angleMaxDeg = angleMax * 180.0 / M_PI;

  for (int i = 0; i < steps; i++)
  {
    float frontAngle = _msg.angle_min() + i * _msg.angle_step();
    float degrees = frontAngle * 180.0 / M_PI;
    float backAngle = frontAngle + M_PI;

    double frontRange = _msg.ranges(i);
    double backRange = _msg.ranges(i + steps);

    double frontX = frontRange * cos(frontAngle);
    double backX = backRange * cos(backAngle);
    double frontY = frontRange * sin(frontAngle);
    double backY = backRange * sin(backAngle);

    std::cout << "Angle: " << degrees
              << " Front Range: " << frontRange
              << " Back Range: " << backRange
              << " Front X: " << frontX
              << " Back X: " << backX
              << " Front Y: " << frontY
              << " Back Y: " << backY
              << " Width: " << (frontX + backX)
              << " Height: " << (frontY + backY)
              << std::endl;

    if (frontRange < 1.0 || backRange < 1.0)
    {
      allMore = false;
      break;
    }
  }

  if (mode == Mode::MOVE_FORWARD) {

    if (allMore) //if all bigger than one
    {
      data.mutable_linear()->set_x(0.5);
      data.mutable_angular()->set_z(0.0);
    }
    else
    {
      data.mutable_linear()->set_x(0.0);
      data.mutable_angular()->set_z(0.5);
    }
    // Don't send a move command once shutdown has started, or it could
    // arrive after the stop command and leave the robot driving.
    if (!running)
      return;
    pub.Publish(data);
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Start ROS 2 without its own Ctrl+C handler, so ours below stays in charge.
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  rosNode = rclcpp::Node::make_shared("lidar_node");
  setUpGuiCommunication();
  // Handle ROS 2 requests (e.g. /robot/reset) on a background thread.
  // spin() returns when rclcpp::shutdown() is called during exit.
  std::thread rosThread([] { rclcpp::spin(rosNode); });

  std::signal(SIGINT, handleInterrupt);   // Ctrl+C
  std::signal(SIGTERM, handleInterrupt);  // kill / pkill
  std::string topic_lidar = "/lidar";   // subscribe to this topic
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_lidar, lidarCb))
  {
    std::cerr << "Error subscribing to topic [" << topic_lidar << "]" << std::endl;
    return -1;
  }

    std::string topic_imu = "/imu";

    if (!node.Subscribe(topic_imu, imuCb))
    {
        std::cerr << "Error subscribing to topic ["
                << topic_imu << "]" << std::endl;
        return -1;
    }

  if (!node.Subscribe("/odom", odomCb))
  {
      std::cerr << "Failed to subscribe to /odom\n";
      return 1;
  }

  std::cout << "Listening for odometry...\n";

  // Zzzzzz.
//  gz::transport::waitForShutdown();
  {
    // odomCb may already be running, so lock like the reset service does.
    std::lock_guard<std::mutex> lock(dataMutex);
    resetMode();
  }
  while (running) {
      loop();
      // 20 times a second, so a turn is checked often enough to stop on time.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // ---- Shutdown: leave Gazebo in a clean state ----
  std::cout << "Shutting down..." << std::endl;

  // 1. Stop listening, so no more callbacks run.
  node.Unsubscribe(topic_lidar);
  node.Unsubscribe(topic_imu);
  node.Unsubscribe("/odom");

  // 2. Stop the robot. The diff-drive plugin keeps using the last command
  //    it received, so without this the robot carries on after we exit.
  //    A Twist with nothing set is all zeros = stop.
  gz::msgs::Twist stop;
  pub.Publish(stop);

  // 3. Publishing is asynchronous: give gz-transport a moment to actually
  //    send the message before the process (and its sockets) go away.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Robot stopped." << std::endl;

  // 4. Stop ROS 2 and wait for its thread to finish.
  rclcpp::shutdown();
  rosThread.join();
  // Release the ROS objects now, while ROS can still clean them up. Left to
  // themselves, these globals are destroyed after ROS has been torn down at
  // program exit, which crashes.
  resetService.reset();
  moveAndScanService.reset();
  logPublisher.reset();
  coordinatesPublisher.reset();
  yawPublisher.reset();
  rosNode.reset();
  return 0;
}

void resetMode() {
  mode = Mode::STOP;
  roomScan.reset();
  waitingForLidarData = false;
  // The direction the robot faces now becomes the new 0.
  yawOffset = rawYaw;
  currentYaw = 0.0;
  coordinates.clear();
  setSpeed(0.0, 0.0);   // in case a reset arrives mid-turn
  if (yawPublisher) {
    std_msgs::msg::Float64 yawMsg;
    yawMsg.data = currentYaw;
    yawPublisher->publish(yawMsg);
  }
}

void setUpGuiCommunication()
{
  resetService = rosNode->create_service<std_srvs::srv::Trigger>(
      "/robot/reset",
      [&](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
          // Reset your robot variables
           std::cout << "Resetting robot..." << std::endl;
           // This runs on the ROS thread, so lock like loop() does.
           std::lock_guard<std::mutex> lock(dataMutex);
           resetMode();
          response->success = true;
          response->message = "Robot reset successfully";
      }
  );
  // Turn to an angle and take one scan. Replies straight away (accepted or
  // not); the work happens in loop(), with progress on /robot/log.
  moveAndScanService = rosNode->create_service<diagnostic_msgs::srv::AddDiagnostics>(
      "/robot/move_and_scan",
      [](
          const std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Request> request,
          std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Response> response)
      {
          const std::string &angleText = request->load_namespace;

          // Text -> number. std::stod throws if there's no number at the start;
          // `used` catches trailing junk like "90abc".
          double degrees;
          try {
            size_t used = 0;
            degrees = std::stod(angleText, &used);
            if (used != angleText.size() || !std::isfinite(degrees))
              throw std::invalid_argument("bad angle");
          } catch (const std::exception &) {
            response->success = false;
            response->message = "Not a valid angle: '" + angleText + "'";
            return;
          }

          // This runs on the ROS thread, so lock like loop() does.
          std::lock_guard<std::mutex> lock(dataMutex);
          if (mode == Mode::SCAN_ROOM) {
            response->success = false;
            response->message = "Busy: room scan in progress";
            return;
          }

          turnTargetYaw = normalizeAngle(degrees * M_PI / 180.0);
          moveAndScanStep = MoveAndScanStep::TURN;
          mode = Mode::MOVE_AND_SCAN;
          response->success = true;
          response->message = "Moving to " + radiansToDegreesString(turnTargetYaw)
                              + " to scan";
      }
  );

  logPublisher =
    rosNode->create_publisher<std_msgs::msg::String>("/robot/log", 10);
  coordinatesPublisher =
    rosNode->create_publisher<sensor_msgs::msg::PointCloud2>("/robot/coordinates", 10);

  // Robot heading (radians, -pi..pi), published by odomCb when it changes.
  // transient_local: keep the last value and hand it to subscribers that
  // join later, so a GUI started while the robot is still gets it at once.
  yawPublisher = rosNode->create_publisher<std_msgs::msg::Float64>(
      "/robot/yaw", rclcpp::QoS(1).transient_local());

}

std::string radiansToDegreesString(double radians) {
  return std::to_string(radians * 180.0 / M_PI) + "°";
}

void rotateCoordinates(
    std::vector<Coordinate>& coordinates,
    double angle,
    double originX = 0.0,
    double originY = 0.0)
{
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    for (auto& point : coordinates)
    {
        // Translate to origin
        double x = point.x - originX;
        double y = point.y - originY;

        // Rotate
        double newX = x * cosA - y * sinA;
        double newY = x * sinA + y * cosA;

        // Translate back
        point.x = newX + originX;
        point.y = newY + originY;
    }
}

void processLidarData(bool clearCoordinates = true) {
  const gz::msgs::LaserScan &msg = latestLidarData.getMessage();
  if (clearCoordinates) {
    coordinates.clear();
  }
  std_msgs::msg::String logMsg;
  logMsg.data = "Current yaw in lidar calc: " + radiansToDegreesString(currentYaw);
  logPublisher->publish(logMsg);
  double angle = msg.angle_min();
  double angleIncrement = msg.angle_step();
  logMsg.data = "Lidar angle min: " + radiansToDegreesString(msg.angle_min()) +
                ", max: " + radiansToDegreesString(msg.angle_max()) +
                ", increment: " + radiansToDegreesString(msg.angle_step());
  logPublisher->publish(logMsg);
  std::vector<Coordinate> newCoordinates;
    for (int i = 0; i < msg.ranges_size(); i++)
    {
        double distance = msg.ranges(i);

        // Ignore invalid readings
        if (!std::isfinite(distance) ||
            distance < msg.range_min() ||
            distance > msg.range_max())
        {
            angle += angleIncrement;
            continue;
        }

        Coordinate point;

        point.x = distance * std::cos(angle);
        point.y = distance * std::sin(angle);

        newCoordinates.push_back(point);

        angle += angleIncrement;
    }

    rotateCoordinates(newCoordinates, (M_PI / 2.0) + currentYaw);
  coordinates.insert(
      coordinates.end(),
      newCoordinates.begin(),
      newCoordinates.end()
  );
}

// Publish every point in `coordinates` on /robot/coordinates as a point cloud
// (x, y, z with z = 0), in the "world" frame.
void publishCoordinates() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = rosNode->now();
  cloud.header.frame_id = "world";

  // Describe the layout (three float fields: x, y, z) and size the buffer.
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(coordinates.size());

  // Iterators step through the x, y and z fields of each point in turn.
  sensor_msgs::PointCloud2Iterator<float> iterX(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(cloud, "z");
  for (const Coordinate &point : coordinates) {
    *iterX = point.x;
    *iterY = point.y;
    *iterZ = 0.0f;
    ++iterX; ++iterY; ++iterZ;
  }

  coordinatesPublisher->publish(cloud);
  std::cout << "Published " << coordinates.size() << " coordinates" << std::endl;
}

void scanRoomLoop() {
  switch (roomScan.mode)
  {
    case RoomScanMode::INIT:
      waitingForLidarData = true;
      roomScan.mode = RoomScanMode::WAIT_FOR_ROOM_SCAN;
      std::cout << "Init room scan..." << std::endl;
      break;
    case RoomScanMode::WAIT_FOR_ROOM_SCAN:
        if (!waitingForLidarData) {
          processLidarData(roomScan.currentScan == 0);
          publishCoordinates();

          roomScan.currentScan++;
          if (roomScan.currentScan >= roomScan.numScans) {
            publishLog("Scan complete, rotate to start position");
          }

          roomScan.mode = RoomScanMode::TURN;
        }
        break;
      case RoomScanMode::TURN:
          if (!haveOdom)
            break;    // no heading yet: wait for the first /odom message
          turnTargetYaw = roomScan.getTurnAngle();
          setSpeed(0.0, 0.5);
          roomScan.mode = RoomScanMode::WAIT_FOR_TURN;
          publishLog("Turning to: " + radiansToDegreesString(turnTargetYaw) + " deg");
          break;
      case RoomScanMode::WAIT_FOR_TURN: {
          double remaining = normalizeAngle(turnTargetYaw - currentYaw);
          if (std::fabs(remaining) < 0.01) {        // within ~2 degrees: done
            setSpeed(0.0, 0.0);
            waitingForLidarData = true;
            if (roomScan.currentScan >= roomScan.numScans) {
              roomScan.reset();
              mode = Mode::STOP;
            } else {
              roomScan.mode = RoomScanMode::WAIT_FOR_ROOM_SCAN;
            }
            std::cout << "Turn done, " << remaining * 180.0 / M_PI
                      << " deg from target" << std::endl;
            publishLog(radiansToDegreesString(currentYaw) + " deg");
          } else {
            // Slow down near the end so it doesn't overshoot: speed is
            // proportional to the angle left, capped at 0.5 rad/s, with a
            // 0.1 rad/s minimum so it doesn't stall just short of the target.
            double speed = std::clamp(1.5 * remaining, -0.5, 0.5);
            if (std::fabs(speed) < 0.1)
              speed = std::copysign(0.1, speed);
            setSpeed(0.0, speed);
          }
          break;
      }
    default:
      break;
  }
}

// Turn to turnTargetYaw (set by the /robot/move_and_scan service), take one
// scan, add it to the map, then go back to MOVE_FORWARD.
void moveAndScanLoop() {
  switch (moveAndScanStep)
  {
    case MoveAndScanStep::TURN:
      if (!haveOdom)
        break;    // no heading yet: wait for the first /odom message
      publishLog("Move & scan: turning to " + radiansToDegreesString(turnTargetYaw));
      moveAndScanStep = MoveAndScanStep::WAIT_FOR_TURN;
      break;

    case MoveAndScanStep::WAIT_FOR_TURN: {
      // Same turn control as the room scan.
      double remaining = normalizeAngle(turnTargetYaw - currentYaw);
      if (std::fabs(remaining) < 0.01) {
        setSpeed(0.0, 0.0);
        waitingForLidarData = true;    // ask lidarCb for the next scan
        moveAndScanStep = MoveAndScanStep::WAIT_FOR_SCAN;
      } else {
        double speed = std::clamp(1.5 * remaining, -0.5, 0.5);
        if (std::fabs(speed) < 0.1)
          speed = std::copysign(0.1, speed);
        setSpeed(0.0, speed);
      }
      break;
    }

    case MoveAndScanStep::WAIT_FOR_SCAN:
      if (!waitingForLidarData) {
        processLidarData(false);   // add to the existing map, don't clear it
        publishCoordinates();
        publishLog("Move & scan done at " + radiansToDegreesString(currentYaw));
        mode = Mode::STOP;
      }
      break;
  }
}

void loop() {
  std::lock_guard<std::mutex> lock(dataMutex);

  // loop() runs 20 times a second, so modes that wait only print when
  // they're first entered, not every time round.
  static std::optional<Mode> lastMode;
  bool justEntered = (lastMode != mode);
  lastMode = mode;

  switch (mode) {
      case Mode::SCAN_ROOM:
        scanRoomLoop();
        break;

      case Mode::MOVE_AND_SCAN:
        moveAndScanLoop();
        break;

      case Mode::MOVE_FORWARD:
          if (justEntered)
            std::cout << "Moving forward..." << std::endl;
          break;
  }
}

void handleInterrupt(int signal)
{
    running = false;
}