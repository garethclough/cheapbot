#include "ros2_comms.hh"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>


void Ros2Comms::init(std::string nodeName) {

    // Start ROS 2 without its own Ctrl+C handler, so ours below stays in charge.
    // No args as configured in GUI
    int argc = 0;
    char ** argv = nullptr;

    rclcpp::init(argc, argv, rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    rosNode = rclcpp::Node::make_shared(nodeName);
    rosThread = std::thread([node = rosNode] { rclcpp::spin(node); });
}

void Ros2Comms::shutdown() {
    rclcpp::shutdown();
    rosThread.join();
    rosNode.reset();

    for (auto &service : services) {
        service.reset();
    }

    for (auto &publisher : publishers) {
        publisher.reset();
    }
}

int Ros2Comms::addService(const std::string &serviceName, Handler handler) {
  auto service = rosNode->create_service<diagnostic_msgs::srv::AddDiagnostics>(
      serviceName,
      [handler](
          const std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Request> request,
          std::shared_ptr<diagnostic_msgs::srv::AddDiagnostics::Response> response)
      {
          response->success = handler(request->load_namespace,
                                            response->message);
      }
  );
services.push_back(service);
  return static_cast<int>(services.size() - 1);   // index of the one just added
}

void Ros2Comms::publish2DCoordinates(int publisherIndex, const std::vector<Coordinate> &coordinates) {

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

  publish(publisherIndex, cloud);
}
void Ros2Comms::publishMap(int coordinatesIndex, int wallsIndex, int roomIndex,
                           const MapMessage &map) {
  publish2DCoordinates(coordinatesIndex, map.coordinates);

  std_msgs::msg::Float64MultiArray wallsMsg;
  for (const Wall &wall : map.walls) {
    wallsMsg.data.push_back(wall.start.x);
    wallsMsg.data.push_back(wall.start.y);
    wallsMsg.data.push_back(wall.end.x);
    wallsMsg.data.push_back(wall.end.y);
  }
  publish(wallsIndex, wallsMsg);

  std_msgs::msg::Float64MultiArray roomMsg;
  roomMsg.data.push_back(map.room.area);
  roomMsg.data.push_back(map.room.perimeter);
  for (const Corner &corner : map.corners) {
    roomMsg.data.push_back(corner.x);
    roomMsg.data.push_back(corner.y);
  }
  publish(roomIndex, roomMsg);
}
