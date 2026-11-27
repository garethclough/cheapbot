#ifndef ROS2_COMMS_HH
#define ROS2_COMMS_HH

#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/srv/add_diagnostics.hpp>
#include "coordinate.hh"
#include "map_message.hh"
#include <sensor_msgs/msg/point_cloud2.hpp>

class Ros2Comms
{
public:
    using Handler = std::function<bool(const std::string &text, std::string &message)>;

    void init(std::string nodeName);
    void shutdown();
    int addService(const std::string &serviceName, Handler handler);
    void publish2DCoordinates(int publisherIndex, const std::vector<Coordinate> &coordinates);
    // Publish a map: its points as a PointCloud2 on coordinatesIndex, and its
    // walls as a Float64MultiArray on wallsIndex, 4 values per wall:
    // [start x, start y, end x, end y, ...] in metres.
    void publishMap(int coordinatesIndex, int wallsIndex, const MapMessage &map);

    template <typename T>
    int addPublisher(std::string topicName,
                     const rclcpp::QoS &qos = rclcpp::QoS(10)) {
        auto publisher = rosNode->create_publisher<T>(topicName, qos);
        publishers.push_back(publisher);
        return static_cast<int>(publishers.size() - 1);   // index of the one just added
    }

    template <typename T>
    void publish(int publisherIndex, const T &message) {
        auto publisher =
            std::dynamic_pointer_cast<rclcpp::Publisher<T>>(
                publishers[publisherIndex]);

        if (publisher) {
            publisher->publish(message);
        }
    }

private:
    rclcpp::Node::SharedPtr rosNode;
    std::thread rosThread;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::vector<rclcpp::PublisherBase::SharedPtr> publishers;
};

#endif
