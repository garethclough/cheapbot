#ifndef GAZEBO_COMMS_HH
#define GAZEBO_COMMS_HH

#include <string>
#include <vector>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

// The Gazebo side of the robot program, for talking to the simulator.
class GazeboComms
{
public:
    // Set up the drive command publisher on `cmdVelTopic` (the diff-drive
    // plugin in building_robot.sdf listens on /cmd_vel).
    // Returns false if it couldn't be advertised.
    bool init(const std::string &cmdVelTopic = "/cmd_vel");

    // Drive the robot: linear = forward speed in m/s, angular = turn rate in
    // rad/s (+ = left). The diff-drive plugin keeps using the last command,
    // so the robot carries on at this speed until told otherwise.
    // Returns false if init() hasn't succeeded or the publish failed.
    bool setSpeed(double linear, double angular);

    // Stop the robot (setSpeed(0, 0)).
    bool stop();

    // Call `callback` for every message on `topic`. The message type comes
    // from the callback, e.g. subscribe("/odom", odomCb) where
    //     void odomCb(const gz::msgs::Odometry &msg);
    // Callbacks run on gz-transport's own threads, so lock any shared data.
    // Returns false if the subscription failed.
    // (A template, so it has to be defined here in the header.)
    template <typename MessageT>
    bool subscribe(const std::string &topic,
                   void (*callback)(const MessageT &))
    {
        if (!node.Subscribe(topic, callback))
            return false;
        subscribedTopics.push_back(topic);
        return true;
    }

    // Stop every subscription made with subscribe(), so no more callbacks run.
    void unsubscribeAll();

private:
    gz::transport::Node node;
    // Not valid until init() has advertised it.
    gz::transport::Node::Publisher cmdVelPublisher;
    // Remembered so unsubscribeAll() knows what to undo.
    std::vector<std::string> subscribedTopics;
};

#endif
