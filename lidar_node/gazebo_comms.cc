#include "gazebo_comms.hh"

bool GazeboComms::init(const std::string &cmdVelTopic)
{
    cmdVelPublisher = node.Advertise<gz::msgs::Twist>(cmdVelTopic);
    return cmdVelPublisher.Valid();
}

bool GazeboComms::setSpeed(double linear, double angular)
{
    if (!cmdVelPublisher.Valid())
        return false;   // init() not called, or it failed

    gz::msgs::Twist cmd;
    cmd.mutable_linear()->set_x(linear);
    cmd.mutable_angular()->set_z(angular);
    return cmdVelPublisher.Publish(cmd);
}

bool GazeboComms::stop()
{
    // A Twist with nothing set is all zeros, but be explicit.
    return setSpeed(0.0, 0.0);
}

void GazeboComms::unsubscribeAll()
{
    for (const std::string &topic : subscribedTopics)
        node.Unsubscribe(topic);
    subscribedTopics.clear();
}
