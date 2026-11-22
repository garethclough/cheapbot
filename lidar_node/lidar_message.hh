#ifndef LIDAR_MESSAGE_HH
#define LIDAR_MESSAGE_HH

#include <gz/msgs/laserscan.pb.h>

class LidarMessage
{
public:
    // Stores a copy: the message passed to the lidar callback is only valid
    // while that callback runs, so keeping a reference to it would dangle.
    void setMessage(const gz::msgs::LaserScan &msg) { this->msg = msg; }

    const gz::msgs::LaserScan &getMessage() const { return msg; }

private:
    gz::msgs::LaserScan msg;
};

#endif
