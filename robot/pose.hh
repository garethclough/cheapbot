#ifndef POSE_HH
#define POSE_HH

#include "coordinate.hh"

// Where the robot is and which way it faces.
class Pose
{
public:
    Coordinate position{0.0, 0.0};   // metres, world frame
    double yaw = 0.0;                // radians, -pi..pi, 0 = facing +x
};

#endif
