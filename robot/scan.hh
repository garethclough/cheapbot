#ifndef SCAN_HH
#define SCAN_HH

#include <vector>
#include "coordinate.hh"

// One lidar scan's hit points, in two frames.
class Scan
{
public:
    // Relative to the lidar: x forward, y to the left, metres.
    std::vector<Coordinate> localPoints;
    // The same points placed in the world using the robot's pose, metres.
    std::vector<Coordinate> worldPoints;
};

#endif
