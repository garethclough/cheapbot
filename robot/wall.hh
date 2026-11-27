#ifndef WALL_HH
#define WALL_HH

#include "coordinate.hh"

// A wall found in the lidar points: the infinite line it lies on
// (ax + by + c = 0) plus the segment of that line the points actually cover.
class Wall
{
public:
    double a = 0.0, b = 0.0, c = 0.0;   // line: ax + by + c = 0
    Coordinate start{0.0, 0.0};          // one end of the wall, metres
    Coordinate end{0.0, 0.0};            // the other end, metres
    int inliers = 0;                     // number of points on the wall
    double length = 0.0;                 // distance from start to end, metres
};

#endif
