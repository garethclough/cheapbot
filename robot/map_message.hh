#ifndef MAP_MESSAGE_HH
#define MAP_MESSAGE_HH

#include <vector>
#include "coordinate.hh"
#include "wall.hh"

// What the map looks like: the lidar points and the lines fitted to them.
class MapMessage
{
public:
    std::vector<Coordinate> coordinates;   // lidar hit points, metres
    std::vector<Wall> walls;               // lines found in those points
};

#endif
