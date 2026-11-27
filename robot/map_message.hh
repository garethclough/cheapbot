#ifndef MAP_MESSAGE_HH
#define MAP_MESSAGE_HH

#include <vector>
#include "coordinate.hh"
#include "wall.hh"
#include "corner.hh"
#include "room_metrics.hh"

// What the map looks like: the lidar points, the walls fitted to them, the
// room's corners and its size.
class MapMessage
{
public:
    std::vector<Coordinate> coordinates;   // lidar hit points, metres
    std::vector<Wall> walls;               // lines found in those points
    std::vector<Corner> corners;           // where walls meet, in order round the room
    RoomMetrics room;                      // area and perimeter from the corners
};

#endif
