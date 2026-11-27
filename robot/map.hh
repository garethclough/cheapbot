#ifndef MAP_HH
#define MAP_HH

#include "coordinate.hh"
#include <vector>
#include "map_message.hh"
#include "wall.hh"
#include "corner.hh"
#include "room_metrics.hh"

class Map
{
public:
    void clear() {
        coordinates.clear();
    }
    void insert(const std::vector<Coordinate>& newCoordinates) {
        coordinates.insert(coordinates.end(), newCoordinates.begin(), newCoordinates.end());
    }
    // A copy of the current points, walls, corners and room size, ready to publish.
    MapMessage getMessage() const;
    void update();
private:
    std::vector<Coordinate> coordinates;
    std::vector<Wall> walls;
    std::vector<Corner> corners;
    RoomMetrics room;
};

#endif
