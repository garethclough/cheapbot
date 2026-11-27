#ifndef MAP_HH
#define MAP_HH

#include "coordinate.hh"
#include <vector>
#include "map_message.hh"
#include "wall.hh"
#include "corner.hh"
#include "room_metrics.hh"
#include "scan.hh"

class Map
{
public:
    void clear() {
        scans.clear();
    }
    void addScan(Scan scan);
    // A copy of the current points, walls, corners and room size, ready to publish.
    MapMessage getMessage() const;
    void update();
private:
    std::vector<Wall> walls;
    std::vector<Corner> corners;
    std::vector<Scan> scans;
    RoomMetrics room;

    // Geometry used for localisation. Not changed by update().
    std::vector<Wall> referenceWalls;
    std::vector<Corner> referenceCorners;
    bool hasReference = false;
};

#endif
