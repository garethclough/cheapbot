#include "map.hh"
#include "coordinate.hh"
#include "wall.hh"
#include "corner.hh"
#include "room_metrics.hh"
#include "scan.hh"

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

struct Config {
    int iterations = 500;
    double distanceThreshold = 0.05; // metres
    int minInliers = 8;
    double minWallLength = 0.30;     // metres
    int maxWalls = 20;
};

Config config;

void Map::addScan(Scan scan) {
    scans.push_back(scan);
}

// Fit a line using Total Least Squares
// Returns normalised coefficients a, b, c
Wall fitLine(const std::vector<Coordinate>& points)
{
    double mx = 0, my = 0;

    for (const auto& p : points) {
        mx += p.x;
        my += p.y;
    }

    mx /= points.size();
    my /= points.size();

    double xx = 0, yy = 0, xy = 0;

    for (const auto& p : points) {
        double dx = p.x - mx;
        double dy = p.y - my;

        xx += dx * dx;
        yy += dy * dy;
        xy += dx * dy;
    }

    // Direction of the principal axis
    double angle = 0.5 * std::atan2(
        2.0 * xy, xx - yy
    );

    // Normal to the fitted line
    double a = -std::sin(angle);
    double b =  std::cos(angle);

    double c = -(a * mx + b * my);

    return {a, b, c, {}, {}, 0, 0};
}

// Perpendicular distance from a point to a line
double lineDistance(
    const Coordinate& p,
    const Wall& line)
{
    return std::abs(
        line.a * p.x +
        line.b * p.y +
        line.c
    );
}

Wall detectLine(
    const std::vector<Coordinate>& points,
    std::mt19937& rng)
{
    Wall best{};
    best.inliers = 0;

    if (points.size() < 2)
        return best;

    std::uniform_int_distribution<size_t> dist(
        0, points.size() - 1
    );

    for (int iteration = 0;
         iteration < config.iterations;
         ++iteration)
    {
        size_t i1 = dist(rng);
        size_t i2 = dist(rng);

        if (i1 == i2)
            continue;

        Coordinate p1 = points[i1];
        Coordinate p2 = points[i2];

        double a = p2.y - p1.y;
        double b = p1.x - p2.x;

        double norm = std::hypot(a, b);

        if (norm < 1e-9)
            continue;

        a /= norm;
        b /= norm;

        double c = -(a * p1.x + b * p1.y);

        Wall candidate{};
        candidate.a = a;
        candidate.b = b;
        candidate.c = c;
        candidate.inliers = 0;

        for (const auto& p : points) {
            if (lineDistance(p, candidate) <
                config.distanceThreshold)
            {
                candidate.inliers++;
            }
        }

        if (candidate.inliers > best.inliers)
            best = candidate;
    }

    return best;
}

// Find endpoints of the fitted wall
void calculateEndpoints(
    Wall& wall,
    const std::vector<Coordinate>& points)
{
    // Direction along the line
    double dx = -wall.b;
    double dy = wall.a;

    double minProjection =
        std::numeric_limits<double>::max();

    double maxProjection =
        std::numeric_limits<double>::lowest();

    for (const auto& p : points) {
        double projection = p.x * dx + p.y * dy;

        minProjection = std::min(
            minProjection, projection
        );

        maxProjection = std::max(
            maxProjection, projection
        );
    }

    wall.start = {
        dx * minProjection - wall.a * wall.c,
        dy * minProjection - wall.b * wall.c
    };

    wall.end = {
        dx * maxProjection - wall.a * wall.c,
        dy * maxProjection - wall.b * wall.c
    };

    wall.length = maxProjection - minProjection;
}

// Detect multiple walls
std::vector<Wall> detectWalls(std::vector<Coordinate> &coordinates)
{
    std::vector<Coordinate> remaining = coordinates;
    std::vector<Wall> walls;

    std::mt19937 rng(42);

    while (remaining.size() >=
           static_cast<size_t>(config.minInliers))
    {
        Wall candidate = detectLine(
            remaining, rng
        );

        if (candidate.inliers < config.minInliers)
            break;

        // Collect points supporting the candidate
        std::vector<Coordinate> inliers;

        for (const auto& p : remaining) {
            if (lineDistance(p, candidate) <
                config.distanceThreshold)
            {
                inliers.push_back(p);
            }
        }

        // Refine the line using all its inliers
        Wall wall = fitLine(inliers);

        // Recalculate inliers against refined line
        std::vector<Coordinate> refined;

        for (const auto& p : remaining) {
            if (lineDistance(p, wall) <
                config.distanceThreshold)
            {
                refined.push_back(p);
            }
        }

        if (refined.size() <
            static_cast<size_t>(config.minInliers))
            break;

        wall = fitLine(refined);
        wall.inliers = static_cast<int>(refined.size());

        calculateEndpoints(wall, refined);

        if (wall.length >= config.minWallLength) {
            walls.push_back(wall);
        }

        // Remove points belonging to this wall
        std::vector<Coordinate> next;

        for (const auto& p : remaining) {
            if (lineDistance(p, wall) >=
                config.distanceThreshold)
            {
                next.push_back(p);
            }
        }

        if (next.size() == remaining.size())
            break;

        remaining = std::move(next);

        if (static_cast<int>(walls.size()) >=
            config.maxWalls)
            break;
    }

    return walls;
}

// Check whether an intersection lies within or close to
// the finite extent of a wall segment.
bool nearWallSegment(
    const Corner& p,
    const Wall& wall,
    double tolerance)
{
    double dx = wall.end.x - wall.start.x;
    double dy = wall.end.y - wall.start.y;

    double lengthSquared = dx * dx + dy * dy;

    if (lengthSquared < 1e-12)
        return false;

    double t = (
        (p.x - wall.start.x) * dx +
        (p.y - wall.start.y) * dy
    ) / lengthSquared;

    double extension = tolerance / std::sqrt(lengthSquared);

    return t >= -extension && t <= 1.0 + extension;
}

// Calculate intersection of two wall lines
bool intersectWalls(
    const Wall& w1,
    const Wall& w2,
    Corner& corner)
{
    double determinant =
        w1.a * w2.b - w2.a * w1.b;

    if (std::abs(determinant) < 1e-8)
        return false;

    corner.x = (
        w1.b * w2.c - w2.b * w1.c
    ) / determinant;

    corner.y = (
        w1.c * w2.a - w2.c * w1.a
    ) / determinant;

    return true;
}

// Remove duplicate corners
std::vector<Corner> detectCorners(
    const std::vector<Wall>& walls,
    double tolerance = 0.20)
{
    std::vector<Corner> corners;

    for (size_t i = 0; i < walls.size(); ++i) {
        for (size_t j = i + 1; j < walls.size(); ++j) {

            Corner corner;

            if (!intersectWalls(
                walls[i], walls[j], corner))
                continue;

            // Ensure the intersection belongs to both
            // finite wall segments, not just their extensions.
            if (!nearWallSegment(
                corner, walls[i], tolerance))
                continue;

            if (!nearWallSegment(
                corner, walls[j], tolerance))
                continue;

            // Avoid duplicate corners
            bool duplicate = false;

            for (const auto& existing : corners) {
                double distance = std::hypot(
                    corner.x - existing.x,
                    corner.y - existing.y
                );

                if (distance < tolerance) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate)
                corners.push_back(corner);
        }
    }

    return corners;
}


void sortCorners(std::vector<Corner>& corners)
{
    if (corners.size() < 3)
        return;

    double cx = 0;
    double cy = 0;

    for (const auto& p : corners) {
        cx += p.x;
        cy += p.y;
    }

    cx /= corners.size();
    cy /= corners.size();

    std::sort(
        corners.begin(),
        corners.end(),
        [cx, cy](const Corner& a, const Corner& b) {
            double angleA = std::atan2(
                a.y - cy, a.x - cx
            );

            double angleB = std::atan2(
                b.y - cy, b.x - cx
            );

            return angleA < angleB;
        }
    );
}


RoomMetrics calculateRoomSize(
    const std::vector<Corner>& corners)
{
    RoomMetrics result;

    if (corners.size() < 3)
        return result;

    double twiceArea = 0;

    for (size_t i = 0; i < corners.size(); ++i) {

        const auto& p1 = corners[i];
        const auto& p2 =
            corners[(i + 1) % corners.size()];

        twiceArea +=
            p1.x * p2.y - p2.x * p1.y;

        result.perimeter += std::hypot(
            p2.x - p1.x,
            p2.y - p1.y
        );
    }

    result.area = std::abs(twiceArea) / 2.0;

    return result;
}


void Map::update() {
    std::vector<Coordinate> coordinates;
    for (const auto& scan : scans) {
        coordinates.insert(
            coordinates.end(),
            scan.worldPoints.begin(),
            scan.worldPoints.end()
        );
    }

    walls = detectWalls(coordinates);

    // Detect corners
    corners = detectCorners(walls);

    // Sort corners around the room
    sortCorners(corners);

    // Calculate room dimensions
    room = calculateRoomSize(corners);

    // Display results
    std::cout << "\nDetected corners:\n";

    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "Corner " << i + 1
                << ": "
                << corners[i].x << ", "
                << corners[i].y << "\n";
    }

    std::cout << "\nRoom measurements:\n";
    std::cout << "Area: " << room.area << " m2\n";
    std::cout << "Perimeter: " << room.perimeter << " m\n";
}

MapMessage Map::getMessage() const {
    MapMessage message;
    message.walls = walls;
    message.coordinates.clear();

    for (const auto& scan : scans) {
        message.coordinates.insert(
            message.coordinates.end(),
            scan.worldPoints.begin(),
            scan.worldPoints.end()
        );
    }
    message.corners = corners;
    message.room = room;
    return message;
}
