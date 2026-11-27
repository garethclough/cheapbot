#include "map.hh"
#include "coordinate.hh"
#include "wall.hh"

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

void Map::update() {
    walls = detectWalls(coordinates);
}

MapMessage Map::getMessage() const {
    MapMessage message;
    message.walls = walls;
    message.coordinates = coordinates;
    return message;
}
