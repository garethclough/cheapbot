#ifndef LINE_HH
#define LINE_HH

// A straight line in the world, e.g. a wall fitted to lidar points.
struct Line {
    double a = 0.0, b = 0.0, c = 0.0;
    // ax + by + c = 0
    int inliers = 0;   // number of points that lie on (close to) the line
};

#endif
