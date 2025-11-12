#include "Point2D.cpp"
#include "AutoUtility.h"
#include <cmath>


Point2D linearInterpolate(double decimal, Point2D start, Point2D end) {
        return Point2D(start.x + (decimal*(end.x-start.x)), end.y + (decimal*(end.y - start.y)));
};

double calculateDistance(Point2D pt1, Point2D pt2) {
    return std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
};


