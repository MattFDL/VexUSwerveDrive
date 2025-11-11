#include "Point2D.cpp"
#include "AutoUtility.h"

Point2D linearInterpolate(double decimal, Point2D start, Point2D end) {
        return Point2D(start.x + (decimal*(end.x-start.x)), end.y + (decimal*(end.y - start.y)));
};

