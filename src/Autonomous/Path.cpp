#include "Point2D.cpp"
#include <vector>
#include "AutoUtility.h"

class Path //Linear, Quadratic, and Cubic Bezier Curves
{
public:
    Point2D start_position;
    Point2D end_position;
    Point2D control_point_1 = Point2D(0, 0);
    Point2D control_point_2 = Point2D(0, 0);

    bool linear = false;
    bool quadratic = false;
    bool cubic = false;

    double steps = 100;

    std::vector<Point2D> curvePoints;

    Path(Point2D start, Point2D end)
        : start_position(start),
          end_position(end),
          linear(true)
    {}

    Path(Point2D start, Point2D control1, Point2D end)
        : start_position(start),
          end_position(end),
          control_point_1(control1),
          quadratic(true)
    {}

    Path(Point2D start, Point2D control1, Point2D control2, Point2D end)
        : start_position(start),
          end_position(end),
          control_point_1(control1),
          control_point_2(control2),
          cubic(true)
    {}

    void setStepCount(double step_count) {
        steps = step_count;
    }

    void generatePath() {
        if (linear) {
            generateLinearPath();
        } else if (quadratic) {
            generateQuadraticPath();
        } else {
            generateCubicPath();
        }
    }

    void generateLinearPath() {
        curvePoints.clear();
        double t = 1.0 / steps;
        for (int i = 0; i <= steps; i++) { 
            double current_interval = t*i;
            Point2D a = linearInterpolate(current_interval, start_position, control_point_1);
            Point2D b = linearInterpolate(current_interval, control_point_1, control_point_2);
            Point2D c = linearInterpolate(current_interval, control_point_2, end_position);

            Point2D ab = linearInterpolate(current_interval, a, b);
            Point2D bc = linearInterpolate(current_interval, b, c);

            Point2D final_point = linearInterpolate(current_interval, ab, bc);
            curvePoints.push_back(final_point);
        }
    }

    void generateQuadraticPath() {
        curvePoints.clear();
        double t = 1.0 / steps;
        for (int i = 0; i <= steps; i++) {
            double current_interval = t*i;
            Point2D pA = linearInterpolate(current_interval, start_position, control_point_1);
            Point2D pB = linearInterpolate(current_interval, control_point_1, end_position);
            Point2D final_point = linearInterpolate(current_interval, pA, pB);
            curvePoints.push_back(final_point);
        }
    }

    void generateCubicPath() {
        curvePoints.clear();
        double t = 1.0 / steps;
        for (int i = 0; i <= steps; i++) {
            double current_interval = t*i;
            Point2D final_point = linearInterpolate(current_interval, start_position, end_position);
            curvePoints.push_back(final_point);
        }
    }

};