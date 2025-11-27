#include "Point2D.cpp"
#include <vector>
#include "AutoUtility.h"
#include <cmath>
#ifndef PATH
#define PATH

// Probably put guards in here eventually (#pragma once)

class Path // Linear, Quadratic, and Cubic Bezier Curves
{
public:
    Point2D start_position = Point2D(0, 0);
    Point2D end_position = Point2D(0, 0);
    Point2D control_point_1 = Point2D(0, 0);
    Point2D control_point_2 = Point2D(0, 0);

    bool linear = false;
    bool quadratic = false;
    bool cubic = false;

    double steps = 100; // maybe should consider converting this to a int

    double curve_length = 0;

    std::vector<Point2D> curvePoints;

    Path() {} // empty path

    Path(const Point2D start, const Point2D end)
        : start_position(start),
          end_position(end),
          linear(true)
    {
    }

    Path(const Point2D start, const Point2D control1, const Point2D end)
        : start_position(start),
          end_position(end),
          control_point_1(control1),
          quadratic(true)
    {
    }

    Path(const Point2D start, const Point2D control1, const Point2D control2, const Point2D end)
        : start_position(start),
          end_position(end),
          control_point_1(control1),
          control_point_2(control2),
          cubic(true)
    {
    }

    void setStepCount(double step_count)
    {
        steps = step_count;
    }

    void generatePath()
    {
        if (linear)
        {
            generateLinearPath();
        }
        else if (quadratic)
        {
            generateQuadraticPathEquation();
        }
        else
        {
            generateCubicPathEquation();
        }
    }

    // void generateCubicPath()
    // {
    //     curvePoints.clear();
    //     double t = 1.0 / steps;
    //     for (int i = 0; i <= steps; i++)
    //     {
    //         double current_interval = t * i;
    //         Point2D a = linearInterpolate(current_interval, start_position, control_point_1);
    //         Point2D b = linearInterpolate(current_interval, control_point_1, control_point_2);
    //         Point2D c = linearInterpolate(current_interval, control_point_2, end_position);

    //         Point2D ab = linearInterpolate(current_interval, a, b);
    //         Point2D bc = linearInterpolate(current_interval, b, c);

    //         Point2D final_point = linearInterpolate(current_interval, ab, bc);
    //         curvePoints.push_back(final_point);
    //     }
    // }

    void generateCubicPathEquation()
    {
        Point2D previousPoint = start_position;
        curvePoints.clear();
        double t_step = 1.0 / steps;
        for (int i = 0; i <= steps; i++)
        {
            double t = t_step * i;
            Point2D final_point = (start_position * std::pow(1 - t, 3)) + (control_point_1 * 3 * std::pow(1 - t, 2) * t) + (control_point_2 * 3 * (1 - t) * std::pow(t, 2)) + (end_position * std::pow(t, 3));
            curvePoints.push_back(final_point);
            curve_length = curve_length + calculateDistance(previousPoint, final_point);
            previousPoint = final_point;
        }
    }

    // void generateQuadraticPath()
    // {
    //     curvePoints.clear();
    //     double t = 1.0 / steps;
    //     for (int i = 0; i <= steps; i++)
    //     {
    //         double current_interval = t * i;
    //         Point2D pA = linearInterpolate(current_interval, start_position, control_point_1);
    //         Point2D pB = linearInterpolate(current_interval, control_point_1, end_position);
    //         Point2D final_point = linearInterpolate(current_interval, pA, pB);
    //         curvePoints.push_back(final_point);
    //     }
    // }

    void generateQuadraticPathEquation()
    {
        Point2D previousPoint = start_position;
        curvePoints.clear();
        double t_step = 1.0 / steps;
        
        for (int i = 0; i <= steps; i++)
        {
            double t = t_step * i;
            Point2D final_point = (start_position * std::pow(1 - t, 2)) + (control_point_1 * 2 * t * (1 - t)) + (end_position * std::pow(t, 2));
            curvePoints.push_back(final_point);
            curve_length = curve_length + calculateDistance(previousPoint, final_point);
            previousPoint = final_point;
        }
    }

    void generateLinearPath()
    {
        curve_length = calculateDistance(start_position, end_position);
        curvePoints.clear();
        double t = 1.0 / steps;
        for (int i = 0; i <= steps; i++)
        {
            double current_interval = t * i;
            Point2D final_point = linearInterpolate(current_interval, start_position, end_position);
            curvePoints.push_back(final_point);

        }
    }

    void resizeCurve(double distance_threshold = 0.1)
    { // hopefully I implemented this correctly
        std::vector<Point2D> newCurvePoints;

        for (int i = 0; i < curvePoints.size() - 1; i++)
        {
            Point2D point1 = curvePoints[i];
            Point2D point2 = curvePoints[i + 1];
            double distanceFromPoints = calculateDistance(point1, point2);
            if (distanceFromPoints > distance_threshold)
            {
                int spacing_times = static_cast<int>(std::ceil(distanceFromPoints / distance_threshold));
                for (int j = 0; j < spacing_times; j++)
                {
                    double t = static_cast<double>(j) / spacing_times;
                    Point2D final_point = linearInterpolate(t, point1, point2);
                    newCurvePoints.push_back(final_point);
                }
            }
            else
            {
                newCurvePoints.push_back(point1);
            }
        }

        newCurvePoints.push_back(curvePoints.back());

        curvePoints = std::move(newCurvePoints);
    }
};
#endif