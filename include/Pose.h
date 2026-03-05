#ifndef POSE
#define POSE
struct Pose {
    double x;
    double y;
    double heading; 
    Pose(const  double m_x, const double m_y, const double m_heading) : x(m_x), y(m_y), heading(m_heading) {}

    // Point2D operator*(const double& multiplier) const {
    //     return Point2D(x*multiplier, y*multiplier);
    // }

    // Point2D operator+(const Point2D& point2) const {
    //     return Point2D(x + point2.x, y + point2.y);
    // }
};
#endif