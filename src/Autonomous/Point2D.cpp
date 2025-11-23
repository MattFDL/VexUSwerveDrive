#ifndef POINT2D
#define POINT2D
struct Point2D {
    double x;
    double y;
    Point2D(const  double m_x, const double m_y) : x(m_x), y(m_y) {}

    Point2D operator*(const double& multiplier) const {
        return Point2D(x*multiplier, y*multiplier);
    }

    Point2D operator+(const Point2D& point2) const {
        return Point2D(x + point2.x, y + point2.y);
    }

};



#endif
