#include "../PIDController.cpp" //change these to headers eventually
#include "../odometry.cpp"      //change these to headers eventually
#include "pros/motor_group.hpp"
#include "Path.cpp"
#include "VelocityConstraints.cpp"
#include "AutoUtility.h"

class PathFollower
{
private:
    pros::MotorGroup &right_mg;
    pros::MotorGroup &left_mg;
    Path path = Path();
    odometry odom;

public:
    VelocityContraints constraints = VelocityContraints(5.0, 5.0);
    int currentIndex = 0;
    double currentTime = 0;
    const double LOOKAHEAD_DISTANCE = 0.5; // Might want to not use a constant and change this value in respect to current velocity
    const double TRACK_WIDTH = 10.0;

    PathFollower(odometry &odometr, pros::MotorGroup &m_right, pros::MotorGroup &m_left) : odom(odometr), right_mg(m_right), left_mg(m_left)
    {
    }

    void driveMotor(double left_volts, double right_volts)
    {
        left_mg.move_voltage(static_cast<int32_t>(left_volts*1000));
        right_mg.move_voltage(static_cast<int32_t>(right_volts*1000));
    }

    void driveMotorVelocity(double left_velocity, double right_velocity) {
        
    }

    void setPath(Path m_path)
    {
        path = m_path;
    }

    void setContraints(double acceleration, double max_vel)
    {
        constraints = VelocityContraints(acceleration, max_vel);
    }

    void followPath()
    {

        double x = odom.position_x;
        double y = odom.position_y;
        double heading_rad = (odom.position_rotation_sensor / 180) * M_PI;
        double vel = constraints.getCurrentVelocity(currentTime, path.curve_length);
        // or use the IMU both should be on the interval [-Pi, Pi]

        Point2D look_ahead_point = path.curvePoints[findLookAheadPoint(LOOKAHEAD_DISTANCE, x, y)];
        double heading_error = std::atan2(look_ahead_point.y - y, look_ahead_point.x - x) - heading_rad;

        // This will calculate the turning in order to follow the path
        // There are technically two ways of doing this:
        // Curvature or using a PID Controller + Error

        double curvature = 2 * std::sin(heading_error) / LOOKAHEAD_DISTANCE;
        // rate in change in direction of a curve at a specific point
        double angular_vel = vel * curvature;

        double vel_left = vel - (angular_vel * 0.5 * TRACK_WIDTH);
        double vel_right = vel + (angular_vel * 0.5 * TRACK_WIDTH);

        driveMotorVelocity(vel_left, vel_right);

        // If near the end might want to consider switching to only PID Controller

        currentTime += 0.2; // This assumes the code loops every 0.2 seconds
        // Could potentially use unsigned int = pros::millis() and get the difference between time calls
    }

    int findLookAheadPoint(double lookahead, double x, double y)
    {
        // Get Current Index / Closest Point to the robots position
        double closest_distance = std::numeric_limits<double>::infinity();
        for (int i = currentIndex; i < (path.curvePoints.size() - 1); i++)
        {
            double distance_current_to_next = calculateDistance(Point2D(x, y), path.curvePoints[i]);
            if (distance_current_to_next < closest_distance)
            {
                closest_distance = distance_current_to_next;
                currentIndex = i;
            }
        }

        currentIndex = std::max(0, currentIndex - 1); // get previous index to account for delay in time not sure if needed
        //----------------------------------------------//
        // calculate the lookahead point
        // this is the point that the robot will attempt to move towards
        int lookahead_index = currentIndex + 1;
        double look_ahead_error = std::numeric_limits<double>::infinity();
        for (int e = currentIndex; e < (path.curvePoints.size() - 1); e++)
        {
            double distance_to_point = calculateDistance(Point2D(x, y), path.curvePoints[e]); // calculateDistance(path.curvePoints[currentIndex], path.curvePoints[e]);
            double error = std::abs(distance_to_point - lookahead);
            if (error < look_ahead_error)
            {
                look_ahead_error = error;
                lookahead_index = e;
            }
        }
        // This algorithm could technically be improved
        // Think about it we could technically be stop searching for index's in the loop if we notice that the number's
        // error keeps increasing
        // however, this algorithm 100% works
        return lookahead_index;
    }
};