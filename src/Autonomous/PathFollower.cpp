#include "../PIDController.cpp" //change these to headers eventually
#include "../odometry.cpp"      //change these to headers eventually
#include "pros/motor_group.hpp"
#include "Path.cpp"
#include "VelocityConstraints.cpp"
#include "FeedForwardBasic.cpp"
#include "CommonUtility.h"

class PathFollower
{
private:
    pros::MotorGroup &right_mg;
    pros::MotorGroup &left_mg;
    Path path = Path();
    odometry odom;

public:
    //VelocityContraints constraints = VelocityContraints(5.0, 15.0);
    int currentIndex = 0;
    double currentTime = 0;
    const double LOOKAHEAD_DISTANCE = 1.5; /// Might want to not use a constant and change this value in respect to current velocity
    //Anything lower than 1.5 seems to not work
    const double TRACK_WIDTH = 10.0;
    PIDController lastPostionPID = PIDController(2.0, 0.1, 0);
    PIDController rotationPID = PIDController(20.0, 3.0, 0); // PIDController(4.0, 1.8, 0);

    bool startPID = false;

    const double kS = 0.4;
    const double kV = 0.19607;
    FeedForwardBasic ff = FeedForwardBasic(kS, kV, 0);

    bool pathFinished = false;

    PathFollower(odometry &odometr, pros::MotorGroup &m_right, pros::MotorGroup &m_left) : odom(odometr), right_mg(m_right), left_mg(m_left)
    {
        lastPostionPID.setIzone(5);
        lastPostionPID.setMaxMinI(10, -10);
        lastPostionPID.setErrorTolerance(0.3);
        rotationPID.enableContinuousInput(-M_PI, M_PI);
        rotationPID.setIzone(0.4);
        rotationPID.setMaxMinI(10, -10);
        rotationPID.setErrorTolerance(0.1);
    }

    void driveMotor(double left_volts, double right_volts)
    {
        left_mg.move_voltage(static_cast<int32_t>(left_volts * 1000));
        right_mg.move_voltage(static_cast<int32_t>(right_volts * 1000));
    }

    void driveMotorVelocity(double left_velocity, double right_velocity)
    {
        driveMotor(ff.calculateVolts(left_velocity), ff.calculateVolts(right_velocity));
    }

    void setPath(Path m_path)
    {
        path = m_path;
    }


    void resetPath() {
        pathFinished = false; 
        startPID = false;
    }

    void followPath(double m_x, double m_y, double heading) //maybe pass velocity into here as well
    {   
        double x = m_x; // odom.position_x;
        double y = m_y; // odom.position_y;

        double heading_rad = (heading / 180) * M_PI; //(odom.position_rotation_sensor / 180) * M_PI;
        double vel = 8;                              // constraints.getCurrentVelocity(currentTime, path.curve_length);
        
        // or use the IMU both should be on the interval [-Pi, Pi]
        double adjusted_look_distance = (vel / 6.0) * LOOKAHEAD_DISTANCE;

        int lookAhead_index = findLookAheadPoint(adjusted_look_distance, x, y);

        Point2D look_ahead_point = path.curvePoints[lookAhead_index];
        double heading_error = inputModulus(std::atan2(look_ahead_point.y - y, look_ahead_point.x - x) - heading_rad, -M_PI, M_PI);
       

        if (((calculateDistance(Point2D(x, y), path.curvePoints.back()) < 5) && (std::abs(heading_error) < 0.3)) || startPID)
        {
            startPID = true;
            // maybe two stages, one for squaring up with target the other for driving to target
            Point2D pid_point = path.curvePoints.back();
            double pid_heading_error = inputModulus(std::atan2(pid_point.y - y, pid_point.x - x) - heading_rad, -M_PI, M_PI);
            double rot_value = rotationPID.calculate(pid_heading_error, 0);

            if (!rotationPID.atTolerance() )
            { //&& (calculateDistance(Point2D(x, y), path.curvePoints.back()) > 1)) {
                double rot_value = rotationPID.calculate(pid_heading_error, 0);
                driveMotorVelocity(rot_value, -rot_value);
                return;
            }

            double input = calculateDistance(path.curvePoints.back(), Point2D(x, y));

            if (pid_heading_error > 2.6 || pid_heading_error < -2.6)
            {
                input = -input;
            }

            double vel = lastPostionPID.calculate(input, 0);
            if (!lastPostionPID.atTolerance())
            {
                driveMotorVelocity(-vel, -vel);
            } else {
                driveMotorVelocity(0, 0);
                pathFinished = true;
            }
            return;
        }

        // This will calculate the turning in order to follow the path
        // There are technically two ways of doing this:
        // Curvature or using a PID Controller + Error

        double curvature = 2 * std::sin(heading_error) / adjusted_look_distance;
        // rate in change in direction of a curve at a specific point
        double angular_vel = vel * curvature;

        double vel_left = vel - (angular_vel * 0.5 * TRACK_WIDTH);
        double vel_right = vel + (angular_vel * 0.5 * TRACK_WIDTH);

        driveMotorVelocity(vel_left, vel_right);
    }

    void followPathBackwards(double m_x, double m_y, double heading) //maybe pass velocity into here as well
    {   
        double x = m_x; // odom.position_x;
        double y = m_y; // odom.position_y;

        double heading_rad = inputModulus((((heading + 180) / 180) * M_PI), -M_PI, M_PI); //+180 for backwards
        double vel = 8;

        double adjusted_look_distance = (vel / 6.0) * LOOKAHEAD_DISTANCE;

        vel = vel * -1; //negative vel for backwards has to be after the look distance is calculated


        int lookAhead_index = findLookAheadPoint(adjusted_look_distance, x, y);

        Point2D look_ahead_point = path.curvePoints[lookAhead_index];
        double heading_error = inputModulus(std::atan2(look_ahead_point.y - y, look_ahead_point.x - x) - heading_rad, -M_PI, M_PI);

        if (((calculateDistance(Point2D(x, y), path.curvePoints.back()) < 5) && (std::abs(heading_error) < 0.3)) || startPID)
        {
            startPID = true;
            // maybe two stages, one for squaring up with target the other for driving to target
            Point2D pid_point = path.curvePoints.back();
            double pid_heading_error = inputModulus(std::atan2(pid_point.y - y, pid_point.x - x) - heading_rad, -M_PI, M_PI);
            double rot_value = rotationPID.calculate(pid_heading_error, 0);

            if (!rotationPID.atTolerance() && (calculateDistance(Point2D(x, y), path.curvePoints.back()) > 1))
            { 
                double rot_value = rotationPID.calculate(pid_heading_error, 0);
                driveMotorVelocity(rot_value, -rot_value);
                return;
            }

            double input = calculateDistance(path.curvePoints.back(), Point2D(x, y));

            if (pid_heading_error > 2.6 || pid_heading_error < -2.6)
            {
                input = -input;
            }

            double vel = lastPostionPID.calculate(input, 0);
            if (!lastPostionPID.atTolerance())
            {
                driveMotorVelocity(vel, vel); //reversed for backwards
            } else {
                driveMotorVelocity(0, 0);
                pathFinished = true;
            }
            return;
        }

        double curvature = 2 * std::sin(heading_error) / adjusted_look_distance;
        // rate in change in direction of a curve at a specific point
        double angular_vel = vel * curvature;

        double vel_left = vel + (angular_vel * 0.5 * TRACK_WIDTH);
        double vel_right = vel - (angular_vel * 0.5 * TRACK_WIDTH);

        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Velocity Left: %f", vel_left);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Velocity Right: %f", vel_right);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Heading Error: %f", heading_error);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "Angular: %f", angular_vel);

        driveMotorVelocity(vel_left, vel_right);

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