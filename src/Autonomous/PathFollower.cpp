#include "../PIDController.cpp" //change these to headers eventually
#include "../odometry.cpp" //change these to headers eventually
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

    void driveMotor(double direction, double turn)
    {
        left_mg.move(direction - turn);
        right_mg.move(direction + turn);
    }

public:
    VelocityContraints constraints = VelocityContraints(5.0, 5.0);
    int currentIndex = 0;
    
    PathFollower(odometry& odometr, pros::MotorGroup &m_right, pros::MotorGroup &m_left) : odom(odometr), right_mg(m_right), left_mg(m_left)
    { 

    }

    
    void setPath(Path m_path) {
        path = m_path;
    }

    void setContraints(double acceleration, double max_vel) {
        constraints = VelocityContraints(acceleration, max_vel);
    }

    void followPath() {

    }

    int findLookAheadPoint(double lookahead, double x, double  y) {
        //Get Current Index / Closest Point to the robots position
        double closest_distance = std::numeric_limits<double>::infinity();
        for (int i = currentIndex; i < (path.curvePoints.size() - 1); i++) {
            double distance_current_to_next = calculateDistance(Point2D(x, y), path.curvePoints[i]);
            if (distance_current_to_next < closest_distance) {
                closest_distance = distance_current_to_next;
                currentIndex = i;
            }
        }

        currentIndex = std::max(0, currentIndex-1); //get previous index to account for delay in time not sure if needed
        //----------------------------------------------//
        //calculate the lookahead point 
        //this is the point that the robot will attempt to move towards
        int lookahead_index = currentIndex + 1;
        double look_ahead_error = std::numeric_limits<double>::infinity();
        for (int e = currentIndex; e < (path.curvePoints.size() - 1); e++) {
            double distance_to_point = calculateDistance(Point2D(x, y), path.curvePoints[e]);//calculateDistance(path.curvePoints[currentIndex], path.curvePoints[e]);
            double error = std::abs(distance_to_point-lookahead);
            if (error < look_ahead_error) {
                look_ahead_error = error;
                lookahead_index = e;
            }
        } 
        //This algorithm could technically be improved
        //Think about it we could technically be stop searching for index's in the loop if we notice that the number's
        //error keeps increasing 
        //however, this algorithm 100% works
        return lookahead_index;

    }

};