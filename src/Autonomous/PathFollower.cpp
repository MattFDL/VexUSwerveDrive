#include "../PIDController.cpp" //change these to headers eventually
#include "../odometry.cpp" //change these to headers eventually
#include "pros/motor_group.hpp"
#include "Path.cpp"

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
    
    //std::vector<> curvePoints;

    PathFollower(odometry& odometry, pros::MotorGroup &m_right, pros::MotorGroup &m_left) : odom(odometry), right_mg(m_right), left_mg(m_left)
    {

    }

    void setPath(Path m_path) {
        path = m_path;
    }

    void setContraints() {
        //TODO: create contraints class (acceleration, max_velocity)
    }

};