#include <array>
#include <cmath>
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

class odometry
{
public:
    double position_x = 0;
    double position_y = 0;
    double position_rotation_sensor = 0;

    bool use_imu = false;

    double position_rotation_imu = 0;

    double forward_count_pre = 0;
    double rotation_count_pre = 0;

    const double WHEEL_DIAMETER = 2.132;
    const double ROTATION_DISTANCE_WHEEL_RADIUS = 6.25;

    double fudge_factor_rotation = 1.2857;

    double test = 0;

    pros::Rotation forwardRotation;
    pros::Rotation sidewaysRotation;
    pros::IMU imu;

    double current_reading_imu_pid = 0;
    

    odometry(pros::Rotation& forward, pros::Rotation& sideways, pros::IMU& i) : forwardRotation(forward),
                                                           sidewaysRotation(sideways), 
                                                           imu(i)
    {}

    void reset_sensors() {
        forwardRotation.reset(); //DO NOT PUT IN CONSTRUCTOR PLEASE THIS CRASHES THINGS DONT REALLY KNOW WHY
        sidewaysRotation.reset();
        imu.reset();
    }

    void calculate_postition()
    {
        double rotation_count_turns = (static_cast<double>(sidewaysRotation.get_position()) / 36000.0);
        double forward_count_turns = (static_cast<double>(forwardRotation.get_position()) / 36000.0);

        test = rotation_count_turns;

        double deltaRotationSensorTurns = rotation_count_turns - rotation_count_pre;
        double deltaRad = ((deltaRotationSensorTurns * WHEEL_DIAMETER * M_PI) / (ROTATION_DISTANCE_WHEEL_RADIUS * 2 * M_PI)) * 360 * fudge_factor_rotation;
        position_rotation_sensor += deltaRad;

        double deltaForwardSensorTurns = forward_count_turns - forward_count_pre;
        double deltaPos = (deltaForwardSensorTurns * WHEEL_DIAMETER * M_PI);

        double position_rotation_rad = position_rotation_sensor * (M_PI / 180.0);
        if (use_imu)
        {
            position_rotation_imu = get_imu_reading();
            position_rotation_rad = position_rotation_imu * (M_PI / 180.0);
        }

        position_x = position_x + (cos(position_rotation_rad) * deltaPos);
        position_y = position_y + (sin(position_rotation_rad) * deltaPos);

        rotation_count_pre = rotation_count_turns;
        forward_count_pre = forward_count_turns;
    }
    double get_imu_reading() {
        double reading_adjusted = (180 - imu.get_heading());
        //might need to do PID stuff here. 
        // double reading_adjusted_pid = pid.calculate(current_reading_imu_pid, reading_adjusted);
        // current_reading_imu_pid = reading_adjusted_pid;
        // return reading_adjusted_pid;
        return reading_adjusted;
    }
};