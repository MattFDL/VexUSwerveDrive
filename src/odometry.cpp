#include <array>
#include <cmath>
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/screen.hpp"
#include "CommonUtility.h"

#ifndef ODOMETRY
#define ODOMETRY

class odometry
{
private:
    double position_rotation_sensor = 0;
    double position_rotation_sensor_pre = 0;
    double forward_count_pre = 0;     // forward rotation sensor previous count
    double rotation_count_pre = 0;    // sideways rotation sensor previous count
    uint32_t previousTime = pros::millis();

    double current_reading_imu_pid = 0; //Unused Currently, might be implemented later

    // SENSOR REFERENCES
    pros::Rotation &forwardRotation;  // forward rotation sensor
    pros::Rotation &sidewaysRotation; // rear rotation sensor
    pros::IMU imu;                   // inertial measurement unit

public:
    // position variables
    double position_x = 0;
    double position_y = 0;

    double velocity_x = 0;
    double velocity_y = 0;
    double velocity = 0;

    double rotation = 0;

    // use IMU for secondary heading claulation
    bool use_imu = false;

    /*
    USE THIS FOR THREADING:

    struct Pose {
        double x, y, theta, velocity;
    };
    pros::Mutex odom_mutex;
    Pose current_pose = {0, 0, 0, 0};
    
    Use this to update pose: 
    Pose new_pose{new_x, new_y, new_theta};

    odom_mutex.take();
    current_pose = new_pose;
    odom_mutex.give();

    Create this method:
    Pose get_pose() 
    {
    odom_mutex.take();
    Pose po = current_pose;   // copy
    odom_mutex.give();
    return po;
    }
    */

    //CONSTANTS FOR DISTANCE CALCULATIONS
    double const ROTATION_SENSOR_TICKS_TO_FULL_TURNS = 143580.0; //This value was determined experimentally (in 360 turns)
    double const FORWARD_SENSOR_TICKS_TO_INCHES = 5243.0; //This value was determined experimentally (in inches)

    // Constructor
    odometry(pros::Rotation &forward, pros::Rotation &sideways, pros::IMU &i) : forwardRotation(forward), sidewaysRotation(sideways), imu(i) {}

    void reset_sensors() {
        forwardRotation.reset(); //* DO NOT PUT IN CONSTRUCTOR PLEASE THIS CRASHES THINGS
        forwardRotation.reset_position();
        sidewaysRotation.reset();
        sidewaysRotation.reset_position();
        imu.reset();
    }

    void set_start_position(double start_x, double start_y, double start_theta) {
        position_x = start_x;
        position_y = start_y;
        position_rotation_sensor = start_theta;
        imu.set_heading(start_theta);
        
    }

    void calculate_postition() {
        double rotation_count_turns = (static_cast<double>(sidewaysRotation.get_position()) / ROTATION_SENSOR_TICKS_TO_FULL_TURNS); 
        double forward_count_inches = (static_cast<double>(forwardRotation.get_position()) / FORWARD_SENSOR_TICKS_TO_INCHES); 
    
        double deltaDegrees = (rotation_count_turns - rotation_count_pre) * 360.0;
        
        position_rotation_sensor += deltaDegrees;

        // change in forward distance
        double deltaPos = forward_count_inches - forward_count_pre; 

        // update heading based on either imu or rotation sensor
        // calculate current heading in degrees
        rotation = inputModulus(position_rotation_sensor, -180, 180);

        double position_rotation_rad = inputModulus((position_rotation_sensor + position_rotation_sensor_pre) * 0.5, -180, 180) * (M_PI / 180.0);
        //average rotation between time steps in radians for position calculations

        if (use_imu)
        {
            // get imu reading then convert to radians
            rotation = get_imu_reading();
            position_rotation_rad = rotation * (M_PI / 180.0);
        }
        
        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0;
        if (dt <= 0) {
            dt=0.001;
        }
        velocity = (forward_count_inches - forward_count_pre) / dt;
        // velocity_x = cos(position_rotation_rad) * velocity;
        // velocity_y = sin(position_rotation_rad) * velocity;

        position_x = position_x + (cos(position_rotation_rad) * deltaPos);
        position_y = position_y + (sin(position_rotation_rad) * deltaPos);


         // update previous counts        
        previousTime = currentTime;
        rotation_count_pre = rotation_count_turns;
        forward_count_pre = forward_count_inches;
        position_rotation_sensor_pre = position_rotation_sensor;
    }
       

    double get_imu_reading() {
        double reading_adjusted = (180 - imu.get_heading()); 
        // TODO:  might need to do PID stuff here.
        // TODO:  double reading_adjusted_pid = pid.calculate(current_reading_imu_pid, reading_adjusted);
        // TODO:  current_reading_imu_pid = reading_adjusted_pid;
        // TODO:  return reading_adjusted_pid;
        return reading_adjusted;
    }

};
#endif
