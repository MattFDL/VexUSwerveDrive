#include <array>
#include <cmath>
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/screen.hpp"

#ifndef ODOMETRY
#define ODOMETRY

class odometry
{
public:
    // position variables
    double position_x = 0;
    double position_y = 0;

    double velocity_x = 0;
    double velocity_y = 0;
    double velocity = 0;

    double position_rotation_sensor = 0;
    double adjusted_rotation = 0;
    double position_rotation_sensor_pre = 0;

    double position_rotation_imu = 0; // imu heading
    double forward_count_pre = 0;     // forward rotation sensor previous count
    double rotation_count_pre = 0;    // sideways rotation sensor previous count

    // use IMU for secondary heading claulation
    bool use_imu = false;
    double current_reading_imu_pid = 0; //Unused Currently, might be implemented later

    //CONSTANTS FOR DISTANCE CALCULATIONS
    double const ROTATION_SENSOR_TICKS_TO_FULL_TURNS = 143580.0; //This value was determined experimentally (in 360 turns)
    double const FORWARD_SENSOR_TICKS_TO_INCHES = 5243.0; //This value was determined experimentally (in inches)

    double totalPosition = 0; // testing purposes curently unused
    double preTotalPosition = 0; 


    double rotation_count_ticks_pre = 0;
    double forward_count_ticks_pre = 0;


    // SENSOR REFERENCES
    pros::Rotation &forwardRotation;  // forward rotation sensor
    pros::Rotation &sidewaysRotation; // rear rotation sensor
    pros::IMU imu;                   // inertial measurement unit

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
        totalPosition = forward_count_inches;
    
        double deltaRotationSensorTurns = rotation_count_turns - rotation_count_pre;
        double deltaDegrees = deltaRotationSensorTurns * 360.0;
        
        position_rotation_sensor += deltaDegrees;

        // change in forward distance
        double deltaPos = forward_count_inches - forward_count_pre; 

        // update heading based on either imu or rotation sensor
        /* #region update heading */
        // angles
        /*
        !   you convert turns to degrees here to then keep track of position_rotation_sensor in degrees
        !   then later you convert to radians without ever using position_rotation_sensor
        ?   would it make more sense to just get rid of position_rotation_sensor and set position_rotation_rad directly
        ?   deltaRad is also not needed it is declared to just add
        !   the rotation sensor is in turns of the wheel not 
        */

        // calculate current heading in radians
        adjusted_rotation = inputModulus((position_rotation_sensor + position_rotation_sensor_pre) * 0.5, -180, 180);  
        //average rotation between time steps

        double position_rotation_rad = adjusted_rotation * (M_PI / 180.0);
        if (use_imu)
        {
            // get imu reading then convert to radians
            position_rotation_imu = get_imu_reading();
            position_rotation_rad = position_rotation_imu * (M_PI / 180.0);
        }
        
        /* #endregion */

        // update current positon
        /* #region update position */
        // velocity = (static_cast<double>(forwardRotation.get_velocity()) / FORWARD_SENSOR_TICKS_TO_INCHES); 
        // velocity_x = cos(position_rotation_rad) * velocity;
        // velocity_y = sin(position_rotation_rad) * velocity;

        position_x = position_x + (cos(position_rotation_rad) * deltaPos);
        position_y = position_y + (sin(position_rotation_rad) * deltaPos);

        /* #endregion */

         // update previous counts
        
        rotation_count_pre = rotation_count_turns;
        forward_count_pre = forward_count_inches;
        position_rotation_sensor_pre = position_rotation_sensor;
        calculateVelocity();
    }
       


    uint32_t previousTime = pros::millis();
    void calculateVelocity() {
        uint32_t currentTime = pros::millis();
        double dt = (currentTime - previousTime) / 1000.0;
        if (dt <= 0) {
            dt=0.001;
        }

        //pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "DT: %f", dt);

        velocity = (totalPosition - preTotalPosition) / dt;
        // velocity_x = cos(position_rotation_rad) * velocity;
        // velocity_y = sin(position_rotation_rad) * velocity;

        preTotalPosition = totalPosition;  
        previousTime = currentTime;

    }


    double get_imu_reading() {
        double reading_adjusted = (180 - imu.get_heading()); 
        // TODO:  might need to do PID stuff here.
        // TODO:  double reading_adjusted_pid = pid.calculate(current_reading_imu_pid, reading_adjusted);
        // TODO:  current_reading_imu_pid = reading_adjusted_pid;
        // TODO:  return reading_adjusted_pid;
        return reading_adjusted;
    }

    double inputModulus(double val, double min, double max) { 
        //Keeps the value within the interval min to max
        //This should be used on the rotation readings for consistency purposes

        //Might move this function into a public file that can be used everywhere
        //Because it is also used in PIDController
        double value = val;
        double modulus = max - min;

        int numMax = (int)((value - min) / modulus); 
        value -= numMax * modulus;

        int numMin = (int)((value - max) / modulus);         value -= numMin * modulus;

        return value;
    }
};
#endif
