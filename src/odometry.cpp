#include <array>
#include <cmath>
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

class odometry
{
public:
    // position variables
    double position_x = 0;
    double position_y = 0;
    double position_rotation_sensor = 0;
    double adjusted_rotation = 0;

    // use IMU for secondary heading claulation
    bool use_imu = true;


    double position_rotation_imu = 0; // imu heading
    double forward_count_pre = 0;     // forward rotation sensor previous count
    double rotation_count_pre = 0;    // sideways rotation sensor previous count


    // constants for chasis
    const double WHEEL_DIAMETER = 2.132;                // diameter of tracking wheel in inches
    //!this is never used
    //? can we use your ticks to deterimine this value more acurately
    //
    const double ROTATION_DISTANCE_WHEEL_RADIUS = 6.25; // distance from center of rotation to the rear tracking wheel in inches

    // const double fudge_factor_rotation = 1.2857;


    double test = 0; // testing purposes curently unused

    // SENSOR REFERENCES
    pros::Rotation forwardRotation;  // forward rotation sensor
    pros::Rotation sidewaysRotation; // rear rotation sensor
    pros::IMU imu;                   // inertial measurement unit


    // PID VALUES
    double current_reading_imu_pid = 0;


    // constructor
    odometry(pros::Rotation &forward, pros::Rotation &sideways, pros::IMU &i) : forwardRotation(forward), sidewaysRotation(sideways), imu(i)
    {
    }

    void reset_sensors()
    {
        forwardRotation.reset(); //* DO NOT PUT IN CONSTRUCTOR PLEASE THIS CRASHES THINGS DONT REALLY KNOW WHY
        forwardRotation.reset_position();
        sidewaysRotation.reset();
        sidewaysRotation.reset_position();
        imu.reset();
    }

    void calculate_postition()
    {
        double rotation_count_turns = (static_cast<double>(sidewaysRotation.get_position()) / 143580.0); //This value was determined experimentally (in 360 turns)
        double forward_count_turns = (static_cast<double>(forwardRotation.get_position()) / 5243.0); //This value was determined experimentally (in inches)
        test = forward_count_turns;
        

        double deltaRotationSensorTurns = rotation_count_turns - rotation_count_pre;
        double deltaDegrees = deltaRotationSensorTurns * 360.0;
        
        position_rotation_sensor += deltaDegrees;

        // change in foraward distance
        double deltaForwardSensorTurns = forward_count_turns - forward_count_pre;
        double deltaPos = deltaForwardSensorTurns; //* WHEEL_DIAMETER * M_PI);

        // update previous counts
        rotation_count_pre = rotation_count_turns;
        forward_count_pre = forward_count_turns;
        /* #endregion */

        // update heading based on either imu or rotation sensor
        /* #region update heading */
        // angels
        /*
        !   you convert turns to degrees here to then keep track of position_rotation_sensor in degrees
        !   then later you convert to radians without ever using position_rotation_sensor
        ?   would it make more sense to just get rid of position_rotation_sensor and set position_rotation_rad directly
        ?   deltaRad is also not needed it is declared to just add
        !   the rotation sensor is in turns of the wheel not 
        */
        double deltaRad = deltaRotationSensorTurns * 360.0;
        position_rotation_sensor += deltaRad;

        // calculate current heading in radians
        double position_rotation_rad = position_rotation_sensor * (M_PI / 180.0);
        if (use_imu)
        {
            // get imu reading then convert to radians
            position_rotation_imu = get_imu_reading();
            position_rotation_rad = position_rotation_imu * (M_PI / 180.0);
        }
        
        /* #endregion */

        // update current positon
        /* #region update position */

        adjusted_rotation = inputModulus(position_rotation_sensor, -180, 180);

        position_x = position_x + (cos(position_rotation_rad) * deltaPos);
        position_y = position_y + (sin(position_rotation_rad) * deltaPos);

        /* #endregion */
    }
    double get_imu_reading()
    {
        double reading_adjusted = (180 - imu.get_heading()); 

        // TODO:  might need to do PID stuff here.
        // TODO:  double reading_adjusted_pid = pid.calculate(current_reading_imu_pid, reading_adjusted);
        // TODO:  current_reading_imu_pid = reading_adjusted_pid;
        // TODO:  return reading_adjusted_pid;
        return reading_adjusted;
    }
    double inputModulus(double val, double min, double max) {
        double value = val;
        double modulus = max - min;

        int numMax = (int)((value - min) / modulus); 
        //casting to int should round down to nearest int (if it works the same as in java)
        value -= numMax * modulus;

        int numMin = (int)((value - max) / modulus); 
        //casting to int should round down to nearest int (if it works the same as in java)
        value -= numMin * modulus;

        return value;
    }
};