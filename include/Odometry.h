#ifndef ODOMETRY
#define ODOMETRY

#include <cmath>
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/screen.hpp"
#include "CommonUtility.h"
#include "Pose.h"

class Odometry {
    private:
        double horizontal_inches_previous = 0;
        double vertical_inches_previous = 0;
        uint32_t previousTime = pros::millis();

        pros::Rotation horizontal_rotation;
        pros::Rotation vertical_rotation;
        pros::IMU imu;

        const double HORIZONTAL_SENSOR_TICKS_TO_INCHES = 5280.35;
        const double VERTICAL_SENSOR_TICKS_TO_INCHES = 5280.35;
    public:
        double position_x = 0;
        double position_y = 0;
        Odometry(pros::Rotation &hr, pros::Rotation &vr, pros::IMU &i);
        void reset_sensors();
        void calculate_position();
        double get_imu_reading();
        double get_position_x();
        double get_position_y();
        void set_starting_position(Pose p);

};




#endif