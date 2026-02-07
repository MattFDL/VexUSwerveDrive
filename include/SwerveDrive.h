#ifndef SWERVEDRIVE
#define SWERVEDRIVE
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "SwerveModule.h"
#include "pros/imu.hpp"
#include "pros/screen.hpp"
#include <utility>
#include <vector>
#include <functional>
#include <cmath>

class SwerveDrive {
    public:
        /*
        ----DRIVE CONSTANTS---------------
        */
        SwerveModule front_right;
        SwerveModule front_left;
        SwerveModule back_right;
        SwerveModule back_left;
        pros::IMU imu;

        std::vector<SwerveModule> modules; 
        SwerveDrive(SwerveModule &fr, SwerveModule &fl, SwerveModule &br, SwerveModule &bl, pros::IMU &i);

        void reset_sensors(); //These might get put in a different location later
        double get_imu_reading();

        void drive_robot_orientated(double left_y_val, double left_x_val, double rot);
        void drive_field_orientated(double left_y_val, double left_x_val, double rot);

};
#endif