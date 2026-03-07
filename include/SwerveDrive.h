#ifndef SWERVEDRIVE
#define SWERVEDRIVE

#include "pros/rotation.hpp"
#include "SwerveModule.h"
#include "pros/imu.hpp"
#include "Odometry.h"
#include <utility>
#include <vector>
#include <cmath>
#include "PIDController.h"
#include "Pose.h"

class SwerveDrive {
    public:
        /*
        ----DRIVE CONSTANTS---------------
        */
        SwerveModule front_right;
        SwerveModule front_left;
        SwerveModule back_right;
        SwerveModule back_left;
        Odometry &odom;
        bool auto_in_use = false;
        PIDController pid_x = PIDController(9.0, 0.6, 0.0);//0.3
        PIDController pid_y = PIDController(4.5, 0.4, 0.0);
        PIDController pid_theta = PIDController(1.2, 0.1, 0.0);

        std::vector<SwerveModule> modules; 
        SwerveDrive(SwerveModule &fr, SwerveModule &fl, SwerveModule &br, SwerveModule &bl, Odometry &o);

        //void reset_sensors(); //These might get put in a different location later
        //double get_imu_reading();

        void drive_robot_orientated(double left_y_val, double left_x_val, double rot);
        void drive_field_orientated(double left_y_val, double left_x_val, double rot);
        void drive_to_point_and_rotation(double x, double y, double theta);
        bool drive_to_point_and_rotation_auto(Pose p, double ff = 5);
        bool drive_to_point_and_rotation_auto_slow(Pose p, double ff = 7);
        void stop_motors();

};
#endif