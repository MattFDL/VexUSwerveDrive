#ifndef SWERVEDRIVE
#define SWERVEDRIVE
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "SwerveModule.cpp"
#include <utility>
#include <vector>
#include <functional>
#include <cmath>


class SwerveDrive {
    public:
        /*
        ----DRIVE CONSTANTS---------------
        */
        SwerveModule &front_right;
        SwerveModule &front_left;
        SwerveModule &back_right;
        SwerveModule &back_left;

        std::vector<std::reference_wrapper<SwerveModule>> modules; 


        SwerveDrive(SwerveModule &fr, SwerveModule &fl, SwerveModule &br, SwerveModule &bl) : 
        front_right(fr), front_left(fl), back_right(br), back_left(bl) {
            modules.push_back(front_right);
            modules.push_back(front_left);
            modules.push_back(back_right);
            modules.push_back(back_left);
        }

        void drive_robot_orientated(double left_y_val, double left_x_val, double rot) {
            double velocity_robot_x = left_y_val * 0.7874; //converstion 100/127 to convert from controller to inches per second
            double velocity_robot_y = left_x_val * 0.7874;

            //rotation 2.4 full turns per second
            //12.57 radians per second using 2 turns per second as an estimate
            double velocity_robot_rotation = rot * 0.0989763; //wheel_speed = w x r
            for (const auto& mod : modules) {
                double x_mod_pos = mod.get().pos_x;
                double y_mod_pos = mod.get().pos_y;
                double vx_module = velocity_robot_x - (velocity_robot_rotation * y_mod_pos);
                double vy_module = velocity_robot_y + (velocity_robot_rotation * x_mod_pos);
                double angle = (180 * (std::atan2(vy_module, vx_module) / M_PI));
                double speed = std::sqrt(std::pow(vx_module, 2) + std::pow(vy_module,2));
                std::pair<double,double> state(angle, speed);
                pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, mod.get().module_number * 2, "Angle: %f", angle);
                pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, mod.get().module_number * 2 + 1, "Speed: %f", speed);

                //mod.get().set_state(state);
            }
            
        }

};
#endif