#include "SwerveDrive.h"

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "SwerveModule.h"
#include "pros/imu.hpp"
#include "pros/screen.hpp"
#include <utility>
#include <vector>
#include <functional>
#include <cmath>

SwerveDrive::SwerveDrive(SwerveModule &fr, SwerveModule &fl, SwerveModule &br, SwerveModule &bl, Odometry &o) : front_right(fr), front_left(fl), back_right(br), back_left(bl), odom(o)
{
    modules.push_back(front_right);
    modules.push_back(front_left);
    modules.push_back(back_right);
    modules.push_back(back_left);
    pid_theta.enableContinuousInput(-180, 180);
    pid_theta.setErrorTolerance(8);
    pid_theta.setIzone(10);
    pid_theta.setMaxMinI(20, -20);
    pid_x.setIzone(5);
    pid_x.setMaxMinI(20, -20);
    pid_x.setErrorTolerance(1.5);
    pid_y.setIzone(3); //5
    pid_y.setMaxMinI(20, -20);
    pid_y.setErrorTolerance(1.5);
    //modules = {front_right, front_left, back_right, back_left};
}

// void SwerveDrive::reset_sensors()
// {
//     imu.reset(true); // blocking
// }
// double SwerveDrive::get_imu_reading()
// {
//     double reading_adjusted = (180 - imu.get_heading());
//     // TODO:  might need to do PID stuff here.
//     // TODO:  double reading_adjusted_pid = pid.calculate(current_reading_imu_pid, reading_adjusted);
//     // TODO:  current_reading_imu_pid = reading_adjusted_pid;
//     // TODO:  return reading_adjusted_pid;
//     return reading_adjusted;
// }

void SwerveDrive::drive_robot_orientated(double left_y_val, double left_x_val, double rot)
{
    double velocity_robot_x = left_y_val * 0.7874; // converstion 100/127 to convert from controller to inches per second
    double velocity_robot_y = left_x_val * 0.7874;

    // rotation 2.4 full turns per second
    // 12.57 radians per second using 2 turns per second as an estimate
    double velocity_robot_rotation = rot * 0.0989763; // wheel_speed = w x r
    for (auto mod : modules)
    { // maybe change to a reference
        double x_mod_pos = mod.pos_x;
        double y_mod_pos = mod.pos_y;
        double vx_module = velocity_robot_x - (velocity_robot_rotation * y_mod_pos);
        double vy_module = velocity_robot_y + (velocity_robot_rotation * x_mod_pos);
        double angle = (180 * (std::atan2(vy_module, vx_module) / M_PI));
        double speed = std::sqrt(std::pow(vx_module, 2) + std::pow(vy_module, 2));
        std::pair<double, double> state(angle, speed);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, mod.module_number * 2, "Angle: %f", angle);
        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, mod.module_number * 2 + 1, "Speed: %f", speed);
        mod.set_state(state);
    }
}

void SwerveDrive::stop_motors() {
    for (auto mod : modules)
    {
        mod.stop();
    }
}
void SwerveDrive::drive_field_orientated(double left_y_val, double left_x_val, double rot) //inches per sec
{
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Y Value: %f", left_y_val);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "X Value: %f", left_x_val);
    double velocity_field_x = -left_y_val * 0.7874; // converstion 100/127 to convert from controller to inches per second
    double velocity_field_y = -left_x_val * 0.7874;

    double theta = odom.get_imu_reading() * (M_PI / 180);

   

    double velocity_robot_x = velocity_field_x * std::cos(theta) + velocity_field_y * std::sin(theta);
    double velocity_robot_y = -velocity_field_x * std::sin(theta) + velocity_field_y * std::cos(theta);

    // rotation 2.4 full turns per second
    // 12.57 radians per second using 2 turns per second as an estimate
    double velocity_robot_rotation = rot * 0.0989763; // wheel_speed = w x r
    if (abs(rot) < 5) {
        velocity_robot_rotation = 0;
    }

    for (auto mod : modules)
    {
        double x_mod_pos = mod.pos_x;
        double y_mod_pos = mod.pos_y;
        double vx_module = velocity_robot_x - (velocity_robot_rotation * y_mod_pos);
        double vy_module = velocity_robot_y + (velocity_robot_rotation * x_mod_pos);
        double angle = std::atan2(vy_module, vx_module) * (180 / M_PI);
        double speed = std::sqrt(std::pow(vx_module, 2) + std::pow(vy_module, 2));
        std::pair<double, double> state(angle, speed);
        // pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, mod.module_number * 2, "Angle: %f", angle);
        // pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, mod.module_number * 2 + 1, "Speed: %f", speed);
        
        mod.set_state(state);
        
        
    }
}

void SwerveDrive::drive_to_point_and_rotation(double x, double y, double theta) {
    double x_position = odom.get_position_x();
    double y_position = odom.get_position_y();
    double theta_position = inputModulus(theta, -180, 180);

    double x_target = x;
    double y_target = y;

    double x_pow = clamp(pid_x.calculate(x_position, x_target), -100, 100);
    double y_pow = clamp(pid_y.calculate(y_position, y_target), -100, 100);

    
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "X position: %f", x_position);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Y position: %f", y_position);
    double theta_pow = pid_theta.calculate(odom.get_imu_reading(), theta);
    drive_field_orientated(x_pow, y_pow, -theta_pow);

}
// bool SwerveDrive::drive_to_point_and_rotation_auto(Pose p) {
//     double x_position = odom.get_position_x();
//     double y_position = odom.get_position_y();
//     double theta_position = inputModulus(p.heading, -180, 180);

//     double x_target = p.x;
//     double y_target = p.y;

//     double x_pow = pid_x.calculate(x_position, x_target);
//     double y_pow = pid_y.calculate(y_position, y_target);

//     pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "X position: %f", x_position);
//     pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Y position: %f", y_position);
    
//     double theta_pow = pid_theta.calculate(odom.get_imu_reading(), theta_position);
//     drive_field_orientated(x_pow, y_pow, -theta_pow);
//     if (pid_x.atTolerance() && pid_y.atTolerance() && pid_theta.atTolerance()) {
//         return true;
//     } else {
//         return false; 
//     }
// }
bool SwerveDrive::drive_to_point_and_rotation_auto(Pose p, double ff) {
    auto_in_use = true;
    double x_position = odom.get_position_x();
    double y_position = odom.get_position_y();
    double theta_position = inputModulus(p.heading, -180, 180);

    double x_target = p.x;
    double y_target = p.y;

    double x_error = (x_target - x_position);
    double y_error = (y_target - y_position);

    double error_hypo = std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2));

    double theta_pow = pid_theta.calculate(odom.get_imu_reading(), theta_position);

    double pid_out = clamp(-(pid_x.calculate(error_hypo,0)), -80, 80);
    double total_output = pid_out + ff;
    //total output will always have the same sign
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Output: %f", total_output);

    double position_angle = std::atan2(y_error, x_error);
    double x_pow = std::cos(position_angle) * total_output;
    double y_pow = std::sin(position_angle) * total_output;
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Angle: %f", (position_angle / M_PI) * 180);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "X-Pow: %f", x_pow);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 10, "Y Pow: %f", y_pow);


    drive_field_orientated(x_pow, y_pow, -theta_pow);
    if (pid_x.atTolerance() && pid_theta.atTolerance()) {
        auto_in_use = false;
        return true;
    } else {
        return false; 
    }
}
bool SwerveDrive::drive_to_point_and_rotation_auto_slow(Pose p, double ff) {
    auto_in_use = true;
    double x_position = odom.get_position_x();
    double y_position = odom.get_position_y();
    double theta_position = inputModulus(p.heading, -180, 180);

    double x_target = p.x;
    double y_target = p.y;

    double x_error = (x_target - x_position);
    double y_error = (y_target - y_position);

    double error_hypo = std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2));

    double theta_pow = pid_theta.calculate(odom.get_imu_reading(), theta_position);

    double pid_out = clamp(-(pid_y.calculate(error_hypo,0)), -40, 40);
    double total_output = pid_out + ff;
    //total output will always have the same sign
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Output: %f", total_output);

    double position_angle = std::atan2(y_error, x_error);
    double x_pow = std::cos(position_angle) * total_output;
    double y_pow = std::sin(position_angle) * total_output;
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 8, "Angle: %f", (position_angle / M_PI) * 180);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 9, "X-Pow: %f", x_pow);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 10, "Y Pow: %f", y_pow);


    drive_field_orientated(x_pow, y_pow, -theta_pow);
    if (pid_y.atTolerance() && pid_theta.atTolerance()) {
        auto_in_use = false;
        return true;
    } else {
        return false; 
    }
}

