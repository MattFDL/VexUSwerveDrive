#include "SwerveModule.h"

#include "PIDController.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.hpp"
#include "CommonUtility.h"
#include <utility>
#include <cmath>

SwerveModule::SwerveModule(pros::Motor &t_m, pros::Motor &b_m, pros::Rotation &m_a, double position_x, double position_y, double mod_num, bool rev) : top_motor(t_m), bottom_motor(b_m), moduleAngle(m_a), pos_x(position_x), pos_y(position_y), module_number(mod_num), reversed(rev)
{
    turn_controller.enableContinuousInput(-180, 180);
    turn_controller.setIzone(0.4);
    turn_controller.setMaxMinI(10, -10);
    turn_controller.setErrorTolerance(0.2);
};

/*
Assumptions:
Both motors positive (right or cw) equals wheel moves left (ccw)
*/

void SwerveModule::set_state(std::pair<double, double> target_state)
{ // angle in (-180, 180) then velocity in inches
    double current_rotation_degs = (static_cast<double>(moduleAngle.get_angle()) / 100.0) - 180.0; // 0 to 36000 for the absolute encoder
    // needs to be kept between -180 and 180 for consistency

    // //angle optimization
    std::pair<double, double> new_target_state = optimize_angle(target_state, current_rotation_degs);
    double angle_setpoint = new_target_state.first;
    double vel = new_target_state.second;

    if (reversed)
    {
        vel = vel * -1;
    }

    vel = (vel * 100);
    // inches per second to voltage estimation
    // probably could do better later lol
    double rotation_gain = turn_controller.calculate(current_rotation_degs, angle_setpoint);

    double top_motor_speed = vel + rotation_gain;
    double bottom_motor_speed = -vel + rotation_gain;

    top_motor_speed = clamp(top_motor_speed, -12000, 12000);
    bottom_motor_speed = clamp(bottom_motor_speed, -12000, 12000);
    // TODO: need to normalize these speeds***********************

    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 2, "Angle: %f", current_rotation_degs);

    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Top Vel: %f", top_motor_speed);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 4, "Bottom Vel: %f", bottom_motor_speed);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 5, "Setpoint: %f", angle_setpoint);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "Velocity: %f", vel);
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "rotation_gain: %f", rotation_gain);

    top_motor.move_voltage(top_motor_speed);
    bottom_motor.move_voltage(bottom_motor_speed);
};

std::pair<double, double> SwerveModule::optimize_angle(std::pair<double, double> target_state, double current_angle)
{
    // angle optimization to choose the quickest path to get to the correct angle
    double angle_distance = std::abs(inputModulus(target_state.first - current_angle, -180, 180));
    // lowest distance calculator
    if (angle_distance < 90.0)
    {
        return target_state;
    }
    else
    {
        std::pair<double, double> temp(inputModulus(target_state.first + 180, -180, 180), target_state.second * -1);
        return temp;
    }
}
