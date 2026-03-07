#include "intake.h"

Intake::Intake(pros::Motor &m_rollers, pros::Motor &m_lever, PneumaticCylinder &m_lifter, PneumaticCylinder &m_flap) : roller_motor(m_rollers), lever_motor(m_lever), lifter(m_lifter), flap(m_flap)
{
    lever_motor.set_gearing(pros::E_MOTOR_GEAR_RED);
    lever_motor.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS);
}
void Intake::lift_lever()
{
    set_lever = true;
    flap.extend();

    if (lifter.is_extended())
    {
        lever_motor.move_voltage(10000);
    }
    else
    {
        lever_motor.move_voltage(5000);
    }

    previousTime = pros::millis();
}
void Intake::lower_lever()
{
}
void Intake::reverse_rollers()
{
    roller_motor.move_velocity(-12000);
}
void Intake::run_rollers()
{
    roller_motor.move_velocity(12000);
    flap.retract();
}
void Intake::stop_rollers()
{
    roller_motor.move_velocity(0);
}
void Intake::handle_intake()
{
    pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 3, "Lever: %f", lever_motor.get_position());
    if (set_lever)
    {
        uint32_t currentTime = pros::millis();
        double period = (currentTime - previousTime) / 1000.0;
        // handle the logic to do lever here
        double ticks = lever_motor.get_position();

        if (period > 1)
        {
            if (reset_lever == false)
            {
                lever_motor.move_absolute(0, 70);
                reset_lever = true;
            }
            if (period > 1.5)
            {
                lever_motor.move_voltage(-700);
            }
            if (period > 1.7)
            {
                lever_motor.move_voltage(0);
                set_lever = false;
                reset_lever = false;
            }
        }
    }
}
void Intake::toggle_lift()
{

    lifter.toggle();
}
void Intake::full_send(double num)
{
    set_lever = true;
    lever_motor.move_voltage(num);
    previousTime = pros::millis();
}