#include "Autos.h"
#include <functional>
#include "pneumatics.h"
#include <vector>
#include "pros/screen.hpp"

Autos::Autos()
{
}

std::function<bool()> Autos::waitCommand(double delay_secs)
{
    auto start_time_ptr = std::make_shared<std::optional<uint32_t>>();

    return [start_time_ptr, delay_secs]() -> bool
    {
        if (!start_time_ptr->has_value())
        {
            *start_time_ptr = pros::millis();
        }

        double current_time_from_start = (pros::millis() - start_time_ptr->value()) / 1000.0;

        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "X: %f", current_time_from_start);

        if (current_time_from_start >= delay_secs)
        {
            return true;
        }
        return false;
    };
}
std::function<bool()> Autos::activate_pneumatics(bool raised, PneumaticCylinder &lift)
{
    auto output_func = [raised, &lift]()
    {
        if (raised)
        {
            lift.extend();
        }
        else
        {
            lift.retract();
        }
        return true;
    };
    return output_func;
}
std::function<bool()> Autos::waitUntilPnuematics(double delay_secs, PneumaticCylinder &cylinder, bool active)
{
    auto start_time_ptr = std::make_shared<std::optional<uint32_t>>();

    return [start_time_ptr, delay_secs, &cylinder, active]() -> bool
    {
        if (!start_time_ptr->has_value())
        {
            *start_time_ptr = pros::millis();
        }

        double current_time_from_start = (pros::millis() - start_time_ptr->value()) / 1000.0;

        pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "X: %f", current_time_from_start);

        if (current_time_from_start >= delay_secs)
        {
            if (active)
            {
                cylinder.extend();
            }
            else
            {
                cylinder.retract();
            }
            return true;
        }
        return false;
    };
}
std::function<bool()> Autos::goToPose(Pose p, SwerveDrive &drive) {
    auto func = [&drive, p]() -> bool {
        bool b = drive.drive_to_point_and_rotation_auto(p);
        return b;
    };
    return func;
}

AutoBuilder Autos::getAutoRightSide()
{

    AutoBuilder builder = AutoBuilder();

    return builder;
}
AutoBuilder Autos::getTestAuto()
{
    AutoBuilder builder = AutoBuilder();

    return builder;
}
AutoBuilder Autos::getAutoLeftSide(SwerveDrive &drive, Odometry &odom)
{
    AutoBuilder builder = AutoBuilder();
    
    builder.add_command(Command([&odom, this](){
        odom.set_starting_position(leftSideStart);
        return true;
    }));
    builder.add_command(Command(goToPose(leftside1, drive)));
    builder.add_command(Command(goToPose(leftside2, drive)));
    //builder.add_command(Command(goToPose(leftside3, drive)));


    return builder;
}
