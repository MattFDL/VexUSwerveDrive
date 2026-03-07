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
std::function<bool()> Autos::goToPoseSlow(Pose p, SwerveDrive &drive) {
    auto func = [&drive, p]() -> bool {
        bool b = drive.drive_to_point_and_rotation_auto_slow(p);
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
AutoBuilder Autos::noMatchLoadAuto(SwerveDrive &drive, Odometry &odom, Intake &intake)
{
    AutoBuilder builder = AutoBuilder();
    
    builder.add_command(Command([&odom, this](){
        odom.set_starting_position(noMatchStart);
        return true;
    }));
    builder.add_command(Command(goToPose(noMatch1, drive)));
    builder.add_command(Command([&intake, this](){
        intake.run_rollers();
        return true;
    }));
    builder.add_command(Command(goToPose(noMatch2, drive)));
    builder.add_command(Command(goToPose(noMatch3, drive)));

    builder.add_command(Command([&intake, this](){
        intake.flap.extend();
        return true;
    }));

    builder.add_command(Command(goToPose(noMatch4, drive)));
    builder.add_command(Command([&intake, this](){
        intake.lift_lever();
        return true;
    }));
    builder.add_command(Command(waitCommand(3)));
    builder.add_command(Command([&intake, this](){
        intake.lift_lever();
        return true;
    }));
    builder.add_command(Command(waitCommand(3)));
    builder.add_command(Command([&intake, this](){
        intake.lift_lever();
        return true;
    }));
    builder.add_command(Command(waitCommand(3)));
    builder.add_command(Command([&intake, this](){
        intake.toggle_lift();
        intake.full_send(5000);
        return true;
    }));
    
    return builder;
}
AutoBuilder Autos::matchLoadAuto(SwerveDrive &drive, Odometry &odom, Intake &intake, PneumaticCylinder &matchLoadMech) {
    AutoBuilder builder = AutoBuilder();
    
    builder.add_command(Command([&odom, this](){
        odom.set_starting_position(matchLoadStart);
        return true;
    }));
    builder.add_command(Command(goToPose(matchLoad1, drive)));
    builder.add_command(Command([&matchLoadMech, &intake](){
        matchLoadMech.extend();
        intake.run_rollers();
        return true;
    }));
    builder.add_command(Command(goToPose(matchLoad2, drive)));
    builder.add_command(Command(waitCommand(1)));
    builder.add_command(Command([&intake](){
        intake.toggle_lift();
        return true;
    }));
    builder.add_command(Command(goToPose(matchLoad3, drive)));
    builder.add_command(Command([&intake](){
        intake.lift_lever();
        return true;
    }));
    builder.add_command(Command(goToPose(matchLoad4, drive)));
    builder.add_command(Command(waitCommand(1)));
    builder.add_command(Command(goToPose(matchLoad5, drive)));
    builder.add_command(Command([&intake](){
        intake.lift_lever();
        return true;
    }));


    return builder;
}
