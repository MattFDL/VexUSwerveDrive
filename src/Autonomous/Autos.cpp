#ifndef AUTOS
#define AUTOS

#include "Command.cpp"
#include "AutoBuilder.cpp"
#include "Point2D.cpp"
#include <functional>
#include "../odometry.cpp"
#include "Path.cpp"
#include "PathFollower.cpp"
#include "pneumatics.h"
#include <vector>

class Autos
{
    Path rightSidePath0;
    Path rightSidePath1;
    Path rightSidePath2;
    Path rightSidePath3;
    Path rightSidePath4;
    Path rightSidePath5;

public:
    Autos()
    {
        //rightSidePath0 = Path(Point2D(54.41873, 16.5433), Point2D(54.66749, 46.97948), Point2D(23.09941, 42.71954), Point2D(24.91983, 16.46387));
        //rightSidePath1 = Path(Point2D(24.91983, 16.46387), Point2D(25.08891, 28.65559));
        //rightSidePath2 = Path(Point2D(25.08891, 28.65559), Point2D(25.0889, 39.11803));
        //rightSidePath3 = Path(Point2D(25.0889, 39.11803), Point2D(42.61952, 6.623184), Point2D(37.10316, 112.6259), Point2D(76.78007, 85.47221));


        rightSidePath0 =Path(Point2D(72, 25.95749),Point2D(81.5609, 46.98569),Point2D(121, 45.02151),Point2D(118.7646, 17.25));//,false,false);
        //rightSidePath5 =Path(Point2D(118.7646, 18),Point2D(118.7646, 17.2));
        rightSidePath1 =Path(Point2D(118.7646, 17.25),Point2D(120.5349, 28.13017));//,true,false);
        rightSidePath2 =Path(Point2D(120.5349, 28.13017),Point2D(120, 41.5));//,false,false);
        rightSidePath3 =Path(Point2D(120, 41.5),Point2D(122.0089, 23.92595),Point2D(124.5628, 22.60023));//,true,false);
        rightSidePath4 =Path(Point2D(124.5628, 22.60023),Point2D(132, 25));//,false,false);


    }

    std::function<bool()> waitCommand(double delay_secs)
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
    std::function<bool()> setPathFollower(PathFollower &f, Path &p)
    {
        auto path_set = [&f, &p]()
        {
            f.setPath(p);
            return true;
        };
        return path_set;
    }
    std::function<bool()> follow_path(PathFollower &f, double vel, bool backwards = false)
    {

        auto path_follower = [&f, backwards, vel]()
        {
            bool b = f.followPath(backwards, vel);
            return b;
        };
        return path_follower;
    }
    std::function<bool()> rotate_to_degrees(PathFollower &f, double deg)
    {
        auto rotaton_follower = [&f, deg]()
        {
            bool b = f.rotateToDegrees(deg);
            return b;
        };
        return rotaton_follower;
    }
    std::function<bool()> start_intake(int32_t intake_vel, pros::MotorGroup &m_intake)
    {
        auto intake_func = [intake_vel, &m_intake]()
        {
            m_intake.move(-intake_vel);
            return true;
        };
        return intake_func;
    }
    std::function<bool()> set_start_position(const Path p, PathFollower &f, const double deg)
    {
        auto set_start_func = [p, &f, deg]()
        {
            f.odom.set_start_position(p.start_position.x, p.start_position.y, deg);
            return true;
        };
        return set_start_func;
    }

    std::function<bool()> pneumatics_Activeate(bool raised, PneumaticCylinder &lift){
        auto output_func=[raised,&lift](){
            if (raised){
                lift.extend();
            }
            else{
                lift.retract();
            }
            return true;
        };
        return output_func;
    }
    std::function<bool()> waitUntilPnuematics(double delay_secs, PneumaticCylinder &cylinder, bool active)
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
                if (active) {
                    cylinder.extend();
                } else {
                    cylinder.retract();
                }
                return true;
            }
            return false;
        };
    }

    void generateAutoRightSide()
    {
        rightSidePath0.generatePath();
        rightSidePath1.generatePath();
        rightSidePath2.generatePath();
        rightSidePath3.generatePath();
        rightSidePath4.generatePath();
    }
    AutoBuilder getAutoRightSide(PathFollower &follower, pros::MotorGroup &intake,PneumaticCylinder &rake,PneumaticCylinder &matchLoad,PneumaticCylinder &flap, PneumaticCylinder &lifter)
    {

        AutoBuilder builder = AutoBuilder();

        builder.add_command(Command(pneumatics_Activeate(true,rake)));
        builder.add_command(Command(waitCommand(.5)));
        
        builder.add_command(Command(pneumatics_Activeate(true,matchLoad)));
        
        
        builder.add_command(Command(setPathFollower(follower, rightSidePath0)));

        std::vector<std::function<bool()>> command_list1;
        command_list1.push_back(follow_path(follower, 14));
        command_list1.push_back(waitUntilPnuematics(3, matchLoad, true));
        command_list1.push_back(waitUntilPnuematics(3, rake, false));

        builder.add_command(Command(command_list1, CommandType::Parallel));
        

        builder.add_command(Command(pneumatics_Activeate(false,rake)));


        builder.add_command(Command(start_intake(127, intake)));
        builder.add_command(Command(waitCommand(3)));

        builder.add_command(Command(start_intake(0, intake)));


        builder.add_command(Command(setPathFollower(follower, rightSidePath1)));
        builder.add_command(Command(follow_path(follower, 10, true)));

        builder.add_command(Command(pneumatics_Activeate(false,matchLoad)));
        builder.add_command(Command(rotate_to_degrees(follower, 90)));
        builder.add_command(Command(pneumatics_Activeate(true,lifter)));

        builder.add_command(Command(setPathFollower(follower, rightSidePath2)));
        builder.add_command(Command(follow_path(follower, 10)));

        
        builder.add_command(Command(pneumatics_Activeate(true,flap)));
        builder.add_command(Command(start_intake(127,intake)));
        builder.add_command(Command(waitCommand(3)));
        builder.add_command(Command(start_intake(0, intake)));

        builder.add_command(Command(setPathFollower(follower, rightSidePath3)));
        builder.add_command(Command(follow_path(follower, 16, true)));

        builder.add_command(Command(start_intake(127,intake)));
        
        builder.add_command(Command(rotate_to_degrees(follower,0)));


        builder.add_command(Command(setPathFollower(follower, rightSidePath4)));
        builder.add_command(Command(follow_path(follower, 5)));
        

        builder.add_command(Command(start_intake(0, intake)));
        
        return builder;
    }
    AutoBuilder getTestAuto()
    {
        AutoBuilder builder = AutoBuilder();
        // builder.add_command(Command(waitCommand(5)));
        // builder.add_command(Command(start_intake(127, intake), waitCommand(5), CommandType::Parallel));
        // builder.add_command(Command(start_intake(0, intake)));

        return builder;
    }
};

#endif