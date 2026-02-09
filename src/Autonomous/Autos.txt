#ifndef AUTOS
#define AUTOS

#include "Command.cpp"
#include "AutoBuilder.cpp"
#include "Point2D.cpp"
#include <functional>
#include "Path.cpp"
#include "pneumatics.h"
#include <vector>
#include "pros/screen.hpp"

class Autos
{
    Path rightSidePath0;
    Path rightSidePath1;
    Path rightSidePath2;
    Path rightSidePath3;
    Path RightPath0;
    Path RightPath1;
    

    Path leftSidePath0;
    Path leftSidePath1;
    Path leftSidePath2;
    Path leftSidePath3;
    Path leftSidePath4;
    Path leftSidePath5;
    Path leftSidePath6;

public:
    Autos()
    {

        rightSidePath0 =Path(Point2D(72, 25.95749),Point2D(73, 65),Point2D(121, 45.02151),Point2D(118.7646, 17.2));//,false,false);
        rightSidePath1 =Path(Point2D(118.7646, 17.2),Point2D(120.5349, 28.13017));//,true,false);
        rightSidePath2 =Path(Point2D(120.5349, 28.13017),Point2D(120, 42));//,false,false);
        rightSidePath3 =Path(Point2D(120, 42),Point2D(122.0089, 23.92595),Point2D(124.5628, 22.60023));//,true,false);
        RightPath0=Path(Point2D(124.5628, 22.60023),Point2D(117.6888, 35.5462));//,false,false);
        RightPath1=Path(Point2D(117.6888, 35.5462),Point2D(92.41413, 45.02419),Point2D(105.7805, 91),Point2D(71.75697, 90));//,false,false);

        leftSidePath0 = Path(Point2D(56.24016, 21.23797), Point2D(52.3446, 64.324), Point2D(27.5537, 40.36225), Point2D(25.0, 16.0));
        leftSidePath1 = Path(Point2D(25.0, 16.0), Point2D(24.95657, 26.07805));
        leftSidePath2 = Path(Point2D(24.95657, 26.07805), Point2D(25.5, 40));
        leftSidePath3 = Path(Point2D(25.5, 40), Point2D(24.95657, 27.73077));
        leftSidePath4 = Path(Point2D(24.95657, 27.73077), Point2D(25.42878, 15.5));
        leftSidePath5 = Path(Point2D(24.95657, 27.73077), Point2D(40, 40));
        leftSidePath6 = Path(Point2D(40, 40), Point2D(58.0, 58.0));
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
    std::function<bool()> pneumatics_Activeate(bool raised, PneumaticCylinder &lift){
        auto output_func=[raised,&lift](){
            if (raised){
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

    void generateAutoRightSide()
    {
        rightSidePath0.generatePath();
        rightSidePath1.generatePath();
        rightSidePath2.generatePath();
        rightSidePath3.generatePath();
        RightPath0.generatePath();
        RightPath1.generatePath();
    }
    void generateAutoLeftSide()
    {
        leftSidePath0.generatePath();
        leftSidePath1.generatePath();
        leftSidePath2.generatePath();
        leftSidePath3.generatePath();
        leftSidePath4.generatePath();
        leftSidePath5.generatePath();
        leftSidePath6.generatePath();
    }
    AutoBuilder getAutoRightSide()
    {

        AutoBuilder builder = AutoBuilder();

        return builder;
    }
    AutoBuilder getTestAuto()
    {
        AutoBuilder builder = AutoBuilder();

        return builder;
    }
    AutoBuilder getAutoLeftSide()
    {
        AutoBuilder builder = AutoBuilder();

        return builder;
    }
};

#endif