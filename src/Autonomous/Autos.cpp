#ifndef AUTOS
#define AUTOS

#include "Command.cpp"
#include "AutoBuilder.cpp"
#include "Point2D.cpp"
#include <functional>
#include "../odometry.cpp"
#include "Path.cpp"
#include "PathFollower.cpp"

class Autos
{
public:
    PathFollower follower;
    Autos(PathFollower &f) : follower(f)
    {
    }

    std::function<bool()> waitCommand(double delay_secs, double test)
    {
        auto start_time_ptr = std::make_shared<std::optional<uint32_t>>();

        return [start_time_ptr, delay_secs, test]() -> bool
        {
            if (!start_time_ptr->has_value())
            {
                *start_time_ptr = pros::millis();
            }

            double current_time_from_start = (pros::millis() - start_time_ptr->value()) / 1000.0;

            pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 6, "X: %f", current_time_from_start);

            if (current_time_from_start >= delay_secs)
            {
                pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM, 7, "Done: %f", test);
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

    AutoBuilder getAutoRightSide()
    {
        AutoBuilder builder = AutoBuilder();
        Path rightSidePath0(Point2D(54.41873, 16.5433), Point2D(54.66749, 46.97948), Point2D(23.09941, 42.71954), Point2D(24.91983, 16.46387));
        Path rightSidePath1(Point2D(24.91983, 16.46387), Point2D(25.08891, 28.65559));
        Path rightSidePath2(Point2D(25.08891, 28.65559), Point2D(25.0889, 39.11803));
        Path rightSidePath3(Point2D(25.0889, 39.11803), Point2D(42.61952, 6.623184), Point2D(37.10316, 112.6259), Point2D(76.78007, 85.47221));

        rightSidePath0.generatePath();
        rightSidePath1.generatePath();
        rightSidePath2.generatePath();
        rightSidePath3.generatePath();

        builder.add_command(Command(setPathFollower(follower, rightSidePath0)));
        builder.add_command(Command(follow_path(follower, 10)));

        builder.add_command(Command(setPathFollower(follower, rightSidePath1)));
        builder.add_command(Command(follow_path(follower, 10, true)));

        builder.add_command(Command(rotate_to_degrees(follower, 90)));

        builder.add_command(Command(setPathFollower(follower, rightSidePath2)));
        builder.add_command(Command(follow_path(follower, 10)));

        builder.add_command(Command(setPathFollower(follower, rightSidePath3)));
        builder.add_command(Command(follow_path(follower, 15, true)));

        return builder;
    }
    AutoBuilder getTestAuto()
    {
        AutoBuilder builder = AutoBuilder();
        builder.add_command(Command(waitCommand(10, 1)));
        builder.add_command(Command(waitCommand(5, 2)));

        return builder;
    }
};

#endif