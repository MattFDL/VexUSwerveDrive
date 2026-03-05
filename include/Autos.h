#ifndef AUTOS
#define AUTOS

#include <functional>
#include "pneumatics.h"
#include <vector>
#include "pros/screen.hpp"
#include "AutoBuilder.h"
#include "Pose.h"
#include "SwerveDrive.h"
#include "Odometry.h"

class Autos {
    public:

    Autos();
    const Pose leftSideStart = Pose(0, 0, 0);
    const Pose leftside1 = Pose(20, 20, 0);

    const Pose leftside2 = Pose(0, 0, 0);

    const Pose leftside3 = Pose(38, 0, 135);

    std::function<bool()> waitCommand(double delay_secs);
    std::function<bool()> activate_pneumatics(bool raised, PneumaticCylinder &lift);
    std::function<bool()> waitUntilPnuematics(double delay_secs, PneumaticCylinder &cylinder, bool active);
    std::function<bool()> goToPose(Pose p, SwerveDrive &drive);

    AutoBuilder getAutoRightSide();
    AutoBuilder getTestAuto();
    AutoBuilder getAutoLeftSide(SwerveDrive &drive, Odometry &odom);
    
};
#endif