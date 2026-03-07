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
#include "intake.h"

class Autos {
    public:

    Autos();
    const Pose noMatchStart = Pose(0, 0, 0);
    const Pose noMatch1 = Pose(42, -13, 0);
    const Pose noMatch2 = Pose(52, -22, -30);
    const Pose noMatch3 = Pose(54.0, -27, -30);
    const Pose noMatch4 = Pose(74.5, 2, -45.0);

    const Pose matchLoadStart = Pose(0, 0, 180);
    const Pose matchLoad1 = Pose(18, 30, 180); //Match Load mech should be dropped and intake running here
    const Pose matchLoad2 = Pose(5, 30, 180); //This is the match load
    const Pose matchLoad3 = Pose(30, 30, 180); //This is the score zone
    const Pose matchLoad4 = Pose(5, 30, 180); //This is the match load 
    //Make sure to time this correctly so we dont score the opposite color
    const Pose matchLoad5 = Pose(30, 30, 180);



    std::function<bool()> waitCommand(double delay_secs);
    std::function<bool()> activate_pneumatics(bool raised, PneumaticCylinder &lift);
    std::function<bool()> waitUntilPnuematics(double delay_secs, PneumaticCylinder &cylinder, bool active);
    std::function<bool()> goToPose(Pose p, SwerveDrive &drive);
    std::function<bool()> goToPoseSlow(Pose p, SwerveDrive &drive);


    AutoBuilder getAutoRightSide();
    AutoBuilder getTestAuto();
    AutoBuilder noMatchLoadAuto(SwerveDrive &drive, Odometry &odom, Intake &intake);
    AutoBuilder matchLoadAuto(SwerveDrive &drive, Odometry &odom, Intake &intake, PneumaticCylinder &matchLoadMech);

    
};
#endif