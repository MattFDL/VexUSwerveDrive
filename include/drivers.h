#ifndef DRIVERS_H
#define DRIVERS_H
#include "api.h"
#include "pneumatics.h"
class driverControls{
    public:
        
        typedef int32_t axis;
        pros::Controller &controller;
        pros::MotorGroup &leftMotors;
        pros::MotorGroup &rightMotors;
        pros::MotorGroup &intakeMotors;

        PneumaticCylinder &lifter;
        PneumaticCylinder &rake;
        PneumaticCylinder &holderFlap;
        PneumaticCylinder &Dscore;
        
        

        int deadzone=10;
        int user;
        driverControls(
            pros::Controller &controller, pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::MotorGroup &intake,//motors
            int userID,                                                                                                         //user
            PneumaticCylinder &Plifter,PneumaticCylinder &Prake,PneumaticCylinder &Pholder,PneumaticCylinder &PDscore):         //pneumatics
        controller(controller), leftMotors(leftMotors), rightMotors(rightMotors), intakeMotors(intake),
        user(userID),lifter(Plifter),rake(Prake),holderFlap(Pholder),Dscore(PDscore) {}
            
        void handleInputs();
    private:
        axis left_Y;
        axis left_X;
        axis right_Y;
        axis right_X;

        bool right_bumper;
        bool right_trigger;

        

        //bool range(int val, int min, int max);


        int32_t deadzoneCalc(int32_t input, int16_t deadzone);
        int32_t throttleCurve(int32_t input);
        void intake();
        void pneumatics();
        void preset_RC();
        void preset_tank();
    };

#endif