#ifndef DRIVERS_H
#define DRIVERS_H
#include "api.h"

class driverControls{
    public:
        
        typedef int32_t axis;
        pros::Controller &controller;
        pros::MotorGroup &leftMotors;
        pros::MotorGroup &rightMotors;
        pros::MotorGroup &intakeMotors;
        int deadzone=10;
        int user=0;
        driverControls(pros::Controller &controller, pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::MotorGroup &intake):
        controller(controller), leftMotors(leftMotors), rightMotors(rightMotors), intakeMotors(intake) {}
            
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
        void preset_RC();
        void preset_tank();
    };

#endif