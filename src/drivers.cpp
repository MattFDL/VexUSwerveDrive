

#include "drivers.h"

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
            
        void handleInputs(){
            switch (user){
                case 0:
                    preset_RC();
                    break;
                case 1:
                    preset_tank();
                    break;
            }
        }
    private:
        axis left_Y = 0;
        axis left_X = 0;
        axis right_Y = 0;
        axis right_X = 0;

        bool right_bumper;
        bool right_trigger;

        bool range(int val, int min, int max) {
            return (val >= min && val <= max);
        }
        int32_t deadzoneCalc(int32_t input, int16_t deadzone) {
            if (range(input, -deadzone, deadzone)) {
                return 0;
            } else {
                return input;
            }
        }
        int32_t throttleCurve(int32_t input) {
            // Implement a simple quadratic throttle curve
            int32_t output = (input)*(pow(input/127.0,2));
            return output;
        }
        void preset_RC(){
                                         // Run for 20 ms then update
            left_Y=controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            left_X=controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            right_X=controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            right_bumper=controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
            right_trigger=controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

            if (right_bumper) {
                intakeMotors.move(127);
            } else if (right_trigger) {
                intakeMotors.move(-127);
            } else {
                intakeMotors.brake();
            }


            int32_t forward = left_Y;
            int32_t turn;
            
            if(abs(left_X) > abs(right_X)) {
                turn = left_X;
            } else {
                turn = right_X;
            }
            

            forward=deadzoneCalc(forward, deadzone);
            turn=deadzoneCalc(turn, deadzone);
            

            int32_t left=forward+(turn*.8);
            int32_t right=forward-(turn*.8);
            if (abs(left)>0){leftMotors.move(left);}
            else {leftMotors.brake();}
            if (abs(right)>0){rightMotors.move(right);}
            else {rightMotors.brake();}
        }
        void preset_tank(){

        }
    };