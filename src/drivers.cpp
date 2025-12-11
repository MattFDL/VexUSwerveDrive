

#include "drivers.h"




void driverControls::handleInputs(){
    switch (user){
        case 0:
            preset_RC();
            break;
        case 1:
            preset_tank();
            break;
    }
    intake();
    pneumatics();
}
void driverControls::pneumatics(){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){rake.toggle();}
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){holderFlap.toggle();}
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){lifter.toggle();}
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){Dscore.toggle();}
}
int32_t driverControls::deadzoneCalc(int32_t input, int16_t deadzone) {
    if (input >= -deadzone && input <= deadzone) {
        return 0;
    } else {
        return input;
    }
}
int32_t driverControls::throttleCurve(int32_t input) {
    // Implement a simple quadratic throttle curve
    int32_t output = (input)*(pow(input/127.0,2));
    return output;
}
void driverControls::intake(){
    right_bumper=controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    right_trigger=controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    if (right_bumper) {
        intakeMotors.move(127);
    } else if (right_trigger) {
        intakeMotors.move(-127);
    } else {
        intakeMotors.brake();
    }
}
void driverControls::preset_RC(){
                                    // Run for 20 ms then update
    left_Y=controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    left_X=controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    right_X=controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);



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
void driverControls::preset_tank(){
    
    int32_t forwardLeft=controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int32_t forwardRight=controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    forwardLeft=deadzoneCalc(forwardLeft, deadzone);
    forwardRight=deadzoneCalc(forwardRight, deadzone);
    
    leftMotors.move(throttleCurve(forwardLeft));       // Set left motor group to left joystick value
    rightMotors.move(throttleCurve(forwardRight));     // Set right motor group to right joystick value
}