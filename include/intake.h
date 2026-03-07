#ifndef INTAKE
#define INTAKE
#include "pros/motors.hpp"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"
#include "pneumatics.h"

class Intake {
    public:
    pros::Motor &roller_motor;
    pros::Motor &lever_motor;
    PneumaticCylinder &lifter; 
    PneumaticCylinder &flap; 
    bool set_lever = false; 
    bool reset_lever = false;
    bool is_lifted = false; 
    uint32_t previousTime = pros::millis();
    int32_t largestCurrentSpike = 0;
    Intake(pros::Motor &m_rollers, pros::Motor &m_lever, PneumaticCylinder &m_lifter, PneumaticCylinder &m_flap);
    void lift_lever();
    void lower_lever();
    void run_rollers();
    void reverse_rollers();
    void stop_rollers();
    void handle_intake(); //This method should be running continuously in tele
    void toggle_lift();
    void full_send(double num);
};
#endif