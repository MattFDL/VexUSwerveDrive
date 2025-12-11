#ifndef PNEUMATICS
#define PNEUMATICS

#include "pros/adi.hpp" // Used to for Digital Port Write access
#include "pros/rtos.hpp" // Included for delay if needed

class PneumaticCylinder {
private:
    pros::adi::DigitalOut solenoid;  // Create a DigitalOut Object for the solenoid
    bool state;                    // Track current state locally (true = extended)

public:
    // Constructor for a direct ADI port on the brain (A–H)
    // Must be instantaied with corresponding port letter
    explicit PneumaticCylinder(char port) 
    : solenoid(port), state(false) { // Initializes DigitalOut Object for solenoid with port letter
        retract();  // de-energize pneumatics by default
    }

    /*
    / Alternative constructor if using an ADI expander (smart port + channel 1–8)
    / NOT USED
    PneumaticCylinder(std::uint8_t smart_port, std::uint8_t channel)
        : solenoid(smart_port, channel), state(false) { 
        retract(); // de-energize pneumatics by defualt
    }
     */

    // Extend (energize solenoid)
    void extend();

    // Retract (de‑energize solenoid)
    void retract();

    // Toggle current state
    void toggle();

    // Query state (true = extended)
    bool is_extended();
};

#endif