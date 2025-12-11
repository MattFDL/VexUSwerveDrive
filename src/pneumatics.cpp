#include "pneumatics.h"





// Extend (energize solenoid)
void PneumaticCylinder::extend() {
    state = true;
    solenoid.set_value(true);  // HIGH/1 [web:38]
}

// Retract (de‑energize solenoid)
void PneumaticCylinder::retract() {
    state = false;
    solenoid.set_value(false); // LOW/0 [web:38]
}

// Toggle current state
void PneumaticCylinder::toggle() {
    state = !state;
    solenoid.set_value(state);
}

// Query state (true = extended)
bool PneumaticCylinder::is_extended() {
    return state;
}