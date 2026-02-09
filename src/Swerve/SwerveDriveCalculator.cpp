#include "SwerveDriveCalculator.h"

#include <cmath>

SwerveDriveCalculator::SwerveDriveCalculator(const std::pair<double,double> fr, const std::pair<double,double> fl, 
    const std::pair<double,double> br, const std::pair<double,double> bl) 
{
    mod_positions = {fr, fl, br, bl};
}

std::vector<std::pair<double, double>> SwerveDriveCalculator::calculate_drive_states(double velocity_x, double velocity_y, double rotation) {
    double largest_speed = 0;
    std::vector<std::pair<double, double>> module_states;
    std::vector<std::pair<double, double>> module_vectors; //x, y
    for (auto pos : mod_positions)
    { 
        double vx_module = velocity_x - (rotation * pos.second);
        double vy_module = velocity_y + (rotation * pos.first);
        double magnitude = std::sqrt(std::pow(vx_module, 2) + std::pow(vy_module, 2));
        if (magnitude > largest_speed) {
            largest_speed = magnitude;
        }
        std::pair<double, double> mod_vec(vx_module, vy_module);
        module_vectors.push_back(mod_vec);
    }

    double multiplier = 1.0;
    if (largest_speed > MAXIMUM_SPEED) {
        multiplier = (MAXIMUM_SPEED/largest_speed);
    }

    for (auto vec : module_vectors) {
        double angle = (180 * (std::atan2(vec.second*multiplier, vec.first*multiplier) / M_PI));
        double speed = std::sqrt(std::pow(vec.first*multiplier, 2) + std::pow(vec.second*multiplier, 2));
        std::pair<double, double> state(angle, speed);
        module_states.push_back(state);
    }

    return module_states;
}
