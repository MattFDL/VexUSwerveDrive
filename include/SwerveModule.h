#ifndef SWERVEMODULE
#define SWERVEMODULE

#include "PIDController.h" 
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <utility>

class SwerveModule {
    private:
        pros::Motor top_motor;
        pros::Motor bottom_motor;
        pros::Rotation moduleAngle;
    public:
        PIDController turn_controller = PIDController(150.0, 0, 0);
        double pos_x; //positive forward
        double pos_y; //positive left
        double module_number;
        bool reversed;
        SwerveModule(pros::Motor &t_m, pros::Motor &b_m, pros::Rotation &m_a, double position_x, double position_y, double mod_num, bool rev = false);

        void set_state(std::pair<double, double> target_state);
        
        std::pair<double, double> optimize_angle(std::pair<double, double> target_state, double current_angle);
};
#endif