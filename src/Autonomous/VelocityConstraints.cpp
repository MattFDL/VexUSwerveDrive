#include <cmath>
#ifndef VELOCITYCONSTRAINTS //I think this needs to be added to be used in multiple files
#define VELOCITYCONSTRAINTS
class VelocityContraints
{
public:
    double acceleration;
    double max_velocity;
    VelocityContraints(double accel, double max_vel) : acceleration(accel), max_velocity(max_vel) {


    }

    double getCurrentVelocity(double time, double dist) {
        double time_constant_beginning = max_velocity/acceleration;
        double time_constant_velocity = (dist/max_velocity) - (max_velocity / acceleration);
        if (time_constant_velocity > 0) {//trapazoid
            if (time < time_constant_beginning) {
                return (acceleration*time);
            } else if (time > (time_constant_beginning+time_constant_velocity))
            {
                return (max_velocity-(time*acceleration));
            } else {
                return max_velocity;
            }
        } else { //triangle
            double total_time = 2.0 * std::sqrt(dist/acceleration);
            if (time <= (total_time/2)) {
                return time * acceleration;
            } else {
                return ((total_time * 0.5 * acceleration) - ((time - (total_time * 0.5)) * acceleration));
            }
        }
    }


    
};
#endif