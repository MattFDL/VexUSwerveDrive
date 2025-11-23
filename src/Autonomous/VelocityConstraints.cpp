#include <cmath>
#ifndef VELOCITYCONSTRAINTS //I think this needs to be added to be used in multiple files
#define VELOCITYCONSTRAINTS
class VelocityContraints
{
public:
    double accerleration;
    double max_velocity;
    VelocityContraints(double accel, double max_vel) : accerleration(accel), max_velocity(max_vel) {


    }

    double getCurrentVelocity(double time, double dist) {
        double time_constant_beginning = max_velocity/accerleration;
        double time_constant_velocity = (dist/max_velocity) - (max_velocity / accerleration);
        if (time_constant_velocity > 0) {//trapazoid
            if (time < time_constant_beginning) {
                return (accerleration*time);
            } else if (time > (time_constant_beginning+time_constant_velocity))
            {
                return (max_velocity-(time*accerleration));
            } else {
                return max_velocity;
            }
        } else { //triangle
            double total_time = 2.0 * std::sqrt(dist/accerleration);
        }
    }


    
};
#endif