#ifndef SWERVEMODULECALC
#define SWERVEMODULECALC

#include <vector>
#include <utility>

class SwerveDriveCalculator {
    private:
        std::vector<std::pair<double,double>> mod_positions;
    public:

        const double MAXIMUM_SPEED = 100.0;
        SwerveDriveCalculator(const std::pair<double,double> fr, const std::pair<double,double> fl, const std::pair<double,double> br, const std::pair<double,double> bl);

        std::vector<std::pair<double, double>> calculate_drive_states(double velocity_x, double velocity_y, double rotation);
};
#endif