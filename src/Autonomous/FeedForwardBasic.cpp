#include <cmath>
#ifndef FEEDFORWARD //I think this needs to be added to be used in multiple files
#define FEEDFORWARD

class FeedForwardBasic {
    public:
    double kS;
    double kV;
    double kA;

    FeedForwardBasic(double kStaticVoltage, double kVoltsPerInchesPerSecond, double kVoltsPerInchesPerSecondSquared) : 
    kS(kStaticVoltage), 
    kV(kVoltsPerInchesPerSecond), 
    kA(kVoltsPerInchesPerSecondSquared)
    {}

   double calculateVolts(double targetInPerSec) { //incorperate kA into this later
        if (targetInPerSec < 0) {
            double volts = (-kS + targetInPerSec * kV);
            if (volts < -12) {
                volts = -12;
            }
            return volts;
        }
        double volts = (kS + targetInPerSec * kV);
        if (volts > 12) {
            volts = 12;
        }
        return volts;
    }

    double simpleCalculateVolts(double targetInPerSec) {
        return ((targetInPerSec / 76.6) * 12);
    }

};
#endif