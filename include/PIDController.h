#ifndef PIDCONTROLLER
#define PIDCONTROLLER

class PIDController
{
public:
    double kP;
    double kI;
    double kD;

    double I_zone = 100;
    double max_I = 1.0;
    double min_I = -1.0;

    double error_tolerance = 0.05;

    double m_period = 0.01;
    // TODO: might need double error_derivative; ...

    double m_setpoint;
    double m_measurement;
    double m_prevError = 0;
    double m_error = 0;
    double m_totalError = 0;
    double m_errorDerivative = 0;

    bool m_continuous = false;
    double m_minimumInput;
    double m_maximumInput;
    PIDController(double P_val, double I_val, double D_val);
    PIDController(double P_val, double I_val, double D_val, double I_zone_val, double max_I_val, double min_I_val);

    void setMaxMinI(double max_I_val, double min_I_val);
    void setIzone(double I_zone_val);
    void setErrorTolerance(double error_val);
    bool atTolerance();
    void setPeriod(double period);
    void enableContinuousInput(double minimumInput, double maximumInput);
    void disableContinuousInput();
    void setSetpoint(double setpoint);
    double calculate(double measurement, double setpoint);
    double calculate(double measurement);
};

#endif