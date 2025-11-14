#include <cmath>
#ifndef PIDCONTROLLER //I think this needs to be added to be used in multiple files
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

    double m_period = 0.02;
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

    PIDController(double P_val, double I_val, double D_val)
    {
        kP = P_val;
        kI = I_val;
        kD = D_val;
    }

    PIDController(double P_val, double I_val, double D_val, double I_zone_val, double max_I_val, double min_I_val)
    {
        kP = P_val;
        kI = I_val;
        kD = D_val;
        I_zone = I_zone_val;
        max_I = max_I_val;
        min_I = min_I_val;
    }

    void setMaxMinI(double max_I_val, double min_I_val)
    {
        // maximum and minimum integral values to prevent excessive oscillation
        max_I = max_I_val;
        min_I = min_I_val;
    }

    void setIzone(double I_zone_val)
    {
        // I zone should be greater than 0
        // Zone in which integral will be applied
        I_zone = I_zone_val;
    }

    void setErrorTolerance(double error_val)
    {
        // tolerance to the setpoint value
        error_tolerance = error_val;
    }

    void setPeriod(double period)
    {
        // use this method to set the cycle times on the PID controller.
        m_period = period;
    }

    // continuous input: useful for rotation controller (when the values can wrap around)
    void enableContinuousInput(double minimumInput, double maximumInput)
    {
        m_continuous = true;
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    void disableContinuousInput()
    {
        m_continuous = false;
    }

    void setSetpoint(double setpoint)
    {
        m_setpoint = setpoint;

        // NEED TO DO ENABLE CONTINUOUS

        // m_haveSetpoint = true;
    }

    double calculate(double measurement, double setpoint)
    {
        setSetpoint(setpoint);

        return calculate(measurement);
    }

    double calculate(double measurement)
    {
        m_measurement = measurement;
        m_prevError = m_error;
        // m_haveMeasurement = true;

        if (m_continuous)
        {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            m_error = inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
        }
        else
        {
            m_error = m_setpoint - m_measurement;
        }

        m_errorDerivative = (m_error - m_prevError) / m_period;

        // If the absolute value of the position error is greater than I_zone, reset the total error
        if (std::abs(m_error) > I_zone)
        {
            m_totalError = 0;
        }
        else if (kI != 0)
        {
            m_totalError =
                clamp(
                    m_totalError + m_error * m_period,
                    min_I / kI,
                    max_I / kI);
        }

        return kP * m_error + kI * m_totalError + kD * m_errorDerivative;
    }

    //These can be private methods
    //Used to clamp val between min and max
    double clamp(double val, double min, double max)
    {
        double value = val;
        if (value > max)
        {
            value = max;
        }
        else if (value < min)
        {
            value = min;
        }
        return value;
    }
    //Used to keep the input val between min and max
    double inputModulus(double val, double min, double max)
    {
        double value = val;
        double modulus = max - min;

        int numMax = (int)((value - min) / modulus); 
        //casting to int should round down to nearest int (if it works the same as in java)
        value -= numMax * modulus;

        int numMin = (int)((value - max) / modulus); 
        //casting to int should round down to nearest int (if it works the same as in java)
        value -= numMin * modulus;

        return value;
    }
};
#endif