#include "PIDController.h"
#include <cmath>
#include "CommonUtility.h"

PIDController::PIDController(double P_val, double I_val, double D_val)
{
    kP = P_val;
    kI = I_val;
    kD = D_val;
}

PIDController::PIDController(double P_val, double I_val, double D_val, double I_zone_val, double max_I_val, double min_I_val)
{
    // alternative PID controller set up for I zone and max/min integral
    kP = P_val;
    kI = I_val;
    kD = D_val;
    I_zone = I_zone_val;
    max_I = max_I_val;
    min_I = min_I_val;
}

void PIDController::setMaxMinI(double max_I_val, double min_I_val)
{
    // maximum and minimum integral values to prevent excessive oscillation
    max_I = max_I_val;
    min_I = min_I_val;
}

void PIDController::setIzone(double I_zone_val)
{
    // I zone should be greater than 0
    // Zone in which integral will be applied
    I_zone = I_zone_val;
}

void PIDController::setErrorTolerance(double error_val)
{
    // tolerance to the setpoint value
    error_tolerance = error_val;
}

bool PIDController::atTolerance()
{
    // get the tolerance of the PID
    return (std::abs(m_error) < error_tolerance);
}

void PIDController::setPeriod(double period)
{
    // use this method to set the cycle times on the PID controller.
    m_period = period;
}

// continuous input: useful for rotation controllers (when the values can wrap around)
void PIDController::enableContinuousInput(double minimumInput, double maximumInput)
{
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
}

void PIDController::disableContinuousInput()
{
    m_continuous = false;
}

void PIDController::setSetpoint(double setpoint)
{
    m_setpoint = setpoint;
}

// calculate and set the setpoint in one function
double PIDController::calculate(double measurement, double setpoint)
{
    setSetpoint(setpoint);

    return calculate(measurement);
}

// calculate the needed feedback
double PIDController::calculate(double measurement)
{
    m_measurement = measurement;
    m_prevError = m_error;

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
                min_I,
                max_I);
    }
    if (std::abs(m_error) < error_tolerance)
    {
        return 0;
    }
    return kP * m_error + kI * m_totalError + kD * m_errorDerivative;
}
