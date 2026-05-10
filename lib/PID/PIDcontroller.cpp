#include "PIDcontroller.h"
//#include "../../include/MessageOutput.h"

// *************************** PID class *********************** //

PIDClass PID;

PIDClass::PIDClass()
{
    pidP.Kp = 0.5;
    pidP.Ki = 0;
    pidP.Kd = 0;
    pidP.Tau = 0;
    Calculate(0, 0);
}

PIDClass::PIDClass(bool Dmode, float Kp, float Ki, float Kd, float Tau)  // Set PID values
{
    pidP.DifferentiatorModeERROR = Dmode;
    pidP.Kp = Kp;
    pidP.Ki = Ki;
    pidP.Kd = Kd;
    pidP.Tau = Tau;
    Calculate(0, 0);
}

void PIDClass::setPIDvalues(bool Dmode, float Kp, float Ki, float Kd)
{
    pidP.DifferentiatorModeERROR = Dmode;
    pidP.Kp = Kp;
    pidP.Ki = Ki;
    pidP.Kd = Kd;
    pidP.Setpoint = 0;
}

void PIDClass::setOutputLimits(float Min, float Max)
{
    /* Output limits */
    pidP.limMinOut = Min;
    pidP.limMaxOut = Max;
}

void PIDClass::setIntLimits(float Min, float Max)
{
    /* Integrator limits */
    pidP.limMinInt = Min;
    pidP.limMaxInt = Max;
}

void PIDClass::setTarget(float Setpoint)
{
    pidP.Setpoint = Setpoint;
}

bool PIDClass::isTargetReached(float Tolerance)
{
    // Check if the Feedback (current state) is within tolerance of the Setpoint
    return fabs(pidP.Setpoint - pidP.Feedback) <= Tolerance;
}

// Use pid values to calculate output
// Units = W, mm and so on..
float PIDClass::Calculate(float Setpoint, float Feedback)
{
    // PID-Algorithmus von: https://github.com/pms67/PID
    // =================================================

    bool isOutPutLimit = false;
    float proportional = 0;
    pidP.prevIntegrator = pidP.integrator;

    pidP.Setpoint = Setpoint;
    pidP.Feedback = Feedback;

    /*
     * Sample time (in seconds)
    */
    unsigned long t = millis();
    // millis() rollover is no problem!
    pidP.T = (t - pidP.prevT) / 1000.0f; // [sec]
    pidP.prevT = t;

    /*
    * Error signal
    */
    float error = pidP.Setpoint - pidP.Feedback;

    /*
    * Proportional
    */
    if (pidP.Kp != 0)
        proportional = pidP.Kp * error;

    /*
    * Integral
    */
    if (pidP.Ki != 0)
    {
        pidP.integrator = pidP.integrator + 0.5f * pidP.Ki * pidP.T * (error + pidP.prevError);

        /* Anti-wind-up via integrator clamping */
        if (pidP.integrator > pidP.limMaxInt) {

            pidP.integrator = pidP.limMaxInt;

        } else if (pidP.integrator < pidP.limMinInt) {

            pidP.integrator = pidP.limMinInt;

        }
    }

    /*
    * Derivative (band-limited differentiator)
    */
    /* Note: derivative on measurement, therefore minus sign in front of equation! */
    if (pidP.Kd != 0)
    {
        if (pidP.DifferentiatorModeERROR == false)
        {
            /*
             * Hier wird nur auf den Istwert (Feedback) geschaut.
             * Wird der Istwert größer, dann wird 'differentiator' negativ.
             * Wenn sich also der Istwert aufgrund des P-Anteiles zum Sollwert bewegt,
             * dann kann dies hierüber abgemildert werden.
             *
             * A PID controller with derivative on measurement (Feedback) (also known as derivative on process variable,
             * or D-on-PV) calculates the derivative component based solely on the actual
             * measured process variable (PV) rather than the error (Setpoint - PV).
             * This technique eliminates "derivative kick," preventing harsh output spikes when the setpoint changes abruptly,
             * allowing for tighter control without instability.
             */
            // Derivative on measurement
            pidP.differentiator = (-(2.0f * pidP.Kd * (pidP.Feedback - pidP.prevFeedback))
                                + (2.0f * pidP.Tau - pidP.T) * pidP.differentiator)
                                / (2.0f * pidP.Tau + pidP.T);
        }
        else
        {
            // Derivative on error
            /* alt..
            pidP.differentiator = (-(2.0f * pidP.Kd * (error - pidP.prevError))
                        + (2.0f * pidP.Tau - pidP.T) * pidP.differentiator)
                        / (2.0f * pidP.Tau + pidP.T);
            */
            pidP.differentiator =  pidP.Kd * (error - pidP.prevError) / pidP.T;
        }
    }

    /*
    * Compute output and apply limits
    */
    pidP.Out = proportional + pidP.integrator + pidP.differentiator;

    // Debug
    #ifndef NO_SERIAL_DEBUG_PID
        if (Serial== true)
        {
            Serial.print("PDI: T=");
            Serial.print(pidP.T);
            Serial.print(" P=");
            Serial.print(proportional);
            Serial.print(" I=");
            Serial.print( pidP.integrator );
            Serial.print(" D=");
            Serial.print(pidP.differentiator);
            Serial.print(" E=");
            Serial.print(error);
            Serial.println();
        }
    #endif

    if (pidP.Out > pidP.limMaxOut)
    {
        pidP.Out = pidP.limMaxOut;
        isOutPutLimit = true;

    }
    else if (pidP.Out < pidP.limMinOut)
    {
        pidP.Out = pidP.limMinOut;
        isOutPutLimit = true;
    }

    // If OutPut is limiting then 'integrator' will not change unless 'Out' is being not clamped
    if ( isOutPutLimit == true)
    {
        pidP.integrator = pidP.prevIntegrator;
    }

    /* Store error and measurement for later use */
    pidP.prevError       = error;
    pidP.prevFeedback = pidP.Feedback;

    /* Return controller output */
    return pidP.Out;


}

