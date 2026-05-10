// This is an automated pid controller.
// You can use the PID object for a simple pid controller or
// use pid controller object to create multiple that run simultaniousely.
// ======================================================================
/*
 * PID Controller implementation
 *
 * Features:
 *  - Proportional, Integral, Derivative control
 *  - Anti-windup via integrator clamping
 *  - Band-limited differentiator (derivative on measurement(Feedback) or on Error)
 *
 * Suitable for embedded and real-time systems.
 *
 *      * A PID controller with derivative on measurement (Feedback) (also known as derivative on process variable,
        * or D-on-PV) calculates the derivative component based solely on the actual
        * measured process variable (PV) rather than the error (Setpoint - PV).
        * This technique eliminates "derivative kick," preventing harsh output spikes when the setpoint changes abruptly,
        * allowing for tighter control without instability.
*/
// Based on: https://github.com/christosneg/concurrentPID/tree/master
// and : https://github.com/pms67/PID
// and: https://www.youtube.com/watch?v=zOByx3Izf5U
// ======================================================================

#pragma once

#include <Arduino.h>

//#define NO_SERIAL_DEBUG_PID // No Serial debug outputs

// TODO: Feedback-Delay (Istwertverzögerung, wie bei Beckhoff=Totzeitkompensation) dazufügen
//       Ausgangsskalierung dazufügen (wie bei Beckhoff)
//       Vorsteuergeschwindigkeit + Kv · Schleppabstand


// PID-Parameters
// ==============
typedef struct
{
    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float Tau;

    /* Differentiator-Mode: 1) Derivative on measurement(Feedback) 2) Derivative on Error */
    bool DifferentiatorModeERROR = false;

    float Setpoint; // [unit]
    float Feedback; // [unit]

    /* Output limits */
    float limMinOut; // [unit]
    float limMaxOut; // [unit]

    /* Integrator limits */
    float limMinInt; // [unit * sec]
    float limMaxInt; // [unit * sec]

    /* Sample time (in seconds) */
    float T; // [sec]

    /* Controller "memory" */
    unsigned long prevT; // [ms]
    float integrator; // [unit * sec]
    float prevIntegrator; // [unit * sec]
    float prevError;  // [unit]	-  Required for integrator
    float differentiator;  // [unit]
    float prevFeedback; // [unit] -  Required for differentiator

    /* Controller output */
    float Out; // [unit]

} PIDController;

class PIDClass
{
public:

    PIDClass();
    PIDClass(bool Dmode, float Kp, float Ki, float Kd, float Tau);

    // Setter functions
    void setPIDvalues(bool Dmode, float Kp, float Ki, float Kd);  // Set PID values
    void setOutputLimits(float Min, float Max); // Output limits
    void setIntLimits(float Min, float Max); // Integrator limits
    void setTarget(float Setpoint);
    float Calculate(float Setpoint, float Feedback); // Use pid equation to calculate output
    bool isTargetReached(float Tolerance); // Check if the error is below tolarance

    // Getter functions
    float getTarget() { return pidP.Setpoint; }
    float getP() { return pidP.Kp; }
    float getD() { return pidP.Kd; }
    float getI() { return pidP.Ki; }

private:

    PIDController pidP = {}; // Parameters

};

extern PIDClass PID;

