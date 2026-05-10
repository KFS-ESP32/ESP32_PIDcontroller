# ESP32_PIDcontroller-LIB
PID Controller for EPS32

## Based on:

https://github.com/christosneg/concurrentPID/tree/master

https://github.com/pms67/PID

https://www.youtube.com/watch?v=zOByx3Izf5U



## PID Controller implementation
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


## Remark

In progress...
