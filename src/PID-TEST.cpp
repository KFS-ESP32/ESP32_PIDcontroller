/*
 * Copyright (C) 08/10/2024 Karlfried S.
 */
#include "PID-TEST.h"
#include "MessageOutput.h"
#include <Arduino.h>


PIDtestClass PIDtest;

PIDtestClass::PIDtestClass()

    : _loopTask(TASK_IMMEDIATE, TASK_FOREVER, std::bind(&PIDtestClass::loop, this))

{
}



void PIDtestClass::init(Scheduler &scheduler)
{
    Counter = 0;
    PID1.setPIDvalues(false, 1.0, 0.0, 0.0);
    PID2.setPIDvalues(false, 2.0, 0.0, 0.0);

    _pidB = PIDClass();

    // _pidA = PIDClass(true, 2.5, 2.0, 0.25, 0.02);
    _pidA = PIDClass(true, 1.1, 0.5, 0.0, 0.02); // PV-Test
    _pidA.setOutputLimits(-1000, +1000);
    _pidA.setIntLimits(-100, +100);

    _pidV2c = new PIDClass(); // dont't forget --> delete _pidV2c


    scheduler.addTask(_loopTask);
    _loopTask.enable();
}

void PIDtestClass::loop()
{
		PIDtest(true);
}

bool PIDtestClass::PIDtest(bool test)
{
    #define TickIntervalMS 100 // 100 ms
    #define AMOUNT_TRACE_POINTS 256
    #define AMOUNT_SET_POINTS 6
    bool SystemTick = false;
    static  unsigned long SystemTickes;
    static int SetpiontIndex = 0;
    static float SetPointX[AMOUNT_SET_POINTS] = {10, 100, 50, -100, -200, 0};
    static float SetPointTrace[AMOUNT_TRACE_POINTS] = {};
    static float FeedbackTrace[AMOUNT_TRACE_POINTS] = {};
    static float OutputTrace[AMOUNT_TRACE_POINTS] = {};
    unsigned long tS = millis();

    // System-Tickes
    if (tS - SystemTickes >= TickIntervalMS)
    {
        SystemTickes = tS;
        SystemTick = true;
    }

    // simuOutput1 = PID1.Calculate(100, simuFeedback1);
    // simuOutput2 = PID2.Calculate(100, simuFeedback1);

    // _pidA.Calculate(0, 0);
    // _pidB.Calculate(0, 0);
    // _pidV2c->Calculate(0, 0);

    // Setpoint-Simulation =========================================================
    if (SystemTick == true)
    {
        SetpiontIndex++;

        int Index = (SetpiontIndex / (AMOUNT_TRACE_POINTS/AMOUNT_SET_POINTS)) % AMOUNT_SET_POINTS;
        simuSetpoint_pidA = SetPointX[Index];

        // Trace-Values
        SetPointTrace[SetpiontIndex % AMOUNT_TRACE_POINTS] = simuSetpoint_pidA;
    }


    // PID-Controller-Simulation ===================================================
    if (SystemTick == true)
    {
        simuOutput_pidA = _pidA.Calculate(simuSetpoint_pidA, simuFeedback_pidA);

        // Trace-Values
        OutputTrace[SetpiontIndex % AMOUNT_TRACE_POINTS] = simuOutput_pidA;
    }

    // Feedback-Simulation =========================================================
     if (SystemTick == true)
    {
        simuFeedback_pidA = FeedbackSimulation(simuOutput_pidA);

        // Trace-Values
        FeedbackTrace[SetpiontIndex % AMOUNT_TRACE_POINTS] = simuFeedback_pidA;
    }


    if (SetpiontIndex % AMOUNT_TRACE_POINTS == 0)
    {
        MessageOutput.println("-----------------");
        MessageOutput.println("Time;Setpoint;Output;Feedback;PosError");
        for (int i = 0; i < AMOUNT_TRACE_POINTS; i++)
        {
            MessageOutput.printf("01/01/2026 00:00:00.%03d;%6.2f;%6.2f;%6.2f;%6.2f\n\r", i, SetPointTrace[i], OutputTrace[i], FeedbackTrace[i], SetPointTrace[i] - FeedbackTrace[i]);
        }
        MessageOutput.println("------------------");
    }

    // Debug-Values
    if (SystemTick == true)
    {
        MessageOutput.printf("%6.2f;%6.2f;%6.2f;%6.2f\n\r", simuSetpoint_pidA, simuOutput_pidA, simuFeedback_pidA, simuSetpoint_pidA - simuFeedback_pidA);
    }
	return true;
}

// Feedback-Simulation
float PIDtestClass::FeedbackSimulation(float Input)
{
    #define OUTPUT_SCALING (1.1)
    static float Feedback = 0.0f;
    const float TickTime = TickIntervalMS / 1000.0f;

    // Calc new Feedback-Value
    Feedback = Feedback + (TickTime * Input) * OUTPUT_SCALING;

    return Feedback;
}

