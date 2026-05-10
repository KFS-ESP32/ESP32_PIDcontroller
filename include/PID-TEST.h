/*
 * Copyright (C) 08/10/2024 Karlfried S.
 */
#pragma once

#include <TaskSchedulerDeclarations.h>
#include "PIDcontroller.h"

class PIDtestClass
{
public:
    PIDtestClass();
    void init(Scheduler& scheduler);


private:
    void loop();
    Task _loopTask;

	bool PIDtest(bool);
    float FeedbackSimulation(float Input);

	unsigned int Counter;

    // PIDs
    PIDClass PID1;
    PIDClass PID2;

    PIDClass _pidA;
    PIDClass _pidB;
    PIDClass *_pidV2c;

    // Simulation
    float simuFeedback1;
    float simuFeedback2;
    float simuOutput1;
    float simuOutput2;

    float simuSetpoint_pidA = 0;
    float simuSetpoint_pidB = 0;
    float simuFeedback_pidA = 0;
    float simuFeedback_pidB = 0;
    float simuOutput_pidA = 0;
    float simuOutput_pidB = 0;
};

extern PIDtestClass PIDtest;
