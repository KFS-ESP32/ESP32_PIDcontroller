/*
 * Copyright (C) 03/02/2024 Karlfried S.
 */

#define BLINK_INTERVALL 250 // [ms] --> 250ms

#include <TaskSchedulerDeclarations.h>

const int BUILTIN_LED_PIN = 2; // blue LED


class LedBlinkerClass
{
public:
    LedBlinkerClass();
    void init(Scheduler& scheduler);
    void StartupBlinkIntervall();

private:
    void loop();

    Task _loopTask;
    bool LED_state;

};

extern LedBlinkerClass LedBlinker;
