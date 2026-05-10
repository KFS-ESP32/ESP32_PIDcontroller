/*
 * Copyright (C) 03/02/2024 Karlfried S.
 */
#include "LedBlinker.h"
#include <Arduino.h>


LedBlinkerClass LedBlinker;

LedBlinkerClass::LedBlinkerClass()

    : _loopTask(BLINK_INTERVALL * TASK_MILLISECOND, TASK_FOREVER, std::bind(&LedBlinkerClass::loop, this))

{
}

void SetPinMode()
{
    // blue LED
    pinMode(BUILTIN_LED_PIN, OUTPUT); // LED
}

void LedBlinkerClass::init(Scheduler &scheduler)
{
    // blue LED
    SetPinMode();

    scheduler.addTask(_loopTask);
    _loopTask.enable();
}

void LedBlinkerClass::loop()
{
    // LED-Blinker, wird alle BLINK_INTERVALL aufgerufen
    if (LED_state == true)
    {
        digitalWrite(BUILTIN_LED_PIN, LOW);
        LED_state = false;
    }
    else
    {
        digitalWrite(BUILTIN_LED_PIN, HIGH);
        LED_state = true;
    }
}

void LedBlinkerClass::StartupBlinkIntervall()
{
    SetPinMode();

    // LED-Blinker
    for (int i = 0; i < 10; i++)
    {
        digitalWrite(BUILTIN_LED_PIN, HIGH);
        delay(100);
        digitalWrite(BUILTIN_LED_PIN, LOW);
        delay(100);
    }
}
