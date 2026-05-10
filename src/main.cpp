/*
 * Copyright (C) 22/02/2024 Karlfried S.
 */
#include "defaults.h"
#include "Scheduler.h"
#include "LedBlinker.h"
#include "MessageOutput.h"
#include "PID-TEST.h"

#include <Arduino.h>
#include <TaskScheduler.h>


// Life counter
uint _lifeCounter;
unsigned long _lastCall;

void setup()
{
    // Initialize serial output
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Hello Arduino......................................\n");
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
    delay(100);
#else
    while (!Serial)
        yield();
#endif

    // LED-Blinker
    LedBlinker.StartupBlinkIntervall();
 
 	// Message-Output
    MessageOutput.init(scheduler);
    MessageOutput.println();
    MessageOutput.println("Starting PID-Controller.....");

    // LED Blinker
    LedBlinker.init(scheduler);

    // Initialize WiFi
    MessageOutput.print("Initialize Network... ");
    //NetworkSettings.init(scheduler);
    MessageOutput.println("done");    
    //NetworkSettings.applyConfig();

    // Initialize PID
    MessageOutput.print("Initialize PIDcontroller... ");
    PIDtest.init(scheduler);
    MessageOutput.println("done");    
 
}


void loop()
{
    scheduler.execute();

    // LifeCouter
    _lifeCounter++;

    if (millis() - _lastCall > 10000)
    {
        MessageOutput.printf("Main: Timestamp / LifeCounter: %lu / %d\r\n", _lastCall, _lifeCounter);

        _lastCall = millis();
    }
}
