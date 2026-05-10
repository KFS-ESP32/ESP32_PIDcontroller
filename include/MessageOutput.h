// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <HardwareSerial.h>
#include <Stream.h>
#include <TaskSchedulerDeclarations.h>
#include <mutex>

#define BUFFER_SIZE 500

class MessageOutputClass : public Print {

// using Print::Print;   // Inheriting constructors

public:
    MessageOutputClass();
    void init(Scheduler& scheduler);
    size_t write(uint8_t c) override;  // Overriding base functionality
    size_t write(const uint8_t* buffer, size_t size) override;  // Overriding base functionality

private:
    void loop();

    Task _loopTask;

    char _buffer[BUFFER_SIZE];
    uint16_t _buff_pos = 0;
    uint32_t _lastSend = 0;
    bool _forceSend = false;

#if defined(ESP32)
    #define HARDWARE "ESP32"
    std::mutex _msgLock;
#endif

};

extern MessageOutputClass MessageOutput;
