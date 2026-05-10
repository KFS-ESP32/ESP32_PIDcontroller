// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Karlfried S. and others
 */
#include "MessageOutput.h"

#include <Arduino.h>

MessageOutputClass MessageOutput;

MessageOutputClass::MessageOutputClass()

    : _loopTask(TASK_IMMEDIATE, TASK_FOREVER, std::bind(&MessageOutputClass::loop, this))
{
}

void MessageOutputClass::init(Scheduler& scheduler)
{
    scheduler.addTask(_loopTask);
    _loopTask.enable();
}

size_t MessageOutputClass::write(uint8_t c)
{
    if (_buff_pos < BUFFER_SIZE) {

#if defined(ESP32)
        std::lock_guard<std::mutex> lock(_msgLock);
#endif

        _buffer[_buff_pos] = c;
        _buff_pos++;
    } else {
        _forceSend = true;
    }

    return Serial.write(c);
}

size_t MessageOutputClass::write(const uint8_t* buffer, size_t size)
{
    #if defined(ESP32)
    std::lock_guard<std::mutex> lock(_msgLock);
    #endif
    if (_buff_pos + size < BUFFER_SIZE) {
        memcpy(&_buffer[_buff_pos], buffer, size);
        _buff_pos += size;
    }
    _forceSend = true;

    return Serial.write(buffer, size);
}

void MessageOutputClass::loop()
{
    // Send data via websocket if either time is over or buffer is full
    if (_forceSend == true || (millis() - _lastSend > 1000))
    {
        #if defined(ESP32)
        std::lock_guard<std::mutex> lock(_msgLock);
        #endif
        /*
        if (_ws && _buff_pos > 0)
        {
            _ws->textAll(_buffer, _buff_pos);
            _buff_pos = 0;
        }
        */
        if (_forceSend == true)
        {
            _buff_pos = 0;
        }
 
        _forceSend = false;
        _lastSend = millis();

    }
}
