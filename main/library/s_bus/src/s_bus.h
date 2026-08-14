#pragma once

#include <Arduino.h>

namespace SBus
{
    void begin();
    void update();

    uint16_t getChannel(uint8_t ch);

    bool isConnected();
}