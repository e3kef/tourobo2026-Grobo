#pragma once

#include <stdint.h>

bool CAN_begin(int txPin, int rxPin);

bool CAN_send(uint16_t id, const int16_t data[4]);

bool CAN_receive(uint16_t &id, int16_t data[4], uint32_t timeout = 0);