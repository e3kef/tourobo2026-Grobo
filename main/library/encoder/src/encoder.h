#pragma once

#include <Arduino.h>

void Encoder_begin(int pin_a, int pin_b);

int32_t Encoder_getCount();
float Encoder_getRPM(float dt);