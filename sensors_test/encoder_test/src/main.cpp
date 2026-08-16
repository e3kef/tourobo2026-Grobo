#include <Arduino.h>

#include "encoder.h"

constexpr int ENC_A = D5;
constexpr int ENC_B = D4;

constexpr int PWM_PIN = 20;
constexpr int DIR_PIN = 8;

constexpr int PWM_CHANNEL = 0;

constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_RESOLUTION = 10;

// 10bit: 0～1023
// 50% duty
constexpr uint16_t PWM_DUTY = 512;

void setup()
{
    Serial.begin(115200);

    Encoder_begin(ENC_A, ENC_B);

    // DIR
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH);

    // PWM設定
    ledcSetup(
        PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        PWM_PIN,
        PWM_CHANNEL
    );

    // 50% duty
    ledcWrite(
        PWM_CHANNEL,
        PWM_DUTY
    );
}

void loop()
{
    static TickType_t last_wake_time = xTaskGetTickCount();

    float rpm = Encoder_getRPM();

    Serial.print("RPM: ");
    Serial.println(rpm);

    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(500)
    );
}