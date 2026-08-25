#include <Arduino.h>
#include "driver/gpio.h"

// =====================
// Encoder
// =====================
constexpr int ENCODER_A = 7;
constexpr int ENCODER_B = 6;

// AMT102-V
// 512 PPR × 2逓倍 = 1024 count/rev
constexpr float CPR = 1024.0f;

// RPM計測周期
constexpr uint32_t SAMPLE_MS = 100;

volatile int32_t encoder_count = 0;
portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;


// =====================
// Encoder ISR
// =====================
void ARDUINO_ISR_ATTR encoderISR()
{
    bool a = gpio_get_level((gpio_num_t)ENCODER_A);
    bool b = gpio_get_level((gpio_num_t)ENCODER_B);

    portENTER_CRITICAL_ISR(&encoder_mux);

    if (a != b) {
        encoder_count++;
    } else {
        encoder_count--;
    }

    portEXIT_CRITICAL_ISR(&encoder_mux);
}


// =====================
// Setup
// =====================
void setup()
{
    Serial.begin(115200);

    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A),
        encoderISR,
        CHANGE
    );

    Serial.println("RPM measurement start");
}


// =====================
// Loop
// =====================
void loop()
{
    static uint32_t last_time = millis();
    static int32_t last_count = 0;

    uint32_t now = millis();

    if (now - last_time >= SAMPLE_MS)
    {
        int32_t current_count;

        portENTER_CRITICAL(&encoder_mux);
        current_count = encoder_count;
        portEXIT_CRITICAL(&encoder_mux);

        int32_t diff = current_count - last_count;

        float dt = (now - last_time) / 1000.0f;

        float rpm =
            ((float)diff / CPR)
            * (60.0f / dt);

        Serial.printf(
            "RPM: %.1f  | count: %ld\n",
            fabs(rpm),
            (long)diff
        );

        last_count = current_count;
        last_time = now;
    }
}