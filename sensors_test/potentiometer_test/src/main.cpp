#include <Arduino.h>

// ========================================
// Potentiometer pins
// ========================================
constexpr int POT_GET_1_PIN = 12;
constexpr int POT_GET_2_PIN = 26;
constexpr int POT_LIFT_PIN  = 35;

// 出力周期
constexpr uint32_t PRINT_INTERVAL_MS = 50;

// 平均化サンプル数
// ポテンショメータの値の揺れを見るため軽く平均化
constexpr int SAMPLE_COUNT = 10;


// ========================================
// ADC読み取り
// ========================================
int readPotAverage(int pin)
{
    uint32_t sum = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        sum += analogRead(pin);
    }

    return sum / SAMPLE_COUNT;
}


// ========================================
// setup
// ========================================
void setup()
{
    Serial.begin(115200);

    // ESP32 ADCを12bitに設定
    // 0 ～ 4095
    analogReadResolution(12);

    pinMode(POT_GET_1_PIN, INPUT);
    pinMode(POT_GET_2_PIN, INPUT);
    pinMode(POT_LIFT_PIN, INPUT);

    Serial.println();
    Serial.println("Potentiometer Test Start");
    Serial.println("GET1, GET2, LIFT");
}


// ========================================
// loop
// ========================================
void loop()
{
    static uint32_t last_print_ms = 0;

    if (millis() - last_print_ms >= PRINT_INTERVAL_MS)
    {
        last_print_ms = millis();

        int get1 = readPotAverage(POT_GET_1_PIN);
        int get2 = readPotAverage(POT_GET_2_PIN);
        int lift = readPotAverage(POT_LIFT_PIN);

        Serial.printf(
            "GET1:%4d  GET2:%4d  LIFT:%4d\n",
            get1,
            get2,
            lift
        );
    }
}