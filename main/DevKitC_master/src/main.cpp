#include <Arduino.h>

#include "s_bus.h"
#include "can.h"
#include "can_protocol.h"

void taskSBus(void *arg);
void taskControl(void *arg);

constexpr int tx_pin = 21;
constexpr int rx_pin = 22;

void setup()
{
    Serial.begin(115200);

    SBus::begin();
    CAN_begin(tx_pin, rx_pin);

    xTaskCreatePinnedToCore(
        taskSBus,
        "SBUS",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        taskControl,
        "CONTROL",
        4096,
        nullptr,
        1,
        nullptr,
        1
    );

    // Serial.println("Setup done.");
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("loop");
}

void taskSBus(void *arg)
{
    while (true)
    {
        SBus::update();
        vTaskDelayUntil(pdMS_TO_TICKS(10));
    }
}

void taskControl(void *arg)
{
    int16_t drive_target[4];

    while (true)
    {
        if (!SBus::isConnected())
        {
            drive_target[0] = 0;
            drive_target[1] = 0;
            drive_target[2] = 0;
            drive_target[3] = 0;
        }
        else
        {
            uint16_t x = SBus::getChannel(0);
            uint16_t y = SBus::getChannel(1);

            Serial.printf("x:%d y:%d\n", x, y);

            // 後でここに足回り運動学を入れる


            // test
            drive_target[0] = 1;
            drive_target[1] = 2;
            drive_target[2] = 3;
            drive_target[3] = 4;
        }

        // ここは現在のCANライブラリの関数名に合わせる
        CAN_send(CAN_ID_DRIVE_TARGET, drive_target);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}