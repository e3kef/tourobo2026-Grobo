#include <Arduino.h>

#include "s_bus.h"
#include "can.h"
#include "can_protocol.h"

void taskSBus(void *arg);
void taskControl(void *arg);
int16_t normalizeStick(uint16_t raw, bool reverse);
void calcDriveTarget(
    int16_t x,
    int16_t y,
    int16_t r,
    int16_t target[4]
);

constexpr int tx_pin = 21;
constexpr int rx_pin = 22;

constexpr int SBUS_MIN = 326;
constexpr int SBUS_MID = 993;
constexpr int SBUS_MAX = 1659;

constexpr int SBUS_DEADZONE = 20;

// for serial test
constexpr int16_t DRIVE_TARGET_MAX = 1000;

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
}

void taskSBus(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        SBus::update();
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(10)
        );
    }
}

void taskControl(void *arg)
{
    int16_t drive_target[4];

    TickType_t last_wake_time = xTaskGetTickCount();

    // for serial test
    uint32_t last_print_ms = 0;

    while (true)
    {
         // CH1: 右スティック左右
        int16_t x =
            normalizeStick(
                SBus::getChannel(0),
                false
            );

        // CH3: 右スティック上下
        // 上端=326なので反転
        int16_t y =
            normalizeStick(
                SBus::getChannel(2),
                true
            );

        // CH4: 左スティック左右
        int16_t r =
            normalizeStick(
                SBus::getChannel(3),
                false
            );

        calcDriveTarget(
            x,
            y,
            r,
            drive_target
        );

        // CAN_send(
        //     CAN_ID_DRIVE_TARGET,
        //     drive_target
        // );

        bool send_ok = CAN_send(
            CAN_ID_DRIVE_TARGET,
            drive_target
        );

        if (!send_ok)
        {
            Serial.println("CAN SEND FAILED");
        }

        // Serialは50ms周期
        if (millis() - last_print_ms >= 50)
        {
            last_print_ms = millis();

            Serial.printf(
                "x:%4d y:%4d r:%4d | "
                "M1:%4d M2:%4d M3:%4d M4:%4d\n",
                x,
                y,
                r,
                drive_target[0],
                drive_target[1],
                drive_target[2],
                drive_target[3]
            );
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(10)
        );
    }


    // simple test↓
    
    // TickType_t last_wake_time = xTaskGetTickCount();

    // int16_t drive_target[4] = {
    //     100,
    //     200,
    //     300,
    //     400
    // };

    // while (true)
    // {
    //     CAN_send(CAN_ID_DRIVE_TARGET, drive_target);

    //     vTaskDelayUntil(
    //         &last_wake_time,
    //         pdMS_TO_TICKS(10)
    //     );
    // }
}

int16_t normalizeStick(uint16_t raw, bool reverse)
{
    int32_t value = 0;

    if (abs((int)raw - SBUS_MID) <= SBUS_DEADZONE)
    {
        return 0;
    }

    if (raw > SBUS_MID)
    {
        value =
            (int32_t)(raw - SBUS_MID) *
            DRIVE_TARGET_MAX /
            (SBUS_MAX - SBUS_MID);
    }
    else
    {
        value =
            -(int32_t)(SBUS_MID - raw) *
            DRIVE_TARGET_MAX /
            (SBUS_MID - SBUS_MIN);
    }

    value = constrain(
        value,
        -DRIVE_TARGET_MAX,
        DRIVE_TARGET_MAX
    );

    return reverse ? -value : value;
}

void calcDriveTarget(int16_t x, int16_t y, int16_t r, int16_t target[4])
{
    int32_t mix[4];

    // M1: 左前
    mix[0] = y + x + r;

    // M2: 右前
    mix[1] = -y + x + r;

    // M3: 右後
    mix[2] = -y - x + r;

    // M4: 左後
    mix[3] = y - x + r;

    // 最大絶対値を取得
    int32_t max_abs = DRIVE_TARGET_MAX;

    for (int i = 0; i < 4; i++)
    {
        int32_t a = abs(mix[i]);

        if (a > max_abs)
            max_abs = a;
    }

    // 比率を維持したまま ±1000 に収める
    for (int i = 0; i < 4; i++)
    {
        target[i] =
            (int32_t)mix[i] *
            DRIVE_TARGET_MAX /
            max_abs;
    }
}