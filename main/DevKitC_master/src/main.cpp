#include <Arduino.h>

#include "s_bus.h"
#include "can.h"
#include "can_protocol.h"

// ============================================================
// Setting
// ============================================================

// ESP32 DevKitC CAN
constexpr int CAN_TX = 21;
constexpr int CAN_RX = 22;

// 制御周期
constexpr uint32_t TASK_PERIOD_MS = 10;

// S.BUS
constexpr int SBUS_MIN = 326;
constexpr int SBUS_MID = 993;
constexpr int SBUS_MAX = 1659;

constexpr int SBUS_DEADZONE = 20;

// スティック内部演算用
constexpr int16_t STICK_MAX = 1000;

// 足回り最大目標回転数
constexpr int16_t DRIVE_MAX_RPM = 265;


// ============================================================
// Shared data
// ============================================================

struct DriveInput
{
    int16_t x;
    int16_t y;
    int16_t r;

    bool connected;
};

DriveInput drive_input = {
    0,
    0,
    0,
    false
};

// S.BUS task と Control task 間の共有データ保護
portMUX_TYPE drive_input_mux = portMUX_INITIALIZER_UNLOCKED;


// ============================================================
// Function prototype
// ============================================================

void taskSBus(void *arg);
void taskControl(void *arg);

int16_t normalizeStick(
    uint16_t raw,
    bool reverse
);

bool isValidSBusValue(
    uint16_t raw
);

void calcDriveTarget(
    int16_t x,
    int16_t y,
    int16_t r,
    int16_t target[4]
);


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    // ---------------------
    // S.BUS
    // ---------------------

    SBus::begin();

    // ---------------------
    // CAN
    // ---------------------

    if (!CAN_begin(CAN_TX, CAN_RX))
    {
        Serial.println("CAN init failed");

        while (true)
        {
            delay(100);
        }
    }

    Serial.println("CAN init OK");

    // ---------------------
    // S.BUS task
    // ---------------------

    xTaskCreatePinnedToCore(
        taskSBus,
        "SBUS",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );

    // ---------------------
    // Control task
    // ---------------------

    xTaskCreatePinnedToCore(
        taskControl,
        "CONTROL",
        4096,
        nullptr,
        1,
        nullptr,
        1
    );

    Serial.println("Setup done");
}


// ============================================================
// Loop
// ============================================================

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}


// ============================================================
// S.BUS Task
// ============================================================

void taskSBus(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        // ---------------------
        // S.BUS受信
        // ---------------------

        SBus::update();

        // ---------------------
        // 必要なCHを取得
        // ---------------------

        // CH1
        // 右スティック左右
        uint16_t raw_x = SBus::getChannel(0);

        // CH3
        // 右スティック上下
        uint16_t raw_y = SBus::getChannel(2);

        // CH4
        // 左スティック左右
        uint16_t raw_r = SBus::getChannel(3);

        // ---------------------
        // S.BUS接続確認
        // ---------------------

        bool connected =
            SBus::isConnected() &&
            isValidSBusValue(raw_x) &&
            isValidSBusValue(raw_y) &&
            isValidSBusValue(raw_r);

        DriveInput input_local = {
            0,
            0,
            0,
            false
        };

        if (connected)
        {
            // +X = 機体右
            input_local.x =
                normalizeStick(
                    raw_x,
                    false
                );

            // CH3は
            // 上端 = 326
            //
            // +Y = 機体前方
            // とするため反転
            input_local.y =
                normalizeStick(
                    raw_y,
                    true
                );

            // +R = CW
            input_local.r =
                normalizeStick(
                    raw_r,
                    false
                );

            input_local.connected = true;
        }

        // ---------------------
        // 共有変数更新
        // ---------------------

        portENTER_CRITICAL(&drive_input_mux);

        drive_input = input_local;

        portEXIT_CRITICAL(&drive_input_mux);

        // ---------------------
        // 10 ms period
        // ---------------------

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(TASK_PERIOD_MS)
        );
    }
}


// ============================================================
// Control Task
// ============================================================

void taskControl(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    int16_t drive_target[4] = {
        0,
        0,
        0,
        0
    };

    uint32_t last_print_ms = 0;

    while (true)
    {
        // ---------------------
        // 共有入力取得
        // ---------------------

        DriveInput input_local;

        portENTER_CRITICAL(&drive_input_mux);

        input_local = drive_input;

        portEXIT_CRITICAL(&drive_input_mux);

        // ---------------------
        // 足回り指示値生成
        // ---------------------

        if (input_local.connected)
        {
            calcDriveTarget(
                input_local.x,
                input_local.y,
                input_local.r,
                drive_target
            );
        }
        else
        {
            // S.BUS通信断
            // → 目標RPMを0
            drive_target[0] = 0;
            drive_target[1] = 0;
            drive_target[2] = 0;
            drive_target[3] = 0;
        }

        // ---------------------
        // CAN送信
        // ---------------------

        bool send_ok =
            CAN_send(
                CAN_ID_DRIVE_TARGET,
                drive_target
            );

        // ---------------------
        // Debug
        // ---------------------

        if (millis() - last_print_ms >= 100)
        {
            last_print_ms = millis();

            Serial.printf(
                "SBUS:%d | "
                "x:%4d y:%4d r:%4d | "
                "RPM M1:%4d M2:%4d M3:%4d M4:%4d | "
                "CAN:%d\n",
                input_local.connected,
                input_local.x,
                input_local.y,
                input_local.r,
                drive_target[0],
                drive_target[1],
                drive_target[2],
                drive_target[3],
                send_ok
            );
        }

        // ---------------------
        // 10 ms period
        // ---------------------

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(TASK_PERIOD_MS)
        );
    }
}


// ============================================================
// Stick normalize
// ============================================================

int16_t normalizeStick(
    uint16_t raw,
    bool reverse
)
{
    // ---------------------
    // Dead zone
    // ---------------------

    if (abs((int)raw - SBUS_MID) <= SBUS_DEADZONE)
    {
        return 0;
    }

    int32_t value;

    // ---------------------
    // MID → MAX
    // ---------------------

    if (raw > SBUS_MID)
    {
        value =
            (int32_t)(raw - SBUS_MID) *
            STICK_MAX /
            (SBUS_MAX - SBUS_MID);
    }

    // ---------------------
    // MIN → MID
    // ---------------------

    else
    {
        value =
            -(int32_t)(SBUS_MID - raw) *
            STICK_MAX /
            (SBUS_MID - SBUS_MIN);
    }

    value = constrain(
        value,
        -STICK_MAX,
        STICK_MAX
    );

    if (reverse)
    {
        value = -value;
    }

    return static_cast<int16_t>(value);
}


// ============================================================
// S.BUS value validation
// ============================================================

bool isValidSBusValue(
    uint16_t raw
)
{
    // 起動直後 channels[] = 0 の状態を
    // 有効なS.BUS入力と判定しないためのチェック
    //
    // 実測範囲 326 ～ 1659 に対し少し余裕を持たせる

    return raw >= 250 && raw <= 1750;
}


// ============================================================
// Drive mixing
// ============================================================

void calcDriveTarget(
    int16_t x,
    int16_t y,
    int16_t r,
    int16_t target[4]
)
{
    int32_t mix[4];

    // ========================================================
    // IMG_9124.jpeg 採用ミックス
    //
    // +X = 右移動
    // +Y = 前進
    // +R = CW
    //
    // M1 = 左前
    // M2 = 右前
    // M3 = 右後
    // M4 = 左後
    // ========================================================

    mix[0] =  y + x + r;
    mix[1] = -y + x + r;
    mix[2] = -y - x + r;
    mix[3] =  y - x + r;

    // ---------------------
    // 最大絶対値取得
    // ---------------------

    int32_t max_abs = STICK_MAX;

    for (int i = 0; i < 4; i++)
    {
        int32_t value = mix[i];

        if (value < 0)
        {
            value = -value;
        }

        if (value > max_abs)
        {
            max_abs = value;
        }
    }

    // ---------------------
    // 比率を維持したまま
    // 最大 ±265 RPM に変換
    // ---------------------

    for (int i = 0; i < 4; i++)
    {
        target[i] =
            static_cast<int16_t>(
                mix[i] *
                DRIVE_MAX_RPM /
                max_abs
            );
    }
}