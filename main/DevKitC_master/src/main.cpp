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

// ------------------------------------------------------------
// 足回り
// ------------------------------------------------------------

// 足回り最大目標回転数
constexpr int16_t DRIVE_MAX_RPM = 265;

// ------------------------------------------------------------
// 射出
// ------------------------------------------------------------


// 射出モーター最大目標周速度
// MOTOR1:
//   1.5 * PI * 1695 ≒ 7987
// MOTOR2:
//   (14 / 15) * PI * 2290 ≒ 6715
// 射出モーター最大目標周速度
// CH6 / CH8 のダイヤル位置を
// 0 ～ 100 % として、この値までの周速度に変換
// 単位 mm/s
// 
// テスト：50%
constexpr int16_t SHOOT_MOTOR1_FULL_SURFACE_SPEED = 12206;
constexpr int16_t SHOOT_MOTOR2_FULL_SURFACE_SPEED = 12608;

constexpr float SHOOT_MOTOR1_OUTPUT_RATIO = 0.8f;
constexpr float SHOOT_MOTOR2_OUTPUT_RATIO = 0.8f;

constexpr int16_t SHOOT_MOTOR1_MAX_SURFACE_SPEED = SHOOT_MOTOR1_FULL_SURFACE_SPEED * SHOOT_MOTOR1_OUTPUT_RATIO;
constexpr int16_t SHOOT_MOTOR2_MAX_SURFACE_SPEED = SHOOT_MOTOR2_FULL_SURFACE_SPEED * SHOOT_MOTOR2_OUTPUT_RATIO;

// 射出最大動作時間
constexpr uint32_t SHOOT_DURATION_MS = 5000;

// CH9
// MIDとMAXの中間より大きければ「下」と判定
//
// CH9:
// 上   ≈ 326
// 中央 ≈ 993
// 下   ≈ 1659
constexpr int SHOOT_SWITCH_ON_THRESHOLD =
    (SBUS_MID + SBUS_MAX) / 2;

// ------------------------------------------------------------
// 取得・昇降
// ------------------------------------------------------------

// 3段トグル判定
constexpr int SWITCH3_UP_THRESHOLD =
    (SBUS_MIN + SBUS_MID) / 2;

constexpr int SWITCH3_DOWN_THRESHOLD =
    (SBUS_MID + SBUS_MAX) / 2;

// Position target
constexpr int16_t POSITION_TARGET_DISABLE = -1;

// GET1
constexpr int16_t GET1_OPEN_TARGET   = 1798;
constexpr int16_t GET1_MIDDLE_TARGET = 2094;
constexpr int16_t GET1_CLOSE_TARGET  = 2166;

// GET2
constexpr int16_t GET2_OPEN_TARGET   = 2226;
constexpr int16_t GET2_MIDDLE_TARGET = 1940;
constexpr int16_t GET2_CLOSE_TARGET  = 1864;

// 昇降
constexpr int16_t LIFT_GET_TARGET    = 908;
constexpr int16_t LIFT_PLATE_TARGET  = 1464;
constexpr int16_t LIFT_GATE_TARGET   = 2680;

// air
constexpr int16_t AIR_CLOSE = 0;
constexpr int16_t AIR_OPEN  = 1;


// ============================================================
// Shared data
// ============================================================

struct ControlInput
{
    // 足回り
    int16_t x;
    int16_t y;
    int16_t r;

    // 射出
    //
    // CH6 = 左ダイヤル
    // CH8 = 右ダイヤル
    // CH9 = 射出トグル
    uint16_t raw_shoot_motor1;
    uint16_t raw_shoot_motor2;
    uint16_t raw_shoot_switch;


    // 取得・開閉
    // CH11 = 昇降3段
    uint16_t raw_lift_switch;

    // CH12 = 取得エアシリ2段
    uint16_t raw_air_switch;

    bool connected;
};

ControlInput control_input = {
    0, 0, 0, 
    0, 0, 0, 
    0, 0,
    false
};

// S.BUS task と Control task 間の共有データ保護
portMUX_TYPE control_input_mux =
    portMUX_INITIALIZER_UNLOCKED;


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

bool isShootSwitchOn(
    uint16_t raw
);

int16_t dialToSurfaceSpeed(
    uint16_t raw,
    int16_t max_speed
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
    TickType_t last_wake_time =
        xTaskGetTickCount();

    while (true)
    {
        // ---------------------
        // S.BUS受信
        // ---------------------

        SBus::update();

        // ---------------------
        // 足回りCH取得
        // ---------------------

        // CH1
        // 右スティック左右
        uint16_t raw_x =
            SBus::getChannel(0);

        // CH3
        // 右スティック上下
        uint16_t raw_y =
            SBus::getChannel(2);

        // CH4
        // 左スティック左右
        uint16_t raw_r =
            SBus::getChannel(3);

        // ---------------------
        // 射出CH取得
        // ---------------------

        // CH6
        // 左ダイヤル
        // 射出モーター1
        uint16_t raw_shoot_motor1 =
            SBus::getChannel(5);

        // CH8
        // 右ダイヤル
        // 射出モーター2
        uint16_t raw_shoot_motor2 =
            SBus::getChannel(7);

        // CH9
        // 射出トグル
        uint16_t raw_shoot_switch =
            SBus::getChannel(8);

        // CH11
        // 昇降
        // 上 = 取得, 中央 = 皿, 下 = 城門
        uint16_t raw_lift_switch =
            SBus::getChannel(10);

        // CH12
        // 取得エアシリ
        // 上 = 開, 下 = 閉
        uint16_t raw_air_switch =
            SBus::getChannel(11);

        // ---------------------
        // S.BUS接続確認
        // ---------------------

        bool connected =
            SBus::isConnected() &&
            isValidSBusValue(raw_x) &&
            isValidSBusValue(raw_y) &&
            isValidSBusValue(raw_r) &&
            isValidSBusValue(raw_shoot_motor1) &&
            isValidSBusValue(raw_shoot_motor2) &&
            isValidSBusValue(raw_shoot_switch) &&
            isValidSBusValue(raw_lift_switch) &&
            isValidSBusValue(raw_air_switch);

        ControlInput input_local = {
            0,
            0,
            0,
            0,
            0,
            0,
            false
        };

        if (connected)
        {
            // ---------------------
            // 足回り
            // ---------------------

            // +X = 機体右
            input_local.x =
                normalizeStick(
                    raw_x,
                    false
                );

            // CH3
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

            // ---------------------
            // 射出
            // ---------------------

            input_local.raw_shoot_motor1 =
                raw_shoot_motor1;

            input_local.raw_shoot_motor2 =
                raw_shoot_motor2;

            input_local.raw_shoot_switch =
                raw_shoot_switch;

            input_local.connected = true;

            // ---------------------
            // 取得・昇降
            // ---------------------            
            input_local.raw_lift_switch =
                raw_lift_switch;

            input_local.raw_air_switch =
                raw_air_switch;
        }


        // ---------------------
        // 共有変数更新
        // ---------------------

        portENTER_CRITICAL(
            &control_input_mux
        );

        control_input =
            input_local;

        portEXIT_CRITICAL(
            &control_input_mux
        );

        // ---------------------
        // 10 ms period
        // ---------------------

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(
                TASK_PERIOD_MS
            )
        );
    }
}


// ============================================================
// Control Task
// ============================================================

void taskControl(void *arg)
{
    TickType_t last_wake_time =
        xTaskGetTickCount();

    // --------------------------------------------------------
    // CAN送信用データフレーム
    // --------------------------------------------------------
    // 足回り
    int16_t drive_target[4] = {
        0,
        0,
        0,
        0
    };

    // 射出
    int16_t shoot_target[4] = {
        0,
        0,
        0,
        0
    };

    // 取得
    int16_t get_target[4] = {
        GET1_OPEN_TARGET,
        GET2_OPEN_TARGET,
        0,
        0
    };

    // 昇降
    int16_t lift_target[4] = {
        LIFT_PLATE_TARGET,
        0,
        0,
        0
    };

    // エアシリ
    int16_t air_target[4] = {
        AIR_CLOSE,
        0,
        0,
        0
    };

    // --------------------------------------------------------
    // 射出状態
    // --------------------------------------------------------

    bool shoot_active = false;

    // falseから開始する
    //
    // 起動時にCH9が下がっていた場合、
    // 勝手に射出開始しないため。
    //
    // 一度CH9を停止側へ戻した後、
    // trueになる。
    bool shoot_armed = false;

    // 射出開始時刻
    uint32_t shoot_start_ms = 0;

    // 射出開始時に一度だけ取得して保持する周速度
    int16_t latched_shoot_speed_1 = 0;
    int16_t latched_shoot_speed_2 = 0;

    // Debug
    uint32_t last_print_ms = 0;

    while (true)
    {
        // ---------------------
        // 共有入力取得
        // ---------------------

        ControlInput input_local;

        portENTER_CRITICAL(
            &control_input_mux
        );

        input_local =
            control_input;

        portEXIT_CRITICAL(
            &control_input_mux
        );

        // 現在時刻
        uint32_t now = millis();

        // ====================================================
        // 足回り指示値生成
        // ====================================================

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

        // ====================================================
        // 射出
        // ====================================================

        bool shoot_switch_on = false;

        if (input_local.connected)
        {
            shoot_switch_on =
                isShootSwitchOn(
                    input_local.raw_shoot_switch
                );
        }

        // ----------------------------------------------------
        // S.BUS通信断
        // ----------------------------------------------------

        if (!input_local.connected)
        {
            // 射出即停止
            shoot_active = false;

            // 再接続時にCH9が下だった場合も
            // 勝手に再射出しないようdisarm
            shoot_armed = false;

            latched_shoot_speed_1 = 0;
            latched_shoot_speed_2 = 0;
        }

        // ----------------------------------------------------
        // CH9停止側
        // ----------------------------------------------------

        else if (!shoot_switch_on)
        {
            // 動作中であっても即停止
            shoot_active = false;

            // 次回の射出を許可
            shoot_armed = true;

            latched_shoot_speed_1 = 0;
            latched_shoot_speed_2 = 0;
        }

        // ----------------------------------------------------
        // CH9射出側
        // ----------------------------------------------------

        else
        {
            // -----------------------------------------------
            // 射出開始
            // -----------------------------------------------

            if (!shoot_active &&
                shoot_armed)
            {
                // CH9を一度戻すまでは
                // 再射出させない
                shoot_armed = false;

                // 開始時刻
                shoot_start_ms = now;

                // -------------------------------------------
                // ダイヤル値取得
                //
                // ここで一度だけRPMへ変換し、
                // 今回の射出中は値を固定する
                // -------------------------------------------

                latched_shoot_speed_1 =
                    dialToSurfaceSpeed(
                        input_local.raw_shoot_motor1,
                        SHOOT_MOTOR1_MAX_SURFACE_SPEED
                    );

                latched_shoot_speed_2 =
                    dialToSurfaceSpeed(
                        input_local.raw_shoot_motor2,
                        SHOOT_MOTOR2_MAX_SURFACE_SPEED
                    );

                shoot_active = true;
            }

            // -----------------------------------------------
            // 最大5秒で停止
            // -----------------------------------------------

            if (shoot_active &&
                (now - shoot_start_ms >=
                 SHOOT_DURATION_MS))
            {
                shoot_active = false;

                latched_shoot_speed_1 = 0;
                latched_shoot_speed_2 = 0;
            }
        }

        // ----------------------------------------------------
        // 射出CANデータ生成
        // ----------------------------------------------------

        if (shoot_active)
        {
            shoot_target[0] =
                latched_shoot_speed_1;

            shoot_target[1] =
                latched_shoot_speed_2;
        }
        else
        {
            shoot_target[0] = 0;
            shoot_target[1] = 0;
        }

        shoot_target[2] = 0;
        shoot_target[3] = 0;

        // ====================================================
        // 取得
        // ====================================================

        if (input_local.connected)
        {
            uint16_t raw =
                input_local.raw_shoot_switch;

            // CH9 上
            // 外
            if (raw <
                SWITCH3_UP_THRESHOLD)
            {
                get_target[0] =
                    GET1_OPEN_TARGET;

                get_target[1] =
                    GET2_OPEN_TARGET;
            }

            // CH9 中央
            // 中・保持
            else if (raw <
                    SWITCH3_DOWN_THRESHOLD)
            {
                get_target[0] =
                    GET1_MIDDLE_TARGET;

                get_target[1] =
                    GET2_MIDDLE_TARGET;
            }

            // CH9 下
            // 内 + 射出
            else
            {
                get_target[0] =
                    GET1_CLOSE_TARGET;

                get_target[1] =
                    GET2_CLOSE_TARGET;
            }
        }
        else
        {
            get_target[0] =
                POSITION_TARGET_DISABLE;

            get_target[1] =
                POSITION_TARGET_DISABLE;
        }

        get_target[2] = 0;
        get_target[3] = 0;


        // ====================================================
        // 昇降
        // ====================================================

        if (input_local.connected)
        {
            uint16_t raw =
                input_local.raw_lift_switch;

            // CH11 上
            // 取得
            if (raw <
                SWITCH3_UP_THRESHOLD)
            {
                lift_target[0] =
                    LIFT_GATE_TARGET;
            }

            // CH11 中央
            // 皿
            else if (raw <
                    SWITCH3_DOWN_THRESHOLD)
            {
                lift_target[0] =
                    LIFT_PLATE_TARGET;
            }

            // CH11 下
            // 城門
            else
            {
                lift_target[0] =
                    LIFT_GET_TARGET;
            }
        }
        else
        {
            lift_target[0] =
                POSITION_TARGET_DISABLE;
        }

        lift_target[1] = 0;
        lift_target[2] = 0;
        lift_target[3] = 0;


        // ====================================================
        // 取得エアシリ
        // ====================================================

        if (input_local.connected)
        {
            // CH12
            //
            // 上 ≈ 326 → OPEN
            // 下 ≈1659 → CLOSE

            if (input_local.raw_air_switch <
                SBUS_MID)
            {
                air_target[0] =
                    AIR_CLOSE;
            }
            else
            {
                air_target[0] =
                    AIR_OPEN;
            }
        }
        else
        {
            // 通信断時は閉
            air_target[0] =
                AIR_CLOSE;
        }

        air_target[1] = 0;
        air_target[2] = 0;
        air_target[3] = 0;

        // ====================================================
        // CAN送信
        // ====================================================

        // 足回り
        bool drive_send_ok =
            CAN_send(
                CAN_ID_DRIVE_TARGET,
                drive_target
            );

        // 射出
        //
        // active中:
        //   ラッチしたRPMを10ms周期で送信
        //
        // inactive中:
        //   0RPMを10ms周期で送信
        bool shoot_send_ok =
            CAN_send(
                CAN_ID_SHOOT_TARGET,
                shoot_target
            );

        // 取得
        bool get_send_ok =
            CAN_send(
                CAN_ID_GET_TARGET,
                get_target
            );

        // 昇降
        bool lift_send_ok =
            CAN_send(
                CAN_ID_LIFT_TARGET,
                lift_target
            );

        // エアシリ
        bool air_send_ok =
            CAN_send(
                CAN_ID_GET_AIR_TARGET,
                air_target
            );



        // ====================================================
        // Debug
        // ====================================================

        if (millis() - last_print_ms >= 100)
        {
            last_print_ms = millis();

            Serial.printf(
                "SBUS:%d | "
                "x:%4d y:%4d r:%4d | "
                "DRIVE M1:%4d M2:%4d M3:%4d M4:%4d | "
                "SHOOT SW:%4u "
                "D1:%4u D2:%4u "
                "ACTIVE:%d ARMED:%d "
                "RPM1:%5d RPM2:%5d | "
                "CAN D:%d S:%d\n",

                input_local.connected,

                input_local.x,
                input_local.y,
                input_local.r,

                drive_target[0],
                drive_target[1],
                drive_target[2],
                drive_target[3],

                input_local.raw_shoot_switch,
                input_local.raw_shoot_motor1,
                input_local.raw_shoot_motor2,

                shoot_active,
                shoot_armed,

                shoot_target[0],
                shoot_target[1],

                drive_send_ok,
                shoot_send_ok
            );
        }

        // ---------------------
        // 10 ms period
        // ---------------------

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(
                TASK_PERIOD_MS
            )
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

    if (abs((int)raw - SBUS_MID)
        <= SBUS_DEADZONE)
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

    return static_cast<int16_t>(
        value
    );
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
    // 実測範囲 326 ～ 1659 に対し
    // 少し余裕を持たせる

    return raw >= 250 &&
           raw <= 1750;
}


// ============================================================
// Shoot switch
// ============================================================

bool isShootSwitchOn(
    uint16_t raw
)
{
    // CH9
    //
    // 上   ≈ 326
    // 中央 ≈ 993
    // 下   ≈ 1659
    //
    // MID-MAX間の中点を超えたら
    // 「下 = 射出ON」とする

    return raw >
           SHOOT_SWITCH_ON_THRESHOLD;
}


// ============================================================
// Shoot dial -> RPM
// ============================================================

int16_t dialToSurfaceSpeed(
    uint16_t raw,
    int16_t max_speed
)
{
    // 実測範囲外をクランプ
    raw = constrain(
        raw,
        SBUS_MIN,
        SBUS_MAX
    );

    // CH6 / CH8
    //
    // 326  ->   0 %
    // 993  -> 約50 %
    // 1659 -> 100 %
    //
    // max_speed に対する割合として変換

    int32_t speed =
        (int32_t)(raw - SBUS_MIN) *
        max_speed /
        (SBUS_MAX - SBUS_MIN);

    speed = constrain(
        speed,
        0,
        max_speed
    );

    return static_cast<int16_t>(
        speed
    );
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

    int32_t max_abs =
        STICK_MAX;

    for (int i = 0; i < 4; i++)
    {
        int32_t value =
            mix[i];

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