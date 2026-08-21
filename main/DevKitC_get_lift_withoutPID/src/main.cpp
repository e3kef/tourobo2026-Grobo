#include <Arduino.h>

#include "can.h"
#include "can_protocol.h"

// ============================================================
// Setting
// ============================================================

// ============================================================
// CAN
// ============================================================

constexpr int CAN_TX = 21;
constexpr int CAN_RX = 22;

// ============================================================
// Potentiometer
// ============================================================

constexpr int POT_GET1_PIN = 32;
constexpr int POT_GET2_PIN = 33;
constexpr int POT_LIFT_PIN = 35;

// ============================================================
// Motor pins
// ============================================================

// GET1
constexpr int GET1_PWM_PIN = 14;
constexpr int GET1_DIR_PIN = 13;

// GET2
constexpr int GET2_PWM_PIN = 26;
constexpr int GET2_DIR_PIN = 25;

// LIFT
constexpr int LIFT_PWM_PIN = 4;
constexpr int LIFT_DIR_PIN = 18;

// ============================================================
// Air cylinder
// ============================================================

constexpr int AIR_PIN = 16;
constexpr uint8_t AIR_OPEN_LEVEL = HIGH;

// ============================================================
// PWM
// ============================================================

constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 10;

constexpr int GET1_PWM_CHANNEL = 0;
constexpr int GET2_PWM_CHANNEL = 1;
constexpr int LIFT_PWM_CHANNEL = 2;

// 10bit PWM
// 0 ～ 1023
constexpr int PWM_DUTY_LIMIT = 512;

// ============================================================
// Motor direction
// ============================================================

constexpr int GET1_MOTOR_SIGN = -1;
constexpr int GET2_MOTOR_SIGN = -1;
constexpr int LIFT_MOTOR_SIGN = +1;

// ============================================================
// Control
// ============================================================

constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t CAN_TIMEOUT_MS = 100;

// ADC平均
constexpr int ADC_AVG_COUNT = 4;

// 目標位置±この範囲で停止
constexpr int GET_POSITION_TOLERANCE = 3;
constexpr int LIFT_POSITION_TOLERANCE = 3;

// -1 = 制御無効
constexpr int16_t POSITION_TARGET_DISABLE = -1;

// ============================================================
// GET HARD LIMIT
// ============================================================

constexpr int GET1_OPEN_LIMIT  = 1744;
constexpr int GET1_CLOSE_LIMIT = 2702;

constexpr int GET2_OPEN_LIMIT  = 2504;
constexpr int GET2_CLOSE_LIMIT = 1984;

// ============================================================
// Shared state
// ============================================================

struct ControlState
{
    int16_t get1_target;
    int16_t get2_target;
    int16_t lift_target;

    int16_t air_target;

    uint32_t last_get_time;
    uint32_t last_lift_time;
    uint32_t last_air_time;

    int16_t mode;

    bool system_stop;
};

ControlState state = {
    POSITION_TARGET_DISABLE,
    POSITION_TARGET_DISABLE,
    POSITION_TARGET_DISABLE,

    0,

    0,
    0,
    0,

    CAN_MODE_TEST,

    false
};

portMUX_TYPE state_mux =
    portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Function prototype
// ============================================================

void taskCAN(void *arg);
void taskControl(void *arg);

void receiveCAN();

int readPotAverage(
    int pin
);

void setMotorPWM(
    int pwm_channel,
    int dir_pin,
    int output,
    int motor_sign
);

void stopAllMotors();

void setAirCylinder(
    bool open
);

// Pot limit
bool arePotLimitsEnabled(
    int open_limit,
    int close_limit
);

bool isTargetInsidePotLimits(
    int target,
    int open_limit,
    int close_limit
);

bool isPotLimitBlocking(
    int output,
    int current_position,
    int open_limit,
    int close_limit
);

// ============================================================
// Setup
// ============================================================

void setup()
{
    // ========================================================
    // 起動直後のモータ出力を確実にOFF
    // ========================================================

    pinMode(GET1_PWM_PIN, OUTPUT);
    pinMode(GET1_DIR_PIN, OUTPUT);

    pinMode(GET2_PWM_PIN, OUTPUT);
    pinMode(GET2_DIR_PIN, OUTPUT);

    pinMode(LIFT_PWM_PIN, OUTPUT);
    pinMode(LIFT_DIR_PIN, OUTPUT);

    digitalWrite(GET1_PWM_PIN, LOW);
    digitalWrite(GET2_PWM_PIN, LOW);
    digitalWrite(LIFT_PWM_PIN, LOW);

    digitalWrite(GET1_DIR_PIN, LOW);
    digitalWrite(GET2_DIR_PIN, LOW);
    digitalWrite(LIFT_DIR_PIN, LOW);

    Serial.begin(115200);

    // ========================================================
    // ADC
    // ========================================================

    analogReadResolution(12);

    pinMode(
        POT_GET1_PIN,
        INPUT
    );

    pinMode(
        POT_GET2_PIN,
        INPUT
    );

    pinMode(
        POT_LIFT_PIN,
        INPUT
    );

    // ========================================================
    // CAN
    // ========================================================

    if (!CAN_begin(
            CAN_TX,
            CAN_RX))
    {
        Serial.println(
            "CAN init failed"
        );

        while (true)
        {
            delay(100);
        }
    }

    Serial.println(
        "CAN init OK"
    );

    // ========================================================
    // Motor
    // ========================================================

    pinMode(
        GET1_DIR_PIN,
        OUTPUT
    );

    pinMode(
        GET2_DIR_PIN,
        OUTPUT
    );

    pinMode(
        LIFT_DIR_PIN,
        OUTPUT
    );

    ledcSetup(
        GET1_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        GET2_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        LIFT_PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        GET1_PWM_PIN,
        GET1_PWM_CHANNEL
    );

    ledcAttachPin(
        GET2_PWM_PIN,
        GET2_PWM_CHANNEL
    );

    ledcAttachPin(
        LIFT_PWM_PIN,
        LIFT_PWM_CHANNEL
    );

    stopAllMotors();

    // ========================================================
    // Air
    // ========================================================

    pinMode(
        AIR_PIN,
        OUTPUT
    );

    // 起動時閉
    setAirCylinder(
        false
    );

    // ========================================================
    // CAN task
    // ========================================================

    xTaskCreatePinnedToCore(
        taskCAN,
        "CAN",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    // ========================================================
    // Control task
    // ========================================================

    xTaskCreatePinnedToCore(
        taskControl,
        "CONTROL",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );

    Serial.println(
        "Setup done"
    );
}

// ============================================================
// Loop
// ============================================================

void loop()
{
    vTaskDelay(
        pdMS_TO_TICKS(1000)
    );
}

// ============================================================
// CAN Task
// ============================================================

void taskCAN(void *arg)
{
    while (true)
    {
        receiveCAN();
    }
}

// ============================================================
// Control Task
// ============================================================

void taskControl(void *arg)
{
    TickType_t last_wake_time =
        xTaskGetTickCount();

    uint32_t last_print_ms = 0;

    // LIFT到達ラッチ
    int16_t last_lift_target = POSITION_TARGET_DISABLE;
    bool lift_target_reached = false;

    // GET到達ラッチ
    int16_t last_get1_target = POSITION_TARGET_DISABLE;
    int16_t last_get2_target = POSITION_TARGET_DISABLE;

    bool get1_target_reached = false;
    bool get2_target_reached = false;

    while (true)
    {
        // ====================================================
        // Shared state snapshot
        // ====================================================

        ControlState local;

        portENTER_CRITICAL(
            &state_mux
        );

        local = state;

        portEXIT_CRITICAL(
            &state_mux
        );

        // ====================================================
        // Pot
        // ====================================================

        int get1_position =
            readPotAverage(
                POT_GET1_PIN
            );

        int get2_position =
            readPotAverage(
                POT_GET2_PIN
            );

        int lift_position =
            readPotAverage(
                POT_LIFT_PIN
            );

        uint32_t now_ms =
            millis();

        // ====================================================
        // Timeout
        // ====================================================

        bool get_timeout =
            local.last_get_time == 0 ||
            now_ms -
                local.last_get_time >
                CAN_TIMEOUT_MS;

        bool lift_timeout =
            local.last_lift_time == 0 ||
            now_ms -
                local.last_lift_time >
                CAN_TIMEOUT_MS;

        bool air_timeout =
            local.last_air_time == 0 ||
            now_ms -
                local.last_air_time >
                CAN_TIMEOUT_MS;

        // ====================================================
        // GET1
        // ====================================================

        bool get1_limit_blocked =
            false;

        bool get1_target_invalid =
            false;

        int get1_output = 0;

        // 新しい有効ターゲットが来たら再アーム
        if (local.get1_target >= 0 &&
            local.get1_target != last_get1_target)
        {
            last_get1_target = local.get1_target;
            get1_target_reached = false;
        }

        if (local.system_stop ||
            get_timeout ||
            local.get1_target <
                0)
        {
            get1_output = 0;
        }
        else if (get1_target_reached)
        {
            // 一度到達したら次のターゲット変更まで停止
            get1_output = 0;
        }
        else
        {
            // ------------------------------------------------
            // Target safety
            // ------------------------------------------------

            if (!isTargetInsidePotLimits(
                    local.get1_target,
                    GET1_OPEN_LIMIT,
                    GET1_CLOSE_LIMIT))
            {
                // TARGET自体がLIMIT外なら動かさない
                get1_target_invalid =
                    true;

                get1_output = 0;
            }
            else
            {
                int error =
                    local.get1_target -
                    get1_position;

                // --------------------------------------------
                // Position tolerance
                // --------------------------------------------

                if (abs(error) <=
                    GET_POSITION_TOLERANCE)
                {
                    get1_target_reached = true;
                    get1_output = 0;
                }
                else
                {
                    // ----------------------------------------
                    // PIDなし
                    //
                    // 誤差の符号だけで方向を決定し、
                    // PWM_DUTY_LIMIT固定で駆動
                    // ----------------------------------------

                    if (error > 0)
                    {
                        get1_output =
                            PWM_DUTY_LIMIT;
                    }
                    else
                    {
                        get1_output =
                            -PWM_DUTY_LIMIT;
                    }

                    // ----------------------------------------
                    // HARD LIMIT
                    // ----------------------------------------

                    if (isPotLimitBlocking(
                            get1_output,
                            get1_position,
                            GET1_OPEN_LIMIT,
                            GET1_CLOSE_LIMIT))
                    {
                        get1_limit_blocked =
                            true;

                        get1_output = 0;
                    }
                }
            }
        }

        setMotorPWM(
            GET1_PWM_CHANNEL,
            GET1_DIR_PIN,
            get1_output,
            GET1_MOTOR_SIGN
        );

        // ====================================================
        // GET2
        // ====================================================

        bool get2_limit_blocked =
            false;

        bool get2_target_invalid =
            false;

        int get2_output = 0;

        // 新しい有効ターゲットが来たら再アーム
        if (local.get2_target >= 0 &&
            local.get2_target != last_get2_target)
        {
            last_get2_target = local.get2_target;
            get2_target_reached = false;
        }

        if (local.system_stop ||
            get_timeout ||
            local.get2_target <
                0)
        {
            get2_output = 0;
        }
        else if (get1_target_reached)
        {
            // 一度到達したら次のターゲット変更まで停止
            get1_output = 0;
        }
        else
        {
            if (!isTargetInsidePotLimits(
                    local.get2_target,
                    GET2_OPEN_LIMIT,
                    GET2_CLOSE_LIMIT))
            {
                get2_target_invalid =
                    true;

                get2_output = 0;
            }
            else
            {
                int error =
                    local.get2_target -
                    get2_position;

                if (abs(error) <=
                    GET_POSITION_TOLERANCE)
                {
                    get2_target_reached = true;
                    get2_output = 0;
                }
                else
                {
                    if (error > 0)
                    {
                        get2_output =
                            PWM_DUTY_LIMIT;
                    }
                    else
                    {
                        get2_output =
                            -PWM_DUTY_LIMIT;
                    }

                    if (isPotLimitBlocking(
                            get2_output,
                            get2_position,
                            GET2_OPEN_LIMIT,
                            GET2_CLOSE_LIMIT))
                    {
                        get2_limit_blocked =
                            true;

                        get2_output = 0;
                    }
                }
            }
        }

        setMotorPWM(
            GET2_PWM_CHANNEL,
            GET2_DIR_PIN,
            get2_output,
            GET2_MOTOR_SIGN
        );

        // ====================================================
        // LIFT
        // ====================================================

        int lift_output = 0;
        // dir 0: LOW, 1: HIGH;
        int lift_dir = LIFT_MOTOR_SIGN;

                if (local.lift_target != last_lift_target)
        {
            last_lift_target =
                local.lift_target;

            lift_target_reached =
                false;
        }

        if (local.system_stop ||
            lift_timeout ||
            local.lift_target <
                0)
        {
            lift_output = 0;
        }
        else if (lift_target_reached)
        {
            // 一度到達したら次の目標値が来るまで停止
            lift_output = 0;
        }
        else
        {
            int error =
                local.lift_target -
                lift_position;

            if (abs(error) <=
                LIFT_POSITION_TOLERANCE)
            {
                // 目標範囲に一度入った時点でラッチ
                lift_target_reached =
                    true;

                lift_output = 0;
            }
            else
            {
                if (error > 0)
                {
                    lift_dir =
                        LIFT_MOTOR_SIGN;

                    lift_output =
                        PWM_DUTY_LIMIT;
                }
                else
                {
                    lift_dir =
                        -(LIFT_MOTOR_SIGN);

                    lift_output =
                        PWM_DUTY_LIMIT;
                }
            }
        }
        setMotorPWM(
            LIFT_PWM_CHANNEL,
            LIFT_DIR_PIN,
            lift_output,
            lift_dir
        );

        // ====================================================
        // AIR
        // ====================================================

        if (local.system_stop ||
            air_timeout)
        {
            // 通信断では閉
            setAirCylinder(
                false
            );
        }
        else
        {
            setAirCylinder(
                local.air_target != 0
            );
        }

        // ====================================================
        // Debug
        // ====================================================

        if (now_ms -
            last_print_ms >=
            100)
        {
            last_print_ms =
                now_ms;

            Serial.printf(
                "GET1 T:%4d P:%4d OUT:%4d LIM:%d INV:%d | "
                "GET2 T:%4d P:%4d OUT:%4d LIM:%d INV:%d | "
                "LIFT T:%4d P:%4d OUT:%4d | "
                "AIR:%d | "
                "TO G:%d L:%d A:%d\n",

                local.get1_target,
                get1_position,
                get1_output,
                get1_limit_blocked,
                get1_target_invalid,

                local.get2_target,
                get2_position,
                get2_output,
                get2_limit_blocked,
                get2_target_invalid,

                local.lift_target,
                lift_position,
                lift_output,

                local.air_target,

                get_timeout,
                lift_timeout,
                air_timeout
            );
        }

        // ====================================================
        // 10 ms
        // ====================================================

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(
                CONTROL_PERIOD_MS
            )
        );
    }
}

// ============================================================
// CAN Receive
// ============================================================

void receiveCAN()
{
    uint16_t id;
    int16_t data[4];

    if (!CAN_receive(
            id,
            data,
            1000))
    {
        return;
    }

    uint32_t now =
        millis();

    switch (id)
    {
        // ====================================================
        // SYSTEM STOP
        // ====================================================

        case CAN_ID_SYSTEM_STOP:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            state.system_stop =
                true;

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }

        // ====================================================
        // MODE
        // ====================================================

        case CAN_ID_MODE_SETTING:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            state.mode =
                data[0];

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }

        // ====================================================
        // GET
        // ====================================================

        case CAN_ID_GET_TARGET:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            state.get1_target =
                data[0];

            state.get2_target =
                data[1];

            state.last_get_time =
                now;

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }

        // ====================================================
        // LIFT
        // ====================================================

        case CAN_ID_LIFT_TARGET:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            state.lift_target =
                data[0];

            state.last_lift_time =
                now;

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }

        // ====================================================
        // AIR
        // ====================================================

        case CAN_ID_GET_AIR_TARGET:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            state.air_target =
                data[0];

            state.last_air_time =
                now;

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }
    }
}

// ============================================================
// Potentiometer
// ============================================================

int readPotAverage(
    int pin
)
{
    uint32_t sum = 0;

    for (int i = 0;
         i < ADC_AVG_COUNT;
         i++)
    {
        sum +=
            analogRead(
                pin
            );
    }

    return static_cast<int>(
        sum /
        ADC_AVG_COUNT
    );
}

// ============================================================
// Pot limit enabled
// ============================================================

bool arePotLimitsEnabled(
    int open_limit,
    int close_limit
)
{
    return open_limit >= 0 &&
           close_limit >= 0;
}

// ============================================================
// Target inside limit
// ============================================================

bool isTargetInsidePotLimits(
    int target,
    int open_limit,
    int close_limit
)
{
    // // -1ならリミット機能自体を無効化
    // if (!arePotLimitsEnabled(
    //         open_limit,
    //         close_limit))
    // {
    //     return true;
    // }

    int lower_limit =
        min(
            open_limit,
            close_limit
        );

    int upper_limit =
        max(
            open_limit,
            close_limit
        );

    return target >= lower_limit &&
           target <= upper_limit;
}

// ============================================================
// Hard limit
// ============================================================

bool isPotLimitBlocking(
    int output,
    int current_position,
    int open_limit,
    int close_limit
)
{
    if (!arePotLimitsEnabled(
            open_limit,
            close_limit))
    {
        return false;
    }

    // OPEN/CLOSEのどちらでADCが大きくなるかは問わない
    int lower_limit =
        min(
            open_limit,
            close_limit
        );

    int upper_limit =
        max(
            open_limit,
            close_limit
        );

    // output > 0
    // ADC値を増加させる方向
    if (output > 0 &&
        current_position >=
            upper_limit)
    {
        return true;
    }

    // output < 0
    // ADC値を減少させる方向
    if (output < 0 &&
        current_position <=
            lower_limit)
    {
        return true;
    }

    // リミット側から中央へ戻る方向は許可
    return false;
}

// ============================================================
// Motor
// ============================================================

void setMotorPWM(
    int pwm_channel,
    int dir_pin,
    int output,
    int motor_sign
)
{
    // --------------------------------------------------------
    // Motor direction correction
    // --------------------------------------------------------

    output *=
        motor_sign;

    // --------------------------------------------------------
    // Output limit
    // --------------------------------------------------------

    output =
        constrain(
            output,
            -PWM_DUTY_LIMIT,
            PWM_DUTY_LIMIT
        );

    // --------------------------------------------------------
    // DIR
    // --------------------------------------------------------

    if (output >= 0)
    {
        digitalWrite(
            dir_pin,
            HIGH
        );
    }
    else
    {
        digitalWrite(
            dir_pin,
            LOW
        );

        output =
            -output;
    }

    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    ledcWrite(
        pwm_channel,
        output
    );
}

// ============================================================
// Stop all motors
// ============================================================

void stopAllMotors()
{
    ledcWrite(
        GET1_PWM_CHANNEL,
        0
    );

    ledcWrite(
        GET2_PWM_CHANNEL,
        0
    );

    ledcWrite(
        LIFT_PWM_CHANNEL,
        0
    );
}

// ============================================================
// Air cylinder
// ============================================================

void setAirCylinder(
    bool open
)
{
    if (open)
    {
        digitalWrite(
            AIR_PIN,
            AIR_OPEN_LEVEL
        );
    }
    else
    {
        digitalWrite(
            AIR_PIN,
            AIR_OPEN_LEVEL == HIGH
                ? LOW
                : HIGH
        );
    }
}