#include <Arduino.h>

#include "can.h"
#include "can_protocol.h"
#include "pid.h"

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

// 10bit
// 0 ～ 1023
constexpr int PWM_DUTY_LIMIT = 205;

constexpr float PID_OUTPUT_MAX = 1000.0f;

// ============================================================
// Motor direction
// ============================================================

constexpr int GET1_MOTOR_SIGN = -1;
constexpr int GET2_MOTOR_SIGN = -1;
constexpr int LIFT_MOTOR_SIGN = -1;

// ============================================================
// Control
// ============================================================

constexpr uint32_t CONTROL_PERIOD_MS = 10;

constexpr uint32_t CAN_TIMEOUT_MS = 100;

// ADC平均
constexpr int ADC_AVG_COUNT = 4;

// 目標位置±この範囲で停止
constexpr int GET_POSITION_TOLERANCE = 10;
constexpr int LIFT_POSITION_TOLERANCE = 10;

// -1 = 制御無効
constexpr int16_t POSITION_TARGET_DISABLE = -1;

// ============================================================
// GET HARD LIMIT
// ============================================================

constexpr int GET1_OPEN_LIMIT  = 3050;
constexpr int GET1_CLOSE_LIMIT = 3800;

constexpr int GET2_OPEN_LIMIT  = 2220;
constexpr int GET2_CLOSE_LIMIT = 1430;

// ============================================================
// PID gain
// ============================================================

constexpr float GET_KP = 0.5f;
constexpr float GET_KI = 0.0f;
constexpr float GET_KD = 0.0f;

constexpr float LIFT_KP = 0.5f;
constexpr float LIFT_KI = 0.0f;
constexpr float LIFT_KD = 0.0f;

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
// PID
// ============================================================

PIDController get1_pid(
    GET_KP,
    GET_KI,
    GET_KD
);

PIDController get2_pid(
    GET_KP,
    GET_KI,
    GET_KD
);

PIDController lift_pid(
    LIFT_KP,
    LIFT_KI,
    LIFT_KD
);

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
    float output,
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
    float output,
    int current_position,
    int open_limit,
    int close_limit
);

// ============================================================
// Setup
// ============================================================

void setup()
{
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
    // PID
    // ========================================================

    get1_pid.setOutputLimits(
        -PID_OUTPUT_MAX,
        PID_OUTPUT_MAX
    );

    get2_pid.setOutputLimits(
        -PID_OUTPUT_MAX,
        PID_OUTPUT_MAX
    );

    lift_pid.setOutputLimits(
        -PID_OUTPUT_MAX,
        PID_OUTPUT_MAX
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

    uint32_t last_control_us =
        micros();

    uint32_t last_print_ms = 0;

    while (true)
    {
        // ====================================================
        // Shared state snapshot
        // ====================================================

        ControlState local;

        portENTER_CRITICAL(
            &state_mux
        );

        local =
            state;

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

        // ====================================================
        // dt
        // ====================================================

        uint32_t now_us =
            micros();

        float dt =
            static_cast<float>(
                now_us -
                last_control_us
            ) /
            1000000.0f;

        last_control_us =
            now_us;

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

        if (local.system_stop ||
            get_timeout ||
            local.get1_target <
                0)
        {
            get1_pid.reset();

            setMotorPWM(
                GET1_PWM_CHANNEL,
                GET1_DIR_PIN,
                0,
                GET1_MOTOR_SIGN
            );
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
                // TARGET自体がLIMIT外なら
                // 一切動かさない
                get1_target_invalid =
                    true;

                get1_pid.reset();

                setMotorPWM(
                    GET1_PWM_CHANNEL,
                    GET1_DIR_PIN,
                    0,
                    GET1_MOTOR_SIGN
                );
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
                    get1_pid.reset();

                    setMotorPWM(
                        GET1_PWM_CHANNEL,
                        GET1_DIR_PIN,
                        0,
                        GET1_MOTOR_SIGN
                    );
                }
                else
                {
                    // ----------------------------------------
                    // PID
                    // ----------------------------------------

                    float output =
                        get1_pid.update(
                            local.get1_target,
                            get1_position,
                            dt
                        );

                    // ----------------------------------------
                    // HARD LIMIT
                    // ----------------------------------------

                    if (isPotLimitBlocking(
                            output,
                            get1_position,
                            GET1_OPEN_LIMIT,
                            GET1_CLOSE_LIMIT))
                    {
                        get1_limit_blocked =
                            true;

                        get1_pid.reset();

                        output = 0;
                    }

                    setMotorPWM(
                        GET1_PWM_CHANNEL,
                        GET1_DIR_PIN,
                        output,
                        GET1_MOTOR_SIGN
                    );
                }
            }
        }

        // ====================================================
        // GET2
        // ====================================================

        bool get2_limit_blocked =
            false;

        bool get2_target_invalid =
            false;

        if (local.system_stop ||
            get_timeout ||
            local.get2_target <
                0)
        {
            get2_pid.reset();

            setMotorPWM(
                GET2_PWM_CHANNEL,
                GET2_DIR_PIN,
                0,
                GET2_MOTOR_SIGN
            );
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

                get2_pid.reset();

                setMotorPWM(
                    GET2_PWM_CHANNEL,
                    GET2_DIR_PIN,
                    0,
                    GET2_MOTOR_SIGN
                );
            }
            else
            {
                int error =
                    local.get2_target -
                    get2_position;

                if (abs(error) <=
                    GET_POSITION_TOLERANCE)
                {
                    get2_pid.reset();

                    setMotorPWM(
                        GET2_PWM_CHANNEL,
                        GET2_DIR_PIN,
                        0,
                        GET2_MOTOR_SIGN
                    );
                }
                else
                {
                    float output =
                        get2_pid.update(
                            local.get2_target,
                            get2_position,
                            dt
                        );

                    if (isPotLimitBlocking(
                            output,
                            get2_position,
                            GET2_OPEN_LIMIT,
                            GET2_CLOSE_LIMIT))
                    {
                        get2_limit_blocked =
                            true;

                        get2_pid.reset();

                        output = 0;
                    }

                    setMotorPWM(
                        GET2_PWM_CHANNEL,
                        GET2_DIR_PIN,
                        output,
                        GET2_MOTOR_SIGN
                    );
                }
            }
        }

        // ====================================================
        // LIFT
        // ====================================================

        if (local.system_stop ||
            lift_timeout ||
            local.lift_target <
                0)
        {
            lift_pid.reset();

            setMotorPWM(
                LIFT_PWM_CHANNEL,
                LIFT_DIR_PIN,
                0,
                LIFT_MOTOR_SIGN
            );
        }
        else
        {
            int error =
                local.lift_target -
                lift_position;

            if (abs(error) <=
                LIFT_POSITION_TOLERANCE)
            {
                lift_pid.reset();

                setMotorPWM(
                    LIFT_PWM_CHANNEL,
                    LIFT_DIR_PIN,
                    0,
                    LIFT_MOTOR_SIGN
                );
            }
            else
            {
                float output =
                    lift_pid.update(
                        local.lift_target,
                        lift_position,
                        dt
                    );

                setMotorPWM(
                    LIFT_PWM_CHANNEL,
                    LIFT_DIR_PIN,
                    output,
                    LIFT_MOTOR_SIGN
                );
            }
        }

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
                "GET1 T:%4d P:%4d LIM:%d INV:%d | "
                "GET2 T:%4d P:%4d LIM:%d INV:%d | "
                "LIFT T:%4d P:%4d | "
                "AIR:%d | "
                "TO G:%d L:%d A:%d\n",

                local.get1_target,
                get1_position,
                get1_limit_blocked,
                get1_target_invalid,

                local.get2_target,
                get2_position,
                get2_limit_blocked,
                get2_target_invalid,

                local.lift_target,
                lift_position,

                local.air_target,

                get_timeout,
                lift_timeout,
                air_timeout
            );
        }

        // ====================================================
        // 10ms
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
    // -1ならリミット機能自体を無効化
    if (!arePotLimitsEnabled(
            open_limit,
            close_limit))
    {
        return true;
    }

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
    float output,
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

    // OPEN/CLOSEのどちらでADCが大きくなるかは
    // 問わない。
    //
    // 小さい側と大きい側を自動取得する。

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

    // --------------------------------------------------------
    // PID output > 0
    //
    // ADC値を増加させる方向
    // --------------------------------------------------------

    if (output > 0.0f &&
        current_position >=
            upper_limit)
    {
        return true;
    }

    // --------------------------------------------------------
    // PID output < 0
    //
    // ADC値を減少させる方向
    // --------------------------------------------------------

    if (output < 0.0f &&
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
    float output,
    int motor_sign
)
{
    // --------------------------------------------------------
    // Motor direction correction
    // --------------------------------------------------------

    output *=
        motor_sign;

    // --------------------------------------------------------
    // PID output limit
    // --------------------------------------------------------

    output =
        constrain(
            output,
            -PID_OUTPUT_MAX,
            PID_OUTPUT_MAX
        );

    // --------------------------------------------------------
    // DIR
    // --------------------------------------------------------

    if (output >= 0.0f)
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
    // PID -> PWM duty
    // --------------------------------------------------------

    // int duty =
    //     static_cast<int>(
    //         output /
    //         PID_OUTPUT_MAX *
    //         PWM_DUTY_LIMIT
    //     );

    // duty =
    //     constrain(
    //         duty,
    //         0,
    //         PWM_DUTY_LIMIT
    //     );

    int duty =
        constrain(
            static_cast<int>(output),
            0,
            PWM_DUTY_LIMIT
        );

    ledcWrite(
        pwm_channel,
        duty
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