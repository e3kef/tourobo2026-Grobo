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
constexpr int POT_GET1_PIN = 35;
constexpr int POT_GET2_PIN = 25;
constexpr int POT_LIFT_PIN = 14;

// ============================================================
// Motor pins
// ============================================================

// GET1
constexpr int GET1_PWM_PIN = 33;
constexpr int GET1_DIR_PIN = 32;

// GET2
constexpr int GET2_PWM_PIN = 18;
constexpr int GET2_DIR_PIN = 19;

// LIFT
constexpr int LIFT_PWM_PIN = 23;
constexpr int LIFT_DIR_PIN = 4;

// ============================================================
// Air cylinder
// ============================================================
constexpr int AIR_PIN = 13;
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

constexpr float GET_PID_OUTPUT_LIMIT =
    static_cast<float>(PWM_DUTY_LIMIT - 200);

constexpr float LIFT_PID_OUTPUT_LIMIT =
    static_cast<float>(PWM_DUTY_LIMIT);

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
constexpr int LIFT_POSITION_TOLERANCE = 3;

// -1 = 制御無効
constexpr int16_t POSITION_TARGET_DISABLE = -1;

// ============================================================
// GET HARD LIMIT
// ============================================================
constexpr int GET1_OPEN_LIMIT  = 1940;
constexpr int GET1_CLOSE_LIMIT = 2450;

constexpr int GET2_OPEN_LIMIT  = 2300;
constexpr int GET2_CLOSE_LIMIT = 1728;

// ============================================================
// PID gain
// ============================================================
//
// まずPのみで動作確認。
// 出力単位はそのままPWM duty。
//
// 例:
// error = 500
// Kp = 0.5
// -> 約250 duty
//
// ============================================================

// GET1
constexpr float GET1_KP = 1.2f;
constexpr float GET1_KI = 1.0f;
constexpr float GET1_KD = 0.0f;

// GET2
constexpr float GET2_KP = 1.1f;
constexpr float GET2_KI = 1.0f;
constexpr float GET2_KD = 0.0f;

// LIFT
constexpr float LIFT_KP = 0.5f;
constexpr float LIFT_KI = 0.5f;
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
    GET1_KP,
    GET1_KI,
    GET1_KD
);

PIDController get2_pid(
    GET2_KP,
    GET2_KI,
    GET2_KD
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
    // PID
    // ========================================================

    // 現在のwithoutPID版と同じ最大出力に制限
    get1_pid.setOutputLimits(
        -GET_PID_OUTPUT_LIMIT,
        GET_PID_OUTPUT_LIMIT
    );

    get2_pid.setOutputLimits(
        -GET_PID_OUTPUT_LIMIT,
        GET_PID_OUTPUT_LIMIT
    );

    lift_pid.setOutputLimits(
        -LIFT_PID_OUTPUT_LIMIT,
        LIFT_PID_OUTPUT_LIMIT
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

    // ========================================================
    // 到達ラッチ
    // ========================================================

    int16_t last_get1_target =
        POSITION_TARGET_DISABLE;

    int16_t last_get2_target =
        POSITION_TARGET_DISABLE;

    int16_t last_lift_target =
        POSITION_TARGET_DISABLE;

    bool get1_target_reached =
        false;

    bool get2_target_reached =
        false;

    bool lift_target_reached =
        false;

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

        // タスク停止等でdtが異常になった場合は
        // 通常の10msとして扱う
        if (dt < 0.001f ||
            dt > 0.050f)
        {
            dt =
                static_cast<float>(
                    CONTROL_PERIOD_MS
                ) /
                1000.0f;
        }

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

        float get1_output =
            0.0f;

        // ----------------------------------------------------
        // Target change
        //
        // -1も含めて変更を検出する。
        // DISABLE -> 同じ以前のtarget
        // となった場合にも確実に再アームされる。
        // ----------------------------------------------------

        if (local.get1_target !=
            last_get1_target)
        {
            last_get1_target =
                local.get1_target;

            get1_target_reached =
                false;

            get1_pid.reset();
        }

        if (local.system_stop ||
            get_timeout ||
            local.get1_target ==
                POSITION_TARGET_DISABLE)
        {
            get1_pid.reset();
            get1_output = 0.0f;
        }
        else if (get1_target_reached)
        {
            // 一度到達したら次のターゲット変更まで停止
            get1_pid.reset();
            get1_output = 0.0f;
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
                get1_target_invalid =
                    true;

                get1_pid.reset();

                get1_output =
                    0.0f;
            }
            else
            {
                int error =
                    local.get1_target -
                    get1_position;

                    // ----------------------------------------
                    // PID
                    // ----------------------------------------

                    get1_output =
                        get1_pid.update(
                            static_cast<float>(
                                local.get1_target
                            ),
                            static_cast<float>(
                                get1_position
                            ),
                            dt
                        );

                    // ----------------------------------------
                    // HARD LIMIT
                    //
                    // PID outputの符号
                    // + : ADC増加方向
                    // - : ADC減少方向
                    //
                    // motor_signを掛ける前に判定する。
                    // ----------------------------------------

                    if (isPotLimitBlocking(
                            get1_output,
                            get1_position,
                            GET1_OPEN_LIMIT,
                            GET1_CLOSE_LIMIT))
                    {
                        get1_limit_blocked =
                            true;

                        get1_pid.reset();

                        get1_output =
                            0.0f;
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

        float get2_output =
            0.0f;

        // ----------------------------------------------------
        // Target change
        // ----------------------------------------------------

        if (local.get2_target !=
            last_get2_target)
        {
            last_get2_target =
                local.get2_target;

            get2_target_reached =
                false;

            get2_pid.reset();
        }

        if (local.system_stop ||
            get_timeout ||
            local.get2_target ==
                POSITION_TARGET_DISABLE)
        {
            get2_pid.reset();
            get2_output = 0.0f;
        }
        else if (get2_target_reached)
        {
            // GET1ではなくGET2自身のラッチを見る
            get2_pid.reset();
            get2_output = 0.0f;
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

                get2_output =
                    0.0f;
            }
            else
            {
                int error =
                    local.get2_target -
                    get2_position;

                // ----------------------------------------
                // PID
                // ----------------------------------------

                get2_output =
                    get2_pid.update(
                        static_cast<float>(
                            local.get2_target
                        ),
                        static_cast<float>(
                            get2_position
                        ),
                        dt
                    );

                // ----------------------------------------
                // HARD LIMIT
                // ----------------------------------------

                if (isPotLimitBlocking(
                        get2_output,
                        get2_position,
                        GET2_OPEN_LIMIT,
                        GET2_CLOSE_LIMIT))
                {
                    get2_limit_blocked =
                        true;

                    get2_pid.reset();

                    get2_output =
                        0.0f;
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

        float lift_output =
            0.0f;

        // ----------------------------------------------------
        // Target change
        // ----------------------------------------------------

        if (local.lift_target !=
            last_lift_target)
        {
            last_lift_target =
                local.lift_target;

            lift_target_reached =
                false;

            lift_pid.reset();
        }

        if (local.system_stop ||
            lift_timeout ||
            local.lift_target ==
                POSITION_TARGET_DISABLE)
        {
            lift_pid.reset();

            lift_output =
                0.0f;
        }
        else if (lift_target_reached)
        {
            // 一度到達したら次の目標値が来るまで停止
            lift_pid.reset();

            lift_output =
                0.0f;
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

                lift_pid.reset();

                lift_output =
                    0.0f;
            }
            else
            {
                // --------------------------------------------
                // PID
                // --------------------------------------------

                lift_output =
                    lift_pid.update(
                        static_cast<float>(
                            local.lift_target
                        ),
                        static_cast<float>(
                            lift_position
                        ),
                        dt
                    );
            }
        }

        setMotorPWM(
            LIFT_PWM_CHANNEL,
            LIFT_DIR_PIN,
            lift_output,
            LIFT_MOTOR_SIGN
        );

        // ====================================================
        // AIR
        // ====================================================

        if (local.system_stop ||
            air_timeout)
        {
            // 通信断では開
            setAirCylinder(
                true
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
                "GET1 T:%4d P:%4d OUT:%7.1f R:%d LIM:%d INV:%d | "
                "GET2 T:%4d P:%4d OUT:%7.1f R:%d LIM:%d INV:%d | "
                "LIFT T:%4d P:%4d OUT:%7.1f R:%d | "
                "AIR:%d | "
                "TO G:%d L:%d A:%d | "
                "dt:%.4f\n",

                local.get1_target,
                get1_position,
                get1_output,
                get1_target_reached,
                get1_limit_blocked,
                get1_target_invalid,

                local.get2_target,
                get2_position,
                get2_output,
                get2_target_reached,
                get2_limit_blocked,
                get2_target_invalid,

                local.lift_target,
                lift_position,
                lift_output,
                lift_target_reached,

                local.air_target,

                get_timeout,
                lift_timeout,
                air_timeout,

                dt
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
    // target > current
    // ADC値を増加させる方向
    if (output > 0.0f &&
        current_position >=
            upper_limit)
    {
        return true;
    }

    // output < 0
    // target < current
    // ADC値を減少させる方向
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
        static_cast<float>(
            motor_sign
        );

    // --------------------------------------------------------
    // Absolute hardware output limit
    // --------------------------------------------------------

    output =
        constrain(
            output,
            -static_cast<float>(
                PWM_DUTY_LIMIT
            ),
            static_cast<float>(
                PWM_DUTY_LIMIT
            )
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
    // PWM
    // --------------------------------------------------------

    int duty =
        constrain(
            static_cast<int>(
                output
            ),
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