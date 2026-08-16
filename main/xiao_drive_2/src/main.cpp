#include <Arduino.h>

#include "can.h"
#include "can_protocol.h"
#include "encoder.h"
#include "pid.h"

// =====================
// Setting
// =====================

// シールド基板準拠
constexpr int CAN_TX = 5;
constexpr int CAN_RX = 4;

// 足回りモータ番号
constexpr uint8_t MOTOR_INDEX = 2;

// Encoder
constexpr int ENCODER_A = 7;
constexpr int ENCODER_B = 6;

// Motor
constexpr int PWM_PIN = 20;
constexpr int DIR_PIN = 8;

// PWM
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 10;

// 10bit PWM: 0～1023
// 24Vで12Vモータを使うため約50%に制限→解除
constexpr int PWM_DUTY_LIMIT = 1023;

// Control
constexpr uint32_t CAN_TIMEOUT_MS = 100;
constexpr uint32_t CONTROL_PERIOD_MS = 10;

constexpr int16_t DRIVE_TARGET_MAX = 1000;
constexpr float MAX_RPM = 265.0f;

constexpr float PID_OUTPUT_MAX = 1000.0f;

// =====================
// PID gain
// =====================

// 仮値
constexpr float KP = 0.0f;
constexpr float KI = 0.0f;
constexpr float KD = 0.0f;

// =====================
// Shared variables
// =====================

int16_t target = 0;
int16_t mode = CAN_MODE_NORMAL;

bool system_stop = false;

uint32_t last_target_time = 0;

// 共有変数保護
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

// =====================
// PID
// =====================

PIDController drive_pid(KP, KI, KD);

// =====================
// Functions
// =====================

void taskCAN(void *arg);
void taskControl(void *arg);

void receiveCAN();

float targetToRPM(int16_t target);

void setMotorPWM(float pwm);

// =====================
// Setup
// =====================

void setup()
{
    Serial.begin(115200);

    // ---------------------
    // CAN
    // ---------------------

    if (!CAN_begin(CAN_TX, CAN_RX))
    {
        Serial.println("CAN init failed");

        while (1)
        {
            delay(100);
        }
    }

    Serial.println("CAN init OK");

    // ---------------------
    // Encoder
    // ---------------------

    Encoder_begin(
        ENCODER_A,
        ENCODER_B
    );

    // ---------------------
    // Motor
    // ---------------------

    pinMode(DIR_PIN, OUTPUT);

    ledcSetup(
        PWM_CHANNEL,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        PWM_PIN,
        PWM_CHANNEL
    );

    setMotorPWM(0);

    // ---------------------
    // PID
    // ---------------------

    drive_pid.setOutputLimits(
        -PID_OUTPUT_MAX,
        PID_OUTPUT_MAX
    );

    // ---------------------
    // CAN task
    // ---------------------

    xTaskCreatePinnedToCore(
        taskCAN,
        "CAN",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    // ---------------------
    // Control task
    // ---------------------

    xTaskCreatePinnedToCore(
        taskControl,
        "CONTROL",
        4096,
        nullptr,
        2,
        nullptr,
        0
    );

    Serial.println("Setup done");
}

// =====================
// Loop
// =====================

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// =====================
// CAN Task
// =====================

void taskCAN(void *arg)
{
    while (true)
    {
        // フレームが来るまで受信待ち
        // 周期Delayは入れない
        receiveCAN();
    }
}

// =====================
// Control Task
// =====================

void taskControl(void *arg)
{
    TickType_t last_wake_time =
        xTaskGetTickCount();

    uint32_t last_control_us =
        micros();

    while (true)
    {
        // ---------------------
        // Shared data snapshot
        // ---------------------

        int16_t target_local;
        bool stop_local;
        uint32_t target_time_local;

        portENTER_CRITICAL(&state_mux);

        target_local = target;
        stop_local = system_stop;
        target_time_local = last_target_time;

        portEXIT_CRITICAL(&state_mux);

        // ---------------------
        // Encoder
        // ---------------------

        // 停止中でも必ず10msごとに読む
        // Encoder_getRPM()内部でカウントを0へ戻すため
        float current_rpm =
            Encoder_getRPM();

        // ---------------------
        // dt
        // ---------------------

        uint32_t now_us = micros();

        float dt =
            (now_us - last_control_us)
            / 1000000.0f;

        last_control_us = now_us;

        // ---------------------
        // Safety
        // ---------------------

        bool no_target =
            (target_time_local == 0);

        bool timeout =
            !no_target &&
            (millis() - target_time_local
             > CAN_TIMEOUT_MS);

        if (no_target ||
            timeout ||
            stop_local)
        {
            drive_pid.reset();

            setMotorPWM(0);
        }
        else
        {
            // ---------------------
            // Target
            // ---------------------

            float target_rpm =
                targetToRPM(target_local);

            // ---------------------
            // Velocity PID
            // ---------------------

            float output =
                drive_pid.update(
                    target_rpm,
                    current_rpm,
                    dt
                );

            // ---------------------
            // PWM
            // ---------------------

            setMotorPWM(output);

            // ---------------------
            // Debug
            // ---------------------

            /*
            Serial.printf(
                "target:%d targetRPM:%.2f rpm:%.2f out:%.2f dt:%.4f\n",
                target_local,
                target_rpm,
                current_rpm,
                output,
                dt
            );
            */
        }

        // ---------------------
        // 10 ms fixed period
        // ---------------------

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(
                CONTROL_PERIOD_MS
            )
        );
    }
}

// =====================
// CAN receive
// =====================

void receiveCAN()
{
    uint16_t id;
    int16_t data[4];

    // 最大1000ms待機するが、
    // CANが来ればその時点ですぐreturnする
    if (!CAN_receive(
            id,
            data,
            1000))
    {
        return;
    }

    switch (id)
    {
        // ---------------------
        // SYSTEM STOP
        // ---------------------

        case CAN_ID_SYSTEM_STOP:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            system_stop = true;

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }

        // ---------------------
        // MODE
        // ---------------------

        case CAN_ID_MODE_SETTING:
        {
            portENTER_CRITICAL(
                &state_mux
            );

            mode = data[0];

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }

        // ---------------------
        // DRIVE TARGET
        // ---------------------

        case CAN_ID_DRIVE_TARGET:
        {
            uint32_t now =
                millis();

            portENTER_CRITICAL(
                &state_mux
            );

            target =
                data[MOTOR_INDEX - 1];

            last_target_time =
                now;

            portEXIT_CRITICAL(
                &state_mux
            );

            break;
        }
    }
}

// =====================
// Target -> RPM
// =====================

float targetToRPM(int16_t target_value)
{
    if (target_value > DRIVE_TARGET_MAX)
    {
        target_value =
            DRIVE_TARGET_MAX;
    }
    else if (
        target_value < -DRIVE_TARGET_MAX)
    {
        target_value =
            -DRIVE_TARGET_MAX;
    }

    return
        static_cast<float>(target_value)
        / DRIVE_TARGET_MAX
        * MAX_RPM;
}

// =====================
// Motor
// =====================

void setMotorPWM(float pwm)
{
    // PID output limit
    if (pwm > PID_OUTPUT_MAX)
    {
        pwm = PID_OUTPUT_MAX;
    }
    else if (pwm < -PID_OUTPUT_MAX)
    {
        pwm = -PID_OUTPUT_MAX;
    }

    // ---------------------
    // Direction
    // ---------------------

    if (pwm >= 0.0f)
    {
        digitalWrite(
            DIR_PIN,
            HIGH
        );
    }
    else
    {
        digitalWrite(
            DIR_PIN,
            LOW
        );

        pwm = -pwm;
    }

    // ---------------------
    // PID output -> duty
    // ---------------------

    int duty =
        static_cast<int>(
            pwm
            / PID_OUTPUT_MAX
            * PWM_DUTY_LIMIT
        );

    if (duty > PWM_DUTY_LIMIT)
    {
        duty = PWM_DUTY_LIMIT;
    }

    ledcWrite(
        PWM_CHANNEL,
        duty
    );
}