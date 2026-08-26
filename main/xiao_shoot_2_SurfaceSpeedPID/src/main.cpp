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

// 射出モータ番号
constexpr uint8_t MOTOR_INDEX = 2;

// Encoder
constexpr int ENCODER_A = 7;
constexpr int ENCODER_B = 6;

// Motor
constexpr int PWM_PIN = 8;
constexpr int DIR_PIN = 20;

// PWM     
constexpr int PWM_CHANNEL = 0;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 10;

// 10bit PWM: 0～1023
// 24V駆動/18V定格 -> 18V相当MAX
constexpr int PWM_DUTY_LIMIT = 767;

// Control
constexpr uint32_t CAN_TIMEOUT_MS = 100;
constexpr uint32_t CONTROL_PERIOD_MS = 10;

constexpr float PID_OUTPUT_MAX = 1000.0f;

// Roller surface speed = (14 / 15) * PI * RPM
constexpr float PI_F = 3.14159265358979323846f;
constexpr float SURFACE_SPEED_PER_RPM =
    (14.0f / 15.0f) * PI_F;



// =====================
// PID gain
// =====================

// 仮値
constexpr float KP = 0.15f;
constexpr float KI = 3.5f;
constexpr float KD = 0.0005f;

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

PIDController shoot_pid(KP, KI, KD);

// =====================
// Functions
// =====================

void taskCAN(void *arg);
void taskControl(void *arg);

void receiveCAN();

int setMotorPWM(float pwm);

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

    shoot_pid.setOutputLimits(
        0.0f,
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
        // dt
        // ---------------------

        uint32_t now_us = micros();

        float dt =
            (now_us - last_control_us)
            / 1000000.0f;

        last_control_us = now_us;

        if(dt <= 0.0f){
            dt =
            CONTROL_PERIOD_MS / 1000.0f;
        }

        // ---------------------
        // Encoder
        // ---------------------

        // 停止中でも必ず10msごとに読む
        // Encoder_getRPM()内部でカウントを0へ戻すため
        float current_rpm =
            -(Encoder_getRPM(dt));

        // RPM -> ローラー周速度
        float current_surface_speed =
            current_rpm * SURFACE_SPEED_PER_RPM;

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
            stop_local ||
            target_local == 0)
        {
            shoot_pid.reset();

            setMotorPWM(0);
        }
        else
        {
            // ---------------------
            // Target
            // ---------------------

            float target_surface_speed =
                static_cast<float>(target_local);

            // ---------------------
            // Velocity PID
            // ---------------------

            float output =
                shoot_pid.update(
                    target_surface_speed,
                    current_surface_speed,
                    dt
                );

            // ---------------------
            // PWM
            // ---------------------

            int output_duty = setMotorPWM(output);

            // ---------------------
            // Debug
            // ---------------------

            static uint32_t last_debug_ms = 0;
            uint32_t now_ms = millis();

            if (now_ms - last_debug_ms >= 100)
            {
                last_debug_ms = now_ms;

                Serial.printf(
                    "target=%.1f, current=%.1f, rpm=%.1f, "
                    "dt=%.4f, pid=%.1f, duty=%d\n",
                    target_surface_speed,
                    current_surface_speed,
                    current_rpm,
                    dt,
                    output,
                    output_duty
                );
            }
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

        case CAN_ID_SHOOT_TARGET:
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
// Motor
// =====================

int setMotorPWM(float pwm)
{
    // PID output limit
    if (pwm > PID_OUTPUT_MAX)
    {
        pwm = PID_OUTPUT_MAX;
    }
    else if (pwm < 0)
    {
        pwm = 0;
    }

    // ---------------------
    // Direction
    // ---------------------

    digitalWrite(
        DIR_PIN, HIGH
    );

    // if (pwm >= 0.0f)
    // {
    //     digitalWrite(
    //         DIR_PIN,
    //         HIGH
    //     );
    // }
    // else
    // {
    //     digitalWrite(
    //         DIR_PIN,
    //         LOW
    //     );

    //     pwm = -pwm;
    // }

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

    return duty;
}