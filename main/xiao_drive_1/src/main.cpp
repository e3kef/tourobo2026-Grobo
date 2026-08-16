#include <Arduino.h>

#include "can.h"
#include "can_protocol.h"
#include "pid.h"

// =====================
// Setting
// =====================

// シールド基板はTX5RX4
constexpr int CAN_TX = 4; 
constexpr int CAN_RX = 5;  

constexpr uint8_t MOTOR_INDEX = 1;

constexpr uint32_t CAN_TIMEOUT_MS = 10000;
constexpr uint32_t TASK_PERIOD_MS = 10;

// PID gain
constexpr float KP = 0.0f;
constexpr float KI = 0.0f;
constexpr float KD = 0.0f;


// =====================
// Variables
// =====================

int16_t target = 0;

int16_t mode = CAN_MODE_NORMAL;

bool system_stop = false;

uint32_t last_target_time = 0;

PIDController drive_pid(KP, KI, KD);

// 共有変数保護
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;


// =====================
// Functions
// =====================

void taskCAN(void *arg);
void taskControl(void *arg);

void receiveCAN();

float getMotorSpeed();
void setMotorPWM(float pwm);


// =====================
// Setup
// =====================

void setup()
{
    Serial.begin(115200);

    if (!CAN_begin(CAN_TX, CAN_RX))
    {
        Serial.println("CAN init failed");
        while (1)
        {
            delay(100);
        }
    }

    Serial.println("CAN init OK");


    drive_pid.setOutputLimits(-1000.0f, 1000.0f);

    // =====================
    // CAN task
    // =====================

    xTaskCreatePinnedToCore(
        taskCAN,
        "CAN",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );


    // =====================
    // Control task
    // =====================

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
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        receiveCAN();

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(TASK_PERIOD_MS)
        );
    }
}


// =====================
// Control Task
// =====================

void taskControl(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    uint32_t last_control_us = micros();

    while (true)
    {
        // ---------------------
        // 共有データ取得
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
        // Safety
        // ---------------------

        bool timeout =
            millis() - target_time_local > CAN_TIMEOUT_MS;

        if (timeout || stop_local)
        {
            drive_pid.reset();

            setMotorPWM(0);
        }
        else
        {
            // ---------------------
            // dt
            // ---------------------

            uint32_t now_us = micros();

            float dt =
                (now_us - last_control_us) / 1000000.0f;

            last_control_us = now_us;


            // ---------------------
            // Encoder
            // ---------------------

            float current_speed = getMotorSpeed();


            // ---------------------
            // PID
            // ---------------------

            float output =
                drive_pid.update(
                    static_cast<float>(target_local),
                    current_speed,
                    dt
                );


            // ---------------------
            // PWM
            // ---------------------

            setMotorPWM(output);


            // debug
            // Serial.printf(
            //     "target:%d speed:%.2f out:%.2f dt:%.4f\n",
            //     target_local,
            //     current_speed,
            //     output,
            //     dt
            // );
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


// =====================
// CAN receive
// =====================

void receiveCAN()
{
    uint16_t id;
    int16_t data[4];

    while (CAN_receive(id, data))
    {   
        // // --- for debug ---
        // Serial.printf(
        //     "RX ID: 0x%03X data: %d %d %d %d\n",
        //     id,
        //     data[0],
        //     data[1],
        //     data[2],
        //     data[3]
        // );
        // // --- for debug ---

        switch (id)
        {
            case CAN_ID_SYSTEM_STOP:
                system_stop = true;
                break;


            case CAN_ID_MODE_SETTING:
                mode = data[0];
                break;


            case CAN_ID_DRIVE_TARGET:
                target = data[MOTOR_INDEX - 1];
                last_target_time = millis();
                break;
        }
    }
}

// =====================
// Encoder
// =====================

float getMotorSpeed()
{
    // AMT102-Vの速度取得をここに実装
    return 0.0f;
}


// =====================
// Motor
// =====================

void setMotorPWM(float pwm)
{
    // CytronへのPWM出力をここに実装
}