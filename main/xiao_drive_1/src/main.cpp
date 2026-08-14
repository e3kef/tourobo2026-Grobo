#include <Arduino.h>

#include "can.h"
#include "can_protocol.h"

// =====================
// Setting
// =====================

constexpr int CAN_TX = 4; 
constexpr int CAN_RX = 5;  

constexpr uint8_t MOTOR_INDEX = 1;

constexpr uint32_t CAN_TIMEOUT_MS = 2000;


// =====================
// Variables
// =====================

int16_t target = 0;

int16_t mode = CAN_MODE_NORMAL;

bool system_stop = false;

uint32_t last_target_time = 0;


// =====================
// Functions
// =====================

void receiveCAN();


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
}


// =====================
// Loop
// =====================

void loop()
{
    receiveCAN();

    // DRIVE_TARGET通信断
    if (millis() - last_target_time > CAN_TIMEOUT_MS)
    {
        target = 0;
    }

    // 非常停止
    if (system_stop)
    {
        target = 0;
    }

    if(target != 0)
        Serial.println(target);

    // 後でここに
    // encoder
    // PID
    // PWM
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