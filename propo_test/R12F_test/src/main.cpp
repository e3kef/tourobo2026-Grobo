#include <Arduino.h>

// #include "SBUSReceiver.h"

constexpr int SBUS_RX = 16;

HardwareSerial hardware_serial(2);

// SBUSReceiver sbus(hardware_serial, SBUS_RX, -1);  // RX=12, TX=13

uint8_t frame[25];
uint8_t byte_count = 0;
uint16_t channels[12];

uint32_t last_frame_us = 0;

// unsigned long lastTime = 0;

void decodeSbus();

void setup()
{
    Serial.begin(115200);

    // S.BUS: 100000bps, 8bit, Even parity, 2 stop bit
    // 最後の true でRX信号を反転
    hardware_serial.begin(100000, SERIAL_8E2, SBUS_RX, -1, true);

    // hardware_serial.begin();
    Serial.println("S.BUS Receiver Ready");
}

void loop()
{
    while (hardware_serial.available())
    {
        uint8_t data = hardware_serial.read();

        // Header待ち
        if (byte_count == 0 && data != 0x0F)
            continue;

        frame[byte_count] = data;
        byte_count++;

        if (byte_count == 25)
        {
            uint32_t now_us = micros();

            if (last_frame_us != 0)
            {
                uint32_t period_us = now_us - last_frame_us;

                Serial.printf(
                    "S.BUS period: %lu us (%.2f ms)\n",
                    period_us,
                    period_us / 1000.0f
                );
            }

            last_frame_us = now_us;

            decodeSbus();

            byte_count = 0;
        }
    }

    // if (sbus.readFrame()) {
    //     unsigned long now = micros();
    //     Serial.print("DT:");
    //     Serial.print(now - lastTime);
    //     for (int i=0; i < 16; i++) {
    //     Serial.print("CH");
    //     Serial.print(i+1);
    //     Serial.print(":");
    //     Serial.print(sbus.getChannel(i));
    //     Serial.print(" ");
    //     }
    //     Serial.print("  CH17:");
    //     Serial.print(sbus.getCh17());
    //     Serial.print("  CH18:");
    //     Serial.print(sbus.getCh18());
    //     Serial.print("  FrameLost:");
    //     Serial.print(sbus.isFrameLost());
    //     Serial.print("  Failsafe:");
    //     Serial.print(sbus.isFailsafe());
    //     Serial.print("  LostConnection:");
    //     Serial.println(sbus.isLostConnection());
    //     lastTime = now;
    // }
} 

void decodeSbus()
{

    for (int ch = 0; ch < 12; ch++)
    {
        int bit_index = ch * 11;
        int byte_index = 1 + bit_index / 8;
        int shift = bit_index % 8;

        uint32_t data =
            ((uint32_t)frame[byte_index]) |
            ((uint32_t)frame[byte_index + 1] << 8) |
            ((uint32_t)frame[byte_index + 2] << 16);

        channels[ch] = (data >> shift) & 0x07FF;
    }
}