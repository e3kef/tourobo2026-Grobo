#include <Arduino.h>

constexpr int SBUS_RX = 16;

HardwareSerial sbus(2);

uint8_t frame[25];
uint8_t byte_count = 0;
uint16_t channels[12];

uint32_t last_frame_us = 0;

void decodeSbus();

void setup()
{
    Serial.begin(115200);

    // S.BUS: 100000bps, 8bit, Even parity, 2 stop bit
    // 最後の true でRX信号を反転
    sbus.begin(100000, SERIAL_8E2, SBUS_RX, -1, true);
}

void loop()
{
    while (sbus.available())
    {
        uint8_t data = sbus.read();

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