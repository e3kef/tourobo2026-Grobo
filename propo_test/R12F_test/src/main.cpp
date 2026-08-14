#include <Arduino.h>

constexpr int SBUS_RX = 16;

HardwareSerial sbus(2);

uint8_t frame[25];
uint8_t byte_count = 0;

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
            for (int i = 0; i < 25; i++)
            {
                Serial.printf("%02X ", frame[i]);
            }

            Serial.println();
            byte_count = 0;
        }
    }
}