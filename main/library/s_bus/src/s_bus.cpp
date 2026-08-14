#include "s_bus.h"

namespace
{
    constexpr int SBUS_RX = 16;

    HardwareSerial sbus(2);

    uint8_t frame[25];
    uint8_t byte_count = 0;

    uint16_t channels[12];

    uint32_t last_receive_time = 0;

    constexpr uint32_t TIMEOUT_MS = 100;

    void decode()
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
}

void SBus::begin()
{
    sbus.begin(
        100000,
        SERIAL_8E2,
        SBUS_RX,
        -1,
        true
    );
}

void SBus::update()
{
    while (sbus.available())
    {
        uint8_t data = sbus.read();

        if (byte_count == 0 && data != 0x0F)
            continue;

        frame[byte_count] = data;
        byte_count++;

        if (byte_count == 25)
        {
            decode();

            last_receive_time = millis();
            byte_count = 0;
        }
    }
}

uint16_t SBus::getChannel(uint8_t ch)
{
    if (ch >= 12)
        return 0;

    return channels[ch];
}

bool SBus::isConnected()
{
    return millis() - last_receive_time < TIMEOUT_MS;
}