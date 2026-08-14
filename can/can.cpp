#include "CAN.h"

#include <ESP32-TWAI-CAN.hpp>
#include <cstring>


bool CAN_begin(int txPin, int rxPin)
{
    return ESP32Can.begin(
        TWAI_SPEED_500KBPS,
        txPin,
        rxPin
    );
}


bool CAN_send(uint16_t id, const int16_t data[4])
{
    CanFrame frame = {};

    frame.identifier = id;
    frame.extd = 0;
    frame.data_length_code = 8;

    memcpy(frame.data, data, 8);

    return ESP32Can.writeFrame(frame);
}


bool CAN_receive(uint16_t &id, int16_t data[4], uint32_t timeout)
{
    CanFrame frame = {};

    if (!ESP32Can.readFrame(frame, timeout)) {
        return false;
    }

    if (frame.extd || frame.data_length_code != 8) {
        return false;
    }

    id = frame.identifier;

    memcpy(data, frame.data, 8);

    return true;
}