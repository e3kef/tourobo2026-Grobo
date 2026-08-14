#include <ESP32-TWAI-CAN.hpp>

// XIAO ESP32-C3
// D2 = GPIO4
// D3 = GPIO5
#define CAN_TX D2
#define CAN_RX D3

#define DEVKIT_TX_ID 0x100
#define XIAO_TX_ID   0x101

#define MAX_RETRY_COUNT 10

CanFrame txFrame;
CanFrame rxFrame;


bool can_receive(uint8_t &received_data)
{
    rxFrame = {};

    // 最大1000 ms待機
    if (!ESP32Can.readFrame(rxFrame, 1000)) {
        return false;
    }

    Serial.printf(
        "RX by XIAO: ID=0x%03X DLC=%u data=",
        rxFrame.identifier,
        rxFrame.data_length_code
    );

    for (uint8_t i = 0; i < rxFrame.data_length_code; i++) {
        Serial.printf("%02X ", rxFrame.data[i]);
    }

    Serial.println();

    // DevKitCからのフレームだけ処理
    if (rxFrame.identifier != DEVKIT_TX_ID) {
        Serial.println("Unexpected CAN ID.");
        return false;
    }

    if (rxFrame.data_length_code < 1) {
        Serial.println("Invalid data length.");
        return false;
    }

    received_data = rxFrame.data[0];

    return true;
}


bool can_transmit(uint8_t transmit_data)
{
    txFrame = {};

    txFrame.identifier       = XIAO_TX_ID;
    txFrame.extd             = 0;
    txFrame.rtr              = 0;
    txFrame.data_length_code = 1;
    txFrame.data[0]          = transmit_data;

    for (uint8_t failed_count = 0;
         failed_count < MAX_RETRY_COUNT;
         failed_count++) {

        if (ESP32Can.writeFrame(txFrame, 100)) {
            Serial.printf(
                "TX queued by XIAO: ID=0x%03X data=%u\n",
                txFrame.identifier,
                transmit_data
            );

            return true;
        }

        Serial.printf(
            "TX queue failed: retry %u/%u\n",
            failed_count + 1,
            MAX_RETRY_COUNT
        );

        delay(10);
    }

    return false;
}


void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Starting XIAO ESP32-C3 CAN responder.");

    if (!ESP32Can.begin(
        TWAI_SPEED_500KBPS,
        CAN_TX,
        CAN_RX
    )) {
        Serial.println("CAN start failed.");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("CAN started.");
}


void loop()
{
    uint8_t received_data = 0;

    if (!can_receive(received_data)) {
        return;
    }

    // 受信したデータをそのままDevKitCへ返す
    if (!can_transmit(received_data)) {
        Serial.println("Reply transmission failed.");

        while (true) {
            delay(1000);
        }
    }

    Serial.printf(
        "Echo reply succeeded: data=%u\n\n",
        received_data
    );
}