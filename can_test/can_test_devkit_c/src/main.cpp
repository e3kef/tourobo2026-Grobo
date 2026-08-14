#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 21
#define CAN_RX 22

#define DEVKITC_TX_ID 0x100
#define XIAO_TX_ID   0x101

#define TEST_COUNT       10
#define MAX_RETRY_COUNT  10

CanFrame txFrame;
CanFrame rxFrame;

bool can_transmit(uint8_t transmit_data);
bool can_receive(uint8_t expected_data);



// main

void setup(){
    Serial.begin(115200);

    // Start CAN(500kbps)
    if(!ESP32Can.begin(
        TWAI_SPEED_500KBPS,
        CAN_TX,
        CAN_RX
    )){
        Serial.println("CAN start failed");
        while(true){
            delay(1000);
        }
    }

    Serial.println("CAN transmitter started.");
}

void loop(){
    static bool test_finished = false;

    if(test_finished){
        delay(1000);
        return;
    }

    for(uint8_t transmit_count = 0; transmit_count < TEST_COUNT; transmit_count++){
        Serial.printf("\nTest %u/%u", transmit_count + 1, TEST_COUNT);

        if(!can_transmit(transmit_count)){
            Serial.println("Transmission failed.");

            while(true){
                delay(1000);
            }
        }

        if(!can_receive(transmit_count)){
            Serial.println("Reception failed.");

            while(true){
                delay(1000);
            }
        }

        Serial.printf("Communication succeed: data=%u\n", transmit_count);
        delay(100);
    }

    Serial.println();
    Serial.println("All CAN communication tests succeed.");

    test_finished = true;
}




// functions

bool can_transmit(uint8_t transmit_data){
    txFrame = {};

    // settings
    txFrame.identifier       = DEVKITC_TX_ID;
    txFrame.extd             = 0;      // 11bit標準ID
    txFrame.rtr              = 0;      // データフレーム
    txFrame.data_length_code = 1;      // データ長1byte
    txFrame.data[0]          = transmit_data;

    for(uint8_t failed_count = 0; failed_count < MAX_RETRY_COUNT; failed_count++){
        if (ESP32Can.writeFrame(txFrame, 100)) {
            Serial.printf(
                "TX queued by DevKitC: ID=0x%03X data=%u\n",
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

bool can_receive(uint8_t expected_data){
    for(uint8_t retry_count = 0; retry_count < MAX_RETRY_COUNT; retry_count++){
        rxFrame = {};

        // wait(max:100ms)
        if(!ESP32Can.readFrame(rxFrame, 100)){
            Serial.println("No data received.");
            continue;
        }

        Serial.printf("RX by DevKitC: ID=0x%03X DLC=%u data=", rxFrame.identifier, rxFrame.data_length_code);

        // Print data
        for(uint8_t i = 0; i < rxFrame.data_length_code; i++){
            Serial.printf("%02X", rxFrame.data[i]);
        }
        Serial.println();

        // Check data if it from xiao
        if(rxFrame.identifier != XIAO_TX_ID){
            Serial.println("Unexpected CAN ID.");
            continue;
        }

        if(rxFrame.data_length_code < 1){
            Serial.println("Invalid data length.");
            continue;
        }

        // Check data if it matches with transmitted data from xiao
        if(rxFrame.data[0] != expected_data){
            Serial.printf("Data mismatch: expected=%u received=%u", expected_data, rxFrame.data[0]);
            continue;
        }

        return true;
    }

    return false;
}