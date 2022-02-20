#pragma once

#include <SoftwareSerial.h>

#include "Types.h"

namespace SerialCom {
    constexpr static const uint8_t PIN_UART_RX = 4; // D2 on Wemos D1 Mini
    constexpr static const uint8_t PIN_UART_TX = 13; // UNUSED

    SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

    uint8_t serialRxBuf[255];
    uint8_t rxBufIdx = 0;

    void setup() {
        sensorSerial.begin(9600);
    }

    void clearRxBuf() {
        // Clear everything for the next message
        memset(serialRxBuf, 0, sizeof(serialRxBuf));
        rxBufIdx = 0;
    }

    void parseState(particleSensorState_t& state) {
        state.measurementIdx = (state.measurementIdx + 1) % 5;

        const uint16_t pm1  = (serialRxBuf[9] << 8 ^ 1) | serialRxBuf[10];
        Serial.printf("Received PM 1 reading: %d\n", pm1);
        state.pm1Measurements[state.measurementIdx] = pm1;

        const uint16_t pm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];
        Serial.printf("Received PM 2.5 reading: %d\n", pm25);
        state.pm25Measurements[state.measurementIdx] = pm25;

        const uint16_t pm10 = (serialRxBuf[13] << 8 ^ 1) | serialRxBuf[14];
        Serial.printf("Received PM 10 reading: %d\n", pm10);
        state.pm10Measurements[state.measurementIdx] = pm10;

        if (state.measurementIdx == 0) {
            float avgPM1 = 0.0f;
            for (uint8_t i = 0; i < 5; ++i) {
                avgPM1 += state.pm1Measurements[i] / 5.0f;
            }
            state.avgPM1 = avgPM1;
            Serial.printf("New Avg PM1: %d\n", state.avgPM1);

            float avgPM25 = 0.0f;
            for (uint8_t i = 0; i < 5; ++i) {
                avgPM25 += state.pm25Measurements[i] / 5.0f;
            }
            state.avgPM25 = avgPM25;
            Serial.printf("New Avg PM25: %d\n", state.avgPM25);

            float avgPM10 = 0.0f;
            for (uint8_t i = 0; i < 5; ++i) {
                avgPM10 += state.pm10Measurements[i] / 5.0f;
            }
            state.avgPM10 = avgPM10;
            Serial.printf("New Avg PM10: %d\n", state.avgPM10);

            state.valid = true;
        }

        clearRxBuf();
    }

    bool isValidHeader() {
        bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

        if (!headerValid) {
            Serial.println("Received message with invalid header.");
        }

        return headerValid;
    }

    bool isValidChecksum() {
        uint8_t checksum = 0;

        for (uint8_t i = 0; i < 20; i++) {
            checksum += serialRxBuf[i];
        }

        if (checksum != 0) {
            Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\n", checksum);
        }

        return checksum == 0;
    }

    void handleUart(particleSensorState_t& state) {
        if (!sensorSerial.available()) {
            return;
        }

        Serial.print("Receiving:");
        while (sensorSerial.available()) {
            serialRxBuf[rxBufIdx++] = sensorSerial.read();
            Serial.print(".");

            // Without this delay, receiving data breaks for reasons that are beyond me
            delay(15);

            if (rxBufIdx >= 64) {
                clearRxBuf();
            }
        }
        Serial.println("Done.");

        if (isValidHeader() && isValidChecksum()) {
            parseState(state);

            Serial.printf(
                "CurrentPM2.5  measurements: %d, %d, %d, %d, %d\n",

                state.pm25Measurements[0],
                state.pm25Measurements[1],
                state.pm25Measurements[2],
                state.pm25Measurements[3],
                state.pm25Measurements[4]
            );
        } else {
            clearRxBuf();
        }
    }
} // namespace SerialCom
