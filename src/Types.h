#pragma once

struct particleSensorState_t {
    uint16_t avgPM1 = 0;
    uint16_t avgPM25 = 0;
    uint16_t avgPM10 = 0;

    uint16_t pm1Measurements[5] = {0, 0, 0, 0, 0};
    uint16_t pm25Measurements[5] = {0, 0, 0, 0, 0};
    uint16_t pm10Measurements[5] = {0, 0, 0, 0, 0};

    uint8_t measurementIdx = 0;
    boolean valid = false;
};
