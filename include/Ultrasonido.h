#pragma once

#include "distance_sensor.h"

class Ultrasonido {
public:
    Ultrasonido(uint8_t pin);
    void init();
    uint32_t getDistance();

private:
    uint8_t xGP = 0;
    DistanceSensor* sensor;
};
