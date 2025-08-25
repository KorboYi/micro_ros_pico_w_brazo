#include "Ultrasonido.h"
#include "pico/stdlib.h"

Ultrasonido::Ultrasonido(uint8_t pin) : xGP(pin) {
}

void Ultrasonido::init() {
    sensor = new DistanceSensor(pio0, 0, xGP);
}

uint32_t Ultrasonido::getDistance() {
    sensor->TriggerRead();
    while (sensor->is_sensing) {
        sleep_us(100);
    }
    return sensor->distance;
}
