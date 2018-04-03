#pragma once

#include "pins.h"
#include <pigpiod_if2.h>

class IRSensor {
    public:
        bool flameVisible() const {
            return gpio_read(0, irSensorPin);
        }
    private:
        IRSensor(); //hidden constructor, not needed
};
