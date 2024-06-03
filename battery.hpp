#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "pinout.hpp"

namespace battery {
    // Function prototypes
    void setup();
    float read(uint8_t channel);
}

#endif // BATTERY_H