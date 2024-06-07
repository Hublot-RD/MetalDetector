#ifndef PULSE_H
#define PULSE_H

#include <Arduino.h>
#include "..\..\pinout.hpp"
#include "..\..\parameters.hpp"

namespace pulse {
    // Function prototypes
    void setup();
    void select(uint8_t channel);
}

#endif // PULSE_H