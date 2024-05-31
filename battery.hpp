#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "pinout.hpp"

// Function prototypes
void battery_setup();
float battery_read(uint8_t channel);

#endif // BATTERY_H