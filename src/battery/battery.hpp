#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

namespace battery {
    // Pinout
    constexpr uint8_t VBAT_PIN      = 15;          // Pin to measure the battery voltage
    constexpr uint8_t BATSELA_PIN   = 6;           // Pins to select the cell to measure
    constexpr uint8_t BATSELB_PIN   = 7;
    constexpr uint8_t BATSELC_PIN   = 8;

    // Function prototypes
    void setup();
    float read(uint8_t channel);
}

#endif // BATTERY_H