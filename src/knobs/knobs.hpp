#ifndef KNOBS_H
#define KNOBS_H

#include <Arduino.h>

namespace knobs {
    // Pinout
    constexpr uint8_t SENSITIVITY_PIN =     21;          // Pin for the sensitivity potentiometer
    constexpr uint8_t THRESHOLD_PIN =       19;          // Pin for the threshold potentiometer
    constexpr uint8_t MODE_PIN =            11;          // Pin for the mode button
    constexpr uint8_t ZERO_PIN =            10;          // Pin for the zero button

    // Function prototypes
    void setup(void);
    int get_sensitivity();
    int get_threshold();
    
} // namespace knobs

#endif // KNOBS_H