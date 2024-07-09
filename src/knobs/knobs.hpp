#ifndef KNOBS_H
#define KNOBS_H

#include <Arduino.h>

namespace knobs {
    // Constants
    constexpr uint32_t MAX_SENSITIVITY = 48*3;            // Maximum value of the sensitivity potentiometer
    constexpr uint32_t MIN_SENSITIVITY = 0;               // Minimum value of the sensitivity potentiometer

    // Pinout
    constexpr uint8_t SENSITIVITY_PIN =     21;          // Pin for the sensitivity potentiometer
    constexpr uint8_t THRESHOLD_PIN =       19;          // Pin for the threshold potentiometer
    constexpr uint8_t MODE_PIN =            11;          // Pin for the mode button
    constexpr uint8_t ZERO_PIN =            10;          // Pin for the zero button

    // Global variables declaration
    extern uint32_t sensitivity;
    extern uint32_t threshold;
    extern bool tare_needed;
    extern bool mode_button_pressed;

    // Function prototypes
    void setup(void);
    int get_sensitivity();
    int get_threshold();
    
} // namespace knobs

#endif // KNOBS_H