#ifndef PULSE_H
#define PULSE_H

#include <Arduino.h>

namespace pulse {
    // Debug options
    constexpr bool ALL_OUTPUTS = true;          // Enable the output of the AC and DAC signals on physical pins
    
    // Constants
    constexpr uint8_t NB_COILS = 3;             // Number of coils
    constexpr uint16_t PULSE_WIDTH_US = 200;    // Length of a pulse [us]
    constexpr uint16_t PULSE_FREQ_HZ = 30;      // Pulse frequency [Hz]
    constexpr uint8_t MAIN_CLK_FREQ_MHZ = 48;   // Main clock frequency [MHz]. This cannot be modified.

    /* Pinout
    Arduino numbering is used.
    Not any value can be used, so better to keep these ones or check chap. 7 of SAMD21G18A datasheet.
    */ 
    constexpr uint8_t PULSE_PIN = 13;           // Pin to control MOSFETs in Arduino numbering
    constexpr uint8_t SIGNAL_PIN = 5;           // Pin to measure the signal (positive input of the Analog Comparator)
    constexpr uint8_t COILSELA_PIN = 2;         // Pins to select the coil to pulse
    constexpr uint8_t COILSELB_PIN = 3;
    constexpr uint8_t COILSELC_PIN = 4;
    constexpr uint8_t REF_PIN = 14;             // Pin to check the reference voltage (negative input of the Analog Comparator). Used only if DEBUG == true.
    constexpr uint8_t AC_OUT_PIN = 12;          // Pin to check the Analog Comparator output. Used only if DEBUG == true.

    // Global variables declaration
    volatile extern uint32_t captured_value;    // Value captured by the Analog Comparator, it is the time between the pulse and the signal detection

    // Function prototypes
    void setup();
    void select(uint8_t channel);
}

#endif // PULSE_H