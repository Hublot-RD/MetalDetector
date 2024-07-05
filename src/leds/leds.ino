/*
Minimal version to test the LEDs library.

This code is a minimal version to test the LEDs library. It sets the 8 LEDs to different colors.
*/

#include <Arduino.h>
#include "leds.hpp"

// Constants
constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;


void setup() {
    // Open serial communications on the native USB port
    if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);while(!SerialUSB);}

    // Setup the library
    leds::setup();
    if(DEBUG) {SerialUSB.println("Setup complete");}

    // Test the function
    uint32_t pulses[leds::NUM_LEDS];
    bool active_coils[leds::NUM_LEDS] = {true, true, true, false, true, true, false, true};
    for (uint8_t i = 0; i < leds::NUM_LEDS; i++) {
        pulses[i] = i*10;
    }

    leds::set_from_pulse(pulses, 15, active_coils);
    delay(5000);
}


void loop() {
    leds::animate();
}
