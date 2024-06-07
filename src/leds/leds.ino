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
}


void loop() {
    leds::animate();
}
