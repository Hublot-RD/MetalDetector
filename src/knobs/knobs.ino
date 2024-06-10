/*
Minimal version to test the buzzer library.

This code is a minimal version to test the buzzer library. It plays the Mario melody and the Metallica melody.
*/

#include <Arduino.h>
#include "knobs.hpp"

// Constants
constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;


void setup() {
    // Open serial communications on the native USB port
    if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);while(!SerialUSB);}

    // Setup the library
    knobs::setup();
    if(DEBUG) {SerialUSB.println("Setup complete");}
}


void loop() {
    SerialUSB.print("Sensitivity: "); SerialUSB.println(knobs::get_sensitivity());
    SerialUSB.print("Threshold: "); SerialUSB.println(knobs::get_threshold());
    delay(1000);
}
