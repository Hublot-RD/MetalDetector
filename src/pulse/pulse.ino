/*
Minimal version to test the pulse library.

This code is a minimal version to test the pulse library.
*/

#include <Arduino.h>
#include "pulse.hpp"

// Constants
constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;


void setup() {
    // Open serial communications on the native USB port
    if(DEBUG) {
        SerialUSB.begin(SERIAL_BAUD_RATE);
        for(uint8_t i = 0; i < 10; i++) {
            if(SerialUSB) {break;}
            delay(100);
        }
    }

    // Setup the library
    pulse::setup();
    if(DEBUG) {SerialUSB.println("Setup complete");}
}


void loop() {
    // Switch betwoon coils
    for(uint8_t i = 0; i < pulse::NB_COILS; i++) {
        pulse::select(i);
        delay(1000);
    }
}