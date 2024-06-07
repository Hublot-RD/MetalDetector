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
    SerialUSB.print("Mode pin: "); SerialUSB.println(digitalRead(knobs::MODE_PIN));
    SerialUSB.print("Zero pin: "); SerialUSB.println(digitalRead(knobs::ZERO_PIN));
    SerialUSB.print("Sensitivity pin: "); SerialUSB.println(analogRead(knobs::SENSITIVITY_PIN));
    SerialUSB.print("Threshold pin: "); SerialUSB.println(analogRead(knobs::THRESHOLD_PIN));
    delay(100);
}
