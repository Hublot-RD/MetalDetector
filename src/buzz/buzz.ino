/*
Minimal version to test the buzzer library.

This code is a minimal version to test the buzzer library. It plays the Mario melody and the Metallica melody.
*/

#include <Arduino.h>
#include "buzz.hpp"

// Constants
constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;


void setup() {
    // Open serial communications on the native USB port
    if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);while(!SerialUSB);}

    // Setup the library
    buzzer::setup();
    if(DEBUG) {SerialUSB.println("Setup complete");}
}


void loop() {
    SerialUSB.println("Playing Mario melody");
    buzzer::playMelodyMario();
    delay(1000);

    SerialUSB.println("Playing some beep beep beep");
    for (uint8_t i = 0; i < 10; i++) {
        buzzer::playMetal(i+1, 10, 0, 95);
        delay(100);
    }
    for (uint8_t i = 0; i < 10; i++) {
        buzzer::playMetal(10-i, 10, 0, 95);
        delay(100);
    }
    delay(2000);
}
