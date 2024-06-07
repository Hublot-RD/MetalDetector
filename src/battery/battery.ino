/*
Minimal version to test the battery library.

This code is a minimal version to test the battery library. It reads the battery voltage and prints it to the serial monitor.
Each cell is read separately. The voltage is given in volts.
*/

#include <Arduino.h>
#include "battery.hpp"

// Constants
constexpr bool DEBUG = true;
constexpr uint32_t SERIAL_BAUD_RATE = 115200;


void setup() {
    // Open serial communications on the native USB port
    if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);while(!SerialUSB);}

    // Setup the library
    battery::setup();
    if(DEBUG) {SerialUSB.println("Setup complete");}
}


void loop() {
    for (uint8_t channel = 0; channel < 6; channel++) {
        float voltage = battery::read(channel);
        if(DEBUG) {SerialUSB.print("Cell "); SerialUSB.print(channel); SerialUSB.print(": "); SerialUSB.print(voltage); SerialUSB.print(" V\n");}
    }
    SerialUSB.println("--------------------");
    
    delay(1000);
}
