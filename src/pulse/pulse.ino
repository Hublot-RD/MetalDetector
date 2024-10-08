/*
Minimal version to test the pulse library.

This code is a minimal version to test the pulse library. It will print the captured values of the coils every second.
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

    bool desired_channels[pulse::NB_COILS] = {true, true, false, false, false, false, false, false};
    pulse::set_active_coils(desired_channels);
    // pulse::set_threshold(100);
    pulse::tare();
    if(DEBUG) {SerialUSB.println("Setup complete");}
}


void loop() {
    // Print the captured values
    if(DEBUG) {
        SerialUSB.println("------------------------------------------------------------");
        SerialUSB.println("\t\tTimeShifting  \tCaptured  \tTare");
        pulse::measure meas = pulse::get_captured_value();
        for(uint8_t i = 0; i < pulse::NB_COILS; i++) {
            SerialUSB.print("Channel "); SerialUSB.print(i); SerialUSB.print(":\t");
            SerialUSB.print(meas.time_shifting[i]); SerialUSB.print("\t\t");
            SerialUSB.print(meas.captured_value[i]); SerialUSB.print("\t\t");
            SerialUSB.println(meas.tare[i]);
        }
    }
    
    delay(1000);
}