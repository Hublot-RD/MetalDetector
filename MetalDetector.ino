/*
Pulse Induction Metal detector.

This script is made for the Arduino Nano 33 IoT, based on the SAMD21G18A microcontroller.

User interface
Inputs :
- Mode button
- Zero button
- Sensitivity potentiometer
- Threshold potentiometer
- (Volume potentiometer)


Outputs:
- Buzzer sound with pitch proportionnal to time shifting
- 8 LEDs as a display. Can either show which coil is detecting some metal, battery level, mode selection
- Serial communication with the computer
*/

#include <Arduino.h>
#include "parameters.hpp"
#include "src\battery\battery.hpp"
#include "src\buzz\buzz.hpp"
#include "src\knobs\knobs.hpp"
#include "src\pulse\pulse.hpp"
#include "src\leds\leds.hpp"

// Global variables
bool desired_channels[pulse::NB_COILS] = {true, false, false, false, false, false, false, false};
uint32_t time_shifting_threshold = 10;


void setup() {
    // Open serial communications on the native USB port
    if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);}

    delay(STARTUP_TIME_MS);
    
    // Setup each library
    battery::setup();
    buzzer::setup();
    knobs::setup();
    leds::setup();
    pulse::setup();
    pulse::set_active_coils(desired_channels);
    // pulse::set_threshold(100);
    pulse::tare();

    // Play startup melody
    buzzer::playMelodyMario();
}


void loop() {
    static uint8_t display_should_stay_x_cycles = 0; // Number of cycles the display should stay on the current state. Used when changing the mode or taring
    pulse::measure meas = pulse::get_captured_value();
    // Print the captured values
    if(DEBUG) {
        SerialUSB.println("------------------------------------------------------------");
        SerialUSB.println("\t\tTimeShifting  \tCaptured  \tTare");
        for(uint8_t i = 0; i < pulse::NB_COILS; i++) {
            SerialUSB.print("Channel "); SerialUSB.print(i); SerialUSB.print(":\t");
            SerialUSB.print(meas.time_shifting[i]); SerialUSB.print("\t\t");
            SerialUSB.print(meas.captured_value[i]); SerialUSB.print("\t\t");
            SerialUSB.println(meas.tare[i]);
        }
    }

    uint32_t highest_time_shifting = meas.time_shifting[0];
    for(uint8_t i = 0; i < pulse::NB_COILS; i++) {
        if(meas.time_shifting[i] > highest_time_shifting) {
            highest_time_shifting = meas.time_shifting[i];
        }
    }

    // Play sound
    buzzer::playMetal(highest_time_shifting, 25*pulse::MAIN_CLK_FREQ_MHZ, time_shifting_threshold, (1000/LOOP_FREQ_HZ)*0.4);

    // Update the display
    if(display_should_stay_x_cycles > 0) {
        display_should_stay_x_cycles--;
    } else {
        leds::set_from_pulse(meas.time_shifting, time_shifting_threshold, desired_channels);
    }

    // Update according to knobs
    pulse::set_threshold(knobs::get_threshold());
    time_shifting_threshold = knobs::get_sensitivity();
    if(DEBUG) {
        SerialUSB.print("Sensitivity: "); SerialUSB.println(time_shifting_threshold);
        SerialUSB.print("Threshold: "); SerialUSB.println(knobs::get_threshold());
    }

    // Tare if necessary
    if(knobs::tare_needed) {
        display_should_stay_x_cycles = 1*LOOP_FREQ_HZ;
        leds::set_tare();
        pulse::tare();
        knobs::tare_needed = false;
        if(DEBUG) {SerialUSB.println("Tare done");}
    }

    delay(1000/LOOP_FREQ_HZ);
}
