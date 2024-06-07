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
#include "pinout.hpp"
#include "parameters.hpp"
#include "src\battery\battery.hpp"
#include "src\buzz\buzz.hpp"
#include "src\knobs\knobs.hpp"
#include "src\pulse\pulse.hpp"

// Global variables
uint32_t tare = 0;


void setup() {
  // Open serial communications on the native USB port
  if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);}

  delay(STARTUP_TIME_MS);
  
  // Setup each library
  battery::setup();
  buzzer::setup();
  pulse::setup();
  knobs::setup();

  // Wait for sensor to be active an zero it
  while(captured_value <= 0){
    static uint8_t i = 1;
    if(DEBUG) {SerialUSB.println("Waiting for captured value ...");}
    delay(100);
    if(i > 10){
      if(DEBUG) {SerialUSB.println("Could not capture a value. Continuing without tare = 0.");}
      break;
    }
    i++;
  };
  tare = captured_value;

  // Play startup melody
  // playMelodyMetallica(BUZZER_PIN);
  buzzer::playMelodyMario();
}


void loop() {
  static uint8_t channel = 0;
  static uint8_t cycle_cnt = 0;
  if (cycle_cnt >= 5*LOOP_FREQ_HZ) {
    channel = (channel+1) % 2;
    pulse::select(channel);
    cycle_cnt = 0;
  }

  if (DEBUG) {
    SerialUSB.print("Channel: "); SerialUSB.println(channel);
    SerialUSB.print("Tare value [us]:  "); SerialUSB.println(tare/float(MAIN_CLK_FREQ_MHZ));
    SerialUSB.print("Captured value [us]:  "); SerialUSB.println(captured_value/float(MAIN_CLK_FREQ_MHZ));
  }

  // Compute time shifting
  uint32_t time_shifting = 0;
  if(captured_value > tare) {time_shifting = captured_value - tare;}

  // Play sound
  buzzer::playMetal(time_shifting, 25*MAIN_CLK_FREQ_MHZ, 10, (1000/LOOP_FREQ_HZ)*0.4);

  cycle_cnt++;
  delay(1000/LOOP_FREQ_HZ);
}
