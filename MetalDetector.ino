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
#include "buzz.hpp"
#include "battery.hpp"
#include "pulse.hpp"
#include "pinout.hpp"
#include "parameters.hpp"

// Global variables
uint32_t tare = 0;


void setup() {
  // Open serial communications on the native USB port
  if(DEBUG) {SerialUSB.begin(SERIAL_BAUD_RATE);}

  delay(STARTUP_TIME_MS);
  
  // Setup each library
  battery_setup();
  buzzer_setup();
  pulse_setup();

  // Wait for sensor to be active an zero it
  while(captured_value <= 0){SerialUSB.println("Waiting for captured value ...");delay(10);};
  tare = captured_value;

  // Play startup melody
  // playMelodyMetallica(BUZZER_PIN);
  playMelodyMario();
}


void loop() {
  SerialUSB.print("Tare value [us]:  ");
  SerialUSB.println(tare/float(MAIN_CLK_FREQ_MHZ));
  SerialUSB.print("Captured value [us]:  ");
  SerialUSB.println(captured_value/float(MAIN_CLK_FREQ_MHZ));

  // Compute time shifting
  uint32_t time_shifting = 0;
  if(captured_value > tare) {time_shifting = captured_value - tare;}
  
  playMetal(time_shifting, 25*MAIN_CLK_FREQ_MHZ, 10, (1000/LOOP_FREQ_HZ)*0.4);

  delay(1000/LOOP_FREQ_HZ);
}
