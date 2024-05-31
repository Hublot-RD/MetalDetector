/*
Pulse Induction Metal detector.

This script is made for the Arduino Nano 33 IoT, based on the SAMD21G18A microcontroller.

User interface
Inputs :
- None

Outputs:
- Buzzer sound with pitch proportionnal to time shifting


Script inspired from https://forum.arduino.cc/t/samd21-mkrzero-analog-comparator/1194763
*/

#include <Arduino.h>
#include "buzz.hpp"
#include "battery.hpp"
#include "pulse.hpp"
#include "pinout.hpp"


#define DEBUG true

// Timing
#define MAIN_CLK_FREQ_MHZ 48      // Main clock frequency [MHz]. This cannot be modified.
#define PULSE_WIDTH_US    200     // Length of a pulse [us]
#define PULSE_FREQ_HZ     30      // Pulse frequency [Hz]
#define STARTUP_TIME_MS   500     // Time to let the supply capacitor charge before starting to do pulses [ms]
#define LOOP_FREQ_HZ      10       // Main loop frequency [Hz]

// Other
#define SERIAL_BAUD_RATE  115200  // Baud rate for the USB communication


uint32_t captured_value = 0;
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
  while(captured_value <= 0){SerialUSB.println("a");};
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
