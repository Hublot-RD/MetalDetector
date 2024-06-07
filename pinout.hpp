#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

/*
Arduino numbering is used. 
Not any value can be used, so better to keep these ones or check chap. 7 of SAMD21G18A datasheet.
*/

// Debugging
#define REF_PIN         14          // Pin to check the reference voltage (negative input of the Analog Comparator). Used only if DEBUG == true.
#define AC_OUT_PIN      12          // Pin to check the Analog Comparator output. Used only if DEBUG == true.

// User interface        
#define LEDS_PIN        20          // Pins for the 8 LEDs


// Pulse
#define PULSE_PIN       13          // Pin to control MOSFETs in Arduino numbering
#define SIGNAL_PIN      5           // Pin to measure the signal (positive input of the Analog Comparator)
#define COILSELA_PIN    2           // Pins to select the coil to pulse
#define COILSELB_PIN    3
#define COILSELC_PIN    4

#endif // PINOUT_H