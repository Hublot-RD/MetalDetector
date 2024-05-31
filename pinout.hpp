#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// Arduino numbering is used. Not any value can be used, so better to keep these ones or check chap. 7 of datasheet.
#define PULSE_PIN       13          // Pin to control MOSFETs in Arduino numbering
#define SIGNAL_PIN      5           // Pin to measure the signal (positive input of the Analog Comparator)
#define REF_PIN         14          // Pin to check the reference voltage (negative input of the Analog Comparator). Used only if DEBUG == true.
#define AC_OUT_PIN      12          // Pin to check the Analog Comparator output. Used only if DEBUG == true.
#define BUZZER_PIN      11          // Pin for the buzzer sound
#define VBAT_PIN        A1
#define BATSELA_PIN     6
#define BATSELB_PIN     7
#define BATSELC_PIN     8

#endif // PINOUT_H