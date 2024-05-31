#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

#define VBAT_PIN A1
#define BATSELA_PIN 6
#define BATSELB_PIN 7
#define BATSELC_PIN 8

// Function prototypes
void battery_setup();
float battery_read(uint8_t channel);

#endif // BATTERY_H