// Ensure this header file is included only once
#ifndef BUZZ_H
#define BUZZ_H

// Include Arduino core library. This is often not needed in .h files,
// but required for defining types like 'int' below.
#include <Arduino.h>
#include "pinout.hpp"

// Define a structure to hold a note and its duration
struct Note {
  int pitch;
  int duration;
};

// Function prototypes
void buzzer_setup();
void playMelodyMario();
void playMelodyMetallica();
void playMetal(uint32_t time_shifting, uint32_t max_time_shifting, uint32_t lower_threshold, uint32_t beep_duration);

#endif // BUZZ_H