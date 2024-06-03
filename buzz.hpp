// Ensure this header file is included only once
#ifndef BUZZ_H
#define BUZZ_H

// Include Arduino core library. This is often not needed in .h files,
// but required for defining types like 'int' below.
#include <Arduino.h>
#include "pinout.hpp"

namespace buzzer {
    // Function prototypes
    void setup();
    void playMelodyMario();
    void playMelodyMetallica();
    void playMetal(uint32_t time_shifting, uint32_t max_time_shifting, uint32_t lower_threshold, uint32_t beep_duration);
} // namespace buzz

#endif // BUZZ_H