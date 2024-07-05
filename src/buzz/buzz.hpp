// Ensure this header file is included only once
#ifndef BUZZ_H
#define BUZZ_H

// Include Arduino core library. This is often not needed in .h files,
// but required for defining types like 'int' below.
#include <Arduino.h>
#include "pitches.hpp"

namespace buzzer {
    // Constants
    constexpr uint16_t NOTE_METAL_LOWEST = NOTE_E6;     // Lowest note for the metal detection sound
    constexpr uint16_t NOTE_METAL_HIGHEST = NOTE_DS7;   // Highest note for the metal detection sound
    constexpr uint8_t BPM = 168;                        // Beats per minute, used for the melody
    
    // pinout
    constexpr uint8_t BUZZER_PIN = 9;   // Pin for the buzzer sound

    // Function prototypes
    void setup();
    void playMelodyMario();
    void playMelodyMetallica();
    void playMetal(uint32_t time_shifting, uint32_t max_time_shifting, uint32_t lower_threshold, uint32_t beep_duration);
} // namespace buzz

#endif // BUZZ_H