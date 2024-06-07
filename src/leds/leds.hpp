#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>
#include <FastLED.h>

namespace leds {
    // Pinout
    constexpr uint8_t LEDS_PIN = 20;          // Pins for the 8 LEDs

    // Colors
    constexpr CRGB RED = CRGB::Red;
    constexpr CRGB GREEN = CRGB::Green;
    constexpr CRGB BLUE = CRGB::Blue;
    constexpr CRGB WHITE = CRGB::White;
    constexpr CRGB BLACK = CRGB::Black;

    // Function prototypes
    void setup(void);
    void set(uint8_t channel, CRGB color);
    void animate(void);
    
} // namespace knobs

#endif // LEDS_H