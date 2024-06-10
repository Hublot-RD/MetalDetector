#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>
#include <FastLED.h>

namespace leds {
    // Constants
    constexpr uint8_t NUM_LEDS = 8;         // Number of LEDs
    constexpr uint8_t BRIGHTNESS = 100;     // LEDs brightness, 0-255

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