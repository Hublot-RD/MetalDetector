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
    #define RED   CHSV(HSVHue::HUE_RED, 255, BRIGHTNESS)
    #define GREEN CHSV(HSVHue::HUE_GREEN, 255, BRIGHTNESS)
    #define BLUE  CHSV(HSVHue::HUE_BLUE, 255, BRIGHTNESS)
    #define YELLOW CHSV(HSVHue::HUE_YELLOW, 255, BRIGHTNESS)
    #define WHITE CHSV(0, 0, BRIGHTNESS)
    #define BLACK CHSV(0, 0, 0)

    // Function prototypes
    void setup(void);
    void set_from_pulse(uint32_t time_shifting[NUM_LEDS], uint32_t threshold, bool active_coils[NUM_LEDS]);
    void set_tare();
    void set(uint8_t channel, CHSV color);
    void animate(void);
    
} // namespace knobs

#endif // LEDS_H