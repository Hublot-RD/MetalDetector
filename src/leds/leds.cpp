#include "leds.hpp"

namespace leds {

constexpr uint8_t NUM_LEDS = 8;         // Number of LEDs
constexpr uint8_t BRIGHTNESS = 100;     // LEDs brightness, 0-255
CRGBArray<NUM_LEDS> led_stick;

void setup(void) {
    // Setup the LED stick
    FastLED.addLeds<NEOPIXEL,LEDS_PIN>(led_stick, NUM_LEDS);
}

void set(uint8_t channel, CRGB color) {
    led_stick[channel] = color;
    FastLED.show();
}

void animate(void) {
    for (uint8_t hue = 0; hue < 255; hue++) {
        for(uint8_t i = 0; i < NUM_LEDS; i++) {
            led_stick[i] = CHSV(hue+32*i, 255, BRIGHTNESS);
        }
        FastLED.show();
        delay(10);
    }
}

} // namespace leds