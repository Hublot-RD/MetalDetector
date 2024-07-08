#include "leds.hpp"

namespace leds {

CRGBArray<NUM_LEDS> led_stick;

void setup(void) {
    /**
     * @brief Setup the LED stick
    */
    // Setup the LED stick
    FastLED.addLeds<NEOPIXEL,LEDS_PIN>(led_stick, NUM_LEDS);
}

void set_from_pulse(uint32_t time_shifting[NUM_LEDS], uint32_t threshold, bool active_coils[NUM_LEDS]) {
    /**
     * @brief Set the LEDs color based on the time shifting values
     * 
     * @param time_shifting: Array of time shifting values
     * @param threshold: Time shifting threshold value to determine the color (in the same unit as the time shifting values)
     * @param active_coils: Array of active coils. Inactive coils are set to black
    */
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        uint8_t coil = NUM_LEDS - i - 1;
        if(!active_coils[coil]) {
            led_stick[i] = BLACK;
            continue;
        }
        if(time_shifting[coil] > threshold) {
            led_stick[i] = GREEN;
        } else {
            led_stick[i] = RED;
        }
    }
    FastLED.show();
}

void set_tare() {
    /**
     * @brief Set the LEDs color to white
    */
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        led_stick[i] = YELLOW;
    }
    FastLED.show();
}

void set_mode(uint8_t mode){
    /**
     * @brief Set the LEDs color based on the mode
     * 
     * @param mode: Mode to set
    */
    for(uint8_t i = 0; i < NUM_LEDS; i++) {
        if(NUM_LEDS - i - 1 == mode) {led_stick[i] = BLUE;} 
        else {led_stick[i] = BLACK;}
    }
    FastLED.show();
}

void set(uint8_t channel, CHSV color) {
    /**
     * @brief Set the color of a specific LED
     * 
     * @param channel: Channel number
     * @param color: Color to set
    */
    led_stick[channel] = color;
    FastLED.show();
}

void animate(void) {
    /**
     * @brief Animate the LEDs with a rainbow effect
    */
    for (uint8_t hue = 0; hue < 255; hue++) {
        for(uint8_t i = 0; i < NUM_LEDS; i++) {
            led_stick[i] = CHSV(hue+32*i, 255, BRIGHTNESS);
        }
        FastLED.show();
        delay(10);
    }
}

} // namespace leds