#include "knobs.hpp"

namespace knobs {

const u_int8_t MODE_EXTINT_NUMB = 0; // External interrupt number for the mode pushbutton. From SAMD21G18A datasheet, table 7-1
const u_int8_t ZERO_EXTINT_NUMB = 5; // External interrupt number for the zero pushbutton. From SAMD21G18A datasheet, table 7-1

// Global variables definition
uint32_t sensitivity = 0;
uint32_t threshold = 0;
bool tare_needed = false;
bool mode_button_pressed = false;

void setup() {
    // Disable interrupts
    __disable_irq();

    // Power management ///////////////////////////////////////////////////////////////////////
    PM->APBCMASK.reg |= PM_APBAMASK_EIC;      // Activate the External Interrupt Controller peripheral

    // Generic clock //////////////////////////////////////////////////////////////////////////
    // Set a 100Hz clock for the external interrupts debounce -> 30ms debounce time

    // Configure the divider
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(328) |          // Divide the clock by 328 (32768Hz / 328 = 100Hz)
                       GCLK_GENDIV_ID(3);               // Select GCLK3
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    
    // Configure the generator
    GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |         // Enable the clock generator
                        GCLK_GENCTRL_SRC_OSC32K |  // Select the 32.768kHz oscillator as source
                        GCLK_GENCTRL_ID(3);            // Select GCLK3
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronizations

    // Configure the clock
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 100Hz GCLK2 to the External Interrupt Controller clock
                        GCLK_CLKCTRL_GEN_GCLK3 |     
                        GCLK_CLKCTRL_ID_EIC;   
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization


    // Port pins /////////////////////////////////////////////////////////////////////
    // Connect the MODE Pin to the external interrupt 0
    PORT->Group[g_APinDescription[MODE_PIN].ulPort].PINCFG[g_APinDescription[MODE_PIN].ulPin].bit.PMUXEN = 1;                   // Enable PORT multiplexer
    PORT->Group[g_APinDescription[MODE_PIN].ulPort].PINCFG[g_APinDescription[MODE_PIN].ulPin].bit.PULLEN = 1;                   // Enable pull-x resistor
    PORT->Group[g_APinDescription[MODE_PIN].ulPort].OUT.reg |= 1 << (g_APinDescription[MODE_PIN].ulPin);                        // Set pull-up resistor
    PORT->Group[g_APinDescription[MODE_PIN].ulPort].PMUX[g_APinDescription[MODE_PIN].ulPin >> 1].reg |= PORT_PMUX_PMUXO_A;      // Select channel A = EXTINT[0]

    // Connect the ZERO Pin to the external interrupt 5
    PORT->Group[g_APinDescription[ZERO_PIN].ulPort].PINCFG[g_APinDescription[ZERO_PIN].ulPin].bit.PMUXEN = 1;                   // Enable PORT multiplexer
    PORT->Group[g_APinDescription[ZERO_PIN].ulPort].PINCFG[g_APinDescription[ZERO_PIN].ulPin].bit.PULLEN = 1;                   // Enable pull-x resistor
    PORT->Group[g_APinDescription[ZERO_PIN].ulPort].OUT.reg |= 1 << (g_APinDescription[ZERO_PIN].ulPin);                        // Set pull-up resistor
    PORT->Group[g_APinDescription[ZERO_PIN].ulPort].PMUX[g_APinDescription[ZERO_PIN].ulPin >> 1].reg |= PORT_PMUX_PMUXE_A;      // Select channel A = EXTINT[5]

    // Set SENSITIVITY potentiometers as analog input
    PORT->Group[g_APinDescription[SENSITIVITY_PIN].ulPort].PINCFG[g_APinDescription[SENSITIVITY_PIN].ulPin].bit.PMUXEN = 0;     // Disable PORT multiplexer
    PORT->Group[g_APinDescription[SENSITIVITY_PIN].ulPort].PINCFG[g_APinDescription[SENSITIVITY_PIN].ulPin].bit.PULLEN = 0;     // Disable pull-x resistor
    PORT->Group[g_APinDescription[SENSITIVITY_PIN].ulPort].DIRCLR.reg |= 1 << g_APinDescription[SENSITIVITY_PIN].ulPin;         // Set as input

    // Set THRESHOLD potentiometers as analog input
    PORT->Group[g_APinDescription[THRESHOLD_PIN].ulPort].PINCFG[g_APinDescription[THRESHOLD_PIN].ulPin].bit.PMUXEN = 0;         // Disable PORT multiplexer
    PORT->Group[g_APinDescription[THRESHOLD_PIN].ulPort].PINCFG[g_APinDescription[THRESHOLD_PIN].ulPin].bit.PULLEN = 0;         // Disable pull-x resistor
    PORT->Group[g_APinDescription[THRESHOLD_PIN].ulPort].DIRCLR.reg |= 1 << g_APinDescription[THRESHOLD_PIN].ulPin;             // Set as input
    
    // External interrupts ///////////////////////////////////////////////////////
    EIC->CTRL.bit.ENABLE = 0;                       // Disable the EIC peripheral
    while (EIC->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Setup the external interrupt for the MODE pushbutton
    EIC->CONFIG[g_APinDescription[MODE_PIN].ulPort].reg |= EIC_CONFIG_SENSE0_FALL |     // Detect falling edges
                                                           EIC_CONFIG_FILTEN0;          // Enable filter (debounger)
    while (EIC->STATUS.bit.SYNCBUSY);              // Wait for synchronization
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT0;   // Enable the interrupt
    while (EIC->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Setup the external interrupt for the ZERO pushbutton
    EIC->CONFIG[g_APinDescription[MODE_PIN].ulPort].reg |= EIC_CONFIG_SENSE5_FALL;// |     // Detect falling edges
                                                        //    EIC_CONFIG_FILTEN5;          // Enable filter (debounger)
    while (EIC->STATUS.bit.SYNCBUSY);              // Wait for synchronization
    EIC->INTENSET.reg |= EIC_INTENSET_EXTINT5;   // Enable the interrupt
    while (EIC->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Enable the EIC peripheral
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Enable the interrupt in the NVIC
    NVIC_SetPriority(EIC_IRQn, 1);       // Set EIC interrupt priority to second highest
    NVIC_EnableIRQ(EIC_IRQn);            // Enable Interrupts for EIC at NVIC

    // Enable interrupts globally
    __enable_irq();

}

int get_sensitivity() {
    /**
     * @brief Get the value of the sensitivity potentiometer.
     * 
     * @return int The value of the sensitivity potentiometer. Range: 0-1023
    */
    return analogRead(SENSITIVITY_PIN);
}

int get_threshold() {
    /**
     * @brief Get the value of the threshold potentiometer.
     * 
     * @return int The value of the threshold potentiometer. Range: 0-1023
    */
    return analogRead(THRESHOLD_PIN);
}

void mode_handler() {
    // Mode button was pressed
    mode_button_pressed = true;
}

void zero_handler() {
    // Zero button was pressed
    tare_needed = true;
}

} // namespace knobs

// Interrupt handler for the pushbuttons. Must be outside of any namespace
void EIC_Handler(void) {
    // Check which button was pressed
    if(EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT0) {
        // Mode button was pressed
        knobs::mode_handler();

        // Reset interrupt flag
        EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT0;
    }
    if(EIC->INTFLAG.reg & EIC_INTFLAG_EXTINT5) {
        // Zero button was pressed
        knobs::zero_handler();

        // Reset interrupt flag
        EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT5;
    }
}
