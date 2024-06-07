#include "knobs.hpp"

namespace knobs {

const u_int8_t MODE_EXTINT_NUMB = 0; // External interrupt number for the mode pushbutton. From SAMD21G18A datasheet, table 7-1
const u_int8_t ZERO_EXTINT_NUMB = 5; // External interrupt number for the zero pushbutton. From SAMD21G18A datasheet, table 7-1

void setup() {
    // Setup the potentiometers
    pinMode(SENSITIVITY_PIN, INPUT);
    pinMode(THRESHOLD_PIN, INPUT);

    // Setup the pushbuttons
    // Disable interrupts
    __disable_irq();

    // Power management ///////////////////////////////////////////////////////////////////////
    PM->APBCMASK.reg |= PM_APBAMASK_EIC;      // Activate the External Interrupt Controller peripheral

    // Generic clock ///////////////////////////////////////////////////////////////////////////
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 32.768kHz GCLK1 to the External Interrupt Controller clock
                        GCLK_CLKCTRL_GEN_GCLK1 |     
                        GCLK_CLKCTRL_ID_EIC;   
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    // TODO: Maybe add a divider to the clock to debounce more

    // Configure port pins /////////////////////////////////////////////////////////////////////
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

    // Set potentiometers as analog inputs
    PORT->Group[g_APinDescription[SENSITIVITY_PIN].ulPort].PINCFG[g_APinDescription[SENSITIVITY_PIN].ulPin].bit.PMUXEN = 0;     // Disable PORT multiplexer
    PORT->Group[g_APinDescription[SENSITIVITY_PIN].ulPort].PINCFG[g_APinDescription[SENSITIVITY_PIN].ulPin].bit.PULLEN = 0;     // Disable pull-x resistor
    PORT->Group[g_APinDescription[SENSITIVITY_PIN].ulPort].DIRCLR.reg = 1 << g_APinDescription[SENSITIVITY_PIN].ulPin;          // Set as input
    PORT->Group[g_APinDescription[THRESHOLD_PIN].ulPort].PINCFG[g_APinDescription[THRESHOLD_PIN].ulPin].bit.PMUXEN = 0;         // Disable PORT multiplexer
    PORT->Group[g_APinDescription[THRESHOLD_PIN].ulPort].PINCFG[g_APinDescription[THRESHOLD_PIN].ulPin].bit.PULLEN = 0;         // Disable pull-x resistor
    PORT->Group[g_APinDescription[THRESHOLD_PIN].ulPort].DIRCLR.reg = 1 << g_APinDescription[THRESHOLD_PIN].ulPin;              // Set as input
    
    // Configure the external interrupts ///////////////////////////////////////////////////////
    // Setup the external interrupt for the MODE pushbutton
    EIC->CONFIG[g_APinDescription[MODE_PIN].ulPort].reg |= EIC_CONFIG_SENSE0_FALL |     // Detect falling edges
                                                           EIC_CONFIG_FILTEN0;          // Enable filter (debounger)
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT0;   // Enable the interrupt

    // Setup the external interrupt for the ZERO pushbutton
    EIC->CONFIG[g_APinDescription[MODE_PIN].ulPort].reg |= EIC_CONFIG_SENSE0_FALL |     // Detect falling edges
                                                           EIC_CONFIG_FILTEN0;          // Enable filter (debounger)
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT0;   // Enable the interrupt

    // Enable interrupts
    __enable_irq();
}

void mode_handler() {
    // Mode button was pressed
    SerialUSB.println("Mode button pressed");
}

void zero_handler() {
    // Zero button was pressed
    SerialUSB.println("Zero button pressed");
}

} // namespace knobs

// Interrupt handler for the pushbuttons. Must be outside of any namespace
void EIC_Handler(void) {
    SerialUSB.println("Interrupt handler called");
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
