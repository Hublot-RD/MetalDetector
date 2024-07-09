/*
Script inspired from https://forum.arduino.cc/t/samd21-mkrzero-analog-comparator/1194763
*/

#include "pulse.hpp"

namespace pulse {

// Global variables definition
bool active_coils[NB_COILS];
measure meas;
volatile uint32_t captured_value = 0;
uint8_t pulsing_coil = 0;

// Private functions prototypes
void setup_AC();
void setup_DAC();
void setup_EVSYS();
void setup_TCC0();
void setup_TC3();
void setup_TC4();
void select(uint8_t channel);
uint8_t select_next_coil(uint8_t last_coil);
void tare_coil(uint8_t coil);
uint16_t map_threshold(uint16_t threshold);


/////////////////////////////////////
/////////////////////////////////////
////                             ////
//// Public functions definition ////
////                             ////
/////////////////////////////////////
/////////////////////////////////////

void setup() {
    /**
     * @brief Setup the pulse library.
     * 
     * This function starts the pulsing and measuring of the coils.
    */
    // Initialize global variables
    active_coils[0] = true;
    for(uint8_t i = 1; i < NB_COILS; i++) {active_coils[i] = false;}

    // Disable interrupts
    __disable_irq();

    // Configure port pins
    pinMode(COILSELA_PIN, OUTPUT);
    pinMode(COILSELB_PIN, OUTPUT);
    pinMode(COILSELC_PIN, OUTPUT);
    digitalWrite(COILSELA_PIN, LOW);
    digitalWrite(COILSELB_PIN, LOW);
    digitalWrite(COILSELC_PIN, LOW);

    // Setup peripherals
    setup_EVSYS();
    setup_TCC0();
    setup_TC3();
    setup_TC4();
    setup_AC();
    setup_DAC();

    // Enable interrupts
    __enable_irq();
}

void set_active_coils(bool desired_coils[NB_COILS]) {
    /**
     * @brief Set the active coils.
     * 
     * @param desired_coils[NB_COILS] Array of booleans to set the active coils. The index of the array corresponds to the coil number.
    */
    for(uint8_t i = 0; i < NB_COILS; i++) {
        active_coils[i] = desired_coils[i];
    }
}

struct measure get_captured_value() {
    /**
     * @brief Get the captured value.
     * 
     * @return struct measure The captured value.
    */
    for(uint8_t i = 0; i < NB_COILS; i++) {
        if(meas.captured_value[i] > meas.tare[i]) {
            meas.time_shifting[i] = meas.captured_value[i] - meas.tare[i];
        } else {
            meas.time_shifting[i] = 0;
        }
    }
    return meas;
}

uint16_t set_threshold(uint32_t threshold_mv) {
    /**
     * @brief Set the threshold.
     * 
     * @param threshold The threshold to set, in mV. 10 bit resolution on 1000mV.
    */
    uint16_t threshold = map_threshold(threshold_mv);

    // Set the threshold
    DAC->DATA.reg = DAC_DATA_DATA(threshold);
    // Wait for synchronization
    while(DAC->STATUS.bit.SYNCBUSY);

    return threshold;
}

void tare() {
    /**
     * @brief Tare all active the coils.
    */
    // Disable automatic coil selection
    TC3->COUNT16.EVCTRL.bit.TCEI = 0;   // Disable input event so the timer will not be triggered

    for(uint8_t i = 0; i < NB_COILS; i++) {
        if(active_coils[i]) {tare_coil(i);}
    }

    // Enable automatic coil selection
    TC3->COUNT16.EVCTRL.bit.TCEI = 1;   // Enable input event so the timer will be triggered
}

//////////////////////////////////////
//////////////////////////////////////
////                              ////
//// Private functions definition ////
////                              ////
//////////////////////////////////////
//////////////////////////////////////

void setup_AC() {
    /**
     * @brief Setup the Analog Comparator.
    */
    // Power Managment: Activate the analog comparator peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_AC;

    // Generic clock: Route the generic clocks
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 32.768kHz GCLK1 to the Analog Comparator analog clock
                        GCLK_CLKCTRL_GEN_GCLK1 |     
                        GCLK_CLKCTRL_ID_AC_ANA;   
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 48MHz GCLK0 to the Analog Comparator digital clock
                        GCLK_CLKCTRL_GEN_GCLK0 |     // This is the main clock of the AC (used for filtering for example)
                        GCLK_CLKCTRL_ID_AC_DIG;   
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

    // PORT: Configure port and pins
    PORT->Group[g_APinDescription[SIGNAL_PIN].ulPort].PINCFG[g_APinDescription[SIGNAL_PIN].ulPin].bit.PMUXEN = 1;                   // Enable PORT multiplexer
    PORT->Group[g_APinDescription[SIGNAL_PIN].ulPort].PMUX[g_APinDescription[SIGNAL_PIN].ulPin >> 1].reg |= PORT_PMUX_PMUXO_B;      // Select channel B = AC channel 1 input = AIN[1]
    if(ALL_OUTPUTS) {
        PORT->Group[g_APinDescription[AC_OUT_PIN].ulPort].PINCFG[g_APinDescription[AC_OUT_PIN].ulPin].bit.PMUXEN = 1;                 // Enable PORT multiplexer
        PORT->Group[g_APinDescription[AC_OUT_PIN].ulPort].PMUX[g_APinDescription[AC_OUT_PIN].ulPin >> 1].reg |= PORT_PMUX_PMUXO_H;    // Select channel H = AC channel 1 output = AC/CMP[1]
    }

    // Analog Comparator: Configure the analog comparator
    AC->COMPCTRL[1].reg = AC_COMPCTRL_OUT_ASYNC |       // Enable comparator output in asynchronous mode
                        AC_COMPCTRL_MUXPOS_PIN1 |     // Set the positive input multiplexer to pin 1
                        AC_COMPCTRL_MUXNEG_DAC |      // Set the negative input multiplexer to the voltage scaler
                        AC_COMPCTRL_INTSEL_FALLING |  // Generate interrupts only on falling edge of AC output
                        AC_COMPCTRL_SPEED_HIGH;       // Place the comparator into high speed mode
    while (AC->STATUSB.bit.SYNCBUSY);                   // Wait for synchronization

    AC->INTENSET.bit.COMP1 |= 1;        // Enable interrupt for AC1
    while (AC->STATUSB.bit.SYNCBUSY);   // Wait for synchronization
    NVIC_SetPriority(AC_IRQn, 0);       // Set AC interrupt priority to highest
    NVIC_EnableIRQ(AC_IRQn);            // Enable Interrupts for AC at NVIC

    AC->CTRLA.bit.ENABLE = 1;                // Enable the analog comparator peripheral
    while (AC->STATUSB.bit.SYNCBUSY);        // Wait for synchronization

    AC->COMPCTRL[1].bit.ENABLE = 1;          // Enable the analog comparator channel 1
    while (AC->STATUSB.bit.SYNCBUSY);        // Wait for synchronization
}

void setup_DAC() {
    /**
     * @brief Setup the Digital to Analog Converter.
    */
    // Power Managment: Activate the DAC peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_DAC;        // Activate the digital to analog converter peripheral

    // Generic clock: Route the generic clocks
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 48MHz GCLK0 to the digital to analog converter clock
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_DAC;
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

    // PORT: Connect the DAC output to PIN A0. Useful for DEBUGGING
    if(ALL_OUTPUTS) {
        PORT->Group[g_APinDescription[REF_PIN].ulPort].PINCFG[g_APinDescription[REF_PIN].ulPin].bit.PMUXEN = 1;                       // Enable PORT multiplexer
        PORT->Group[g_APinDescription[REF_PIN].ulPort].PMUX[g_APinDescription[REF_PIN].ulPin].bit.PMUXO = PORT_PMUX_PMUXO_B_Val;      // Select channel B = DAC output = VOUT
    }

    // Digital to Analog Converter: Setup the DAC
    DAC->CTRLB.reg =  DAC_CTRLB_REFSEL_INT1V |    // Set 1V as reference voltage
                      DAC_CTRLB_IOEN |            // Enable output to internal reference
                      DAC_CTRLB_EOEN;             // Enable output on VOUT pin. Useful for DEBUGGING
    while(DAC->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    DAC->CTRLA.bit.ENABLE = 1;                    // Enable the DAC before writing data
    while(DAC->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    DAC->DATA.reg = DAC_DATA_DATA(100);           // 10 bit resolution on 1000mV. 100 => ~100 mv
    while(DAC->STATUS.bit.SYNCBUSY);              // Wait for synchronization
}

void setup_EVSYS() {
    /**
     * @brief Setup the Event System.
    */
    // Power Managment: Activate the event system peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;      // Activate the event system peripheral
    
    // Event System: Setup the event system
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);                // Set the event user (receiver) as timer TC3

    EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);                // Set the event user (receiver) as timer TC4

    EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection (because asynchronous)
                        EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC0_MCX_3) |        // Set event generator (sender) as TCC 0
                        EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0 
} 

void setup_TC3() {
    /**
     * @brief Setup the Timer Counter 3.
    */
    // Power Managment: Activate the timer counter 3 peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_TC3;        // Activate the timer counter 3 peripheral

    // Generic clock: Route the generic clocks
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 48MHz GCLK0 to the Timer Counter 2 & 3 clock
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_TCC2_TC3;
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

    // TC3: Setup the timer
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;   // Configure TC3 timer for 16-bit mode
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);          // Wait for synchronization

    TC3->COUNT16.EVCTRL.reg |= TC_EVCTRL_TCEI |       // Enable input event
                                TC_EVCTRL_EVACT_RETRIGGER;   // Select event action
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);          // Wait for synchronization

    TC3->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;    // Enable oneshot mode
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);            // Wait for synchronization

    TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;      // Enable compare match interrupt on channel 0
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);          // Wait for synchronization
    NVIC_SetPriority(TC3_IRQn, 2);       // Set TC3 interrupt priority
    NVIC_EnableIRQ(TC3_IRQn);            // Enable Interrupts for TC3 at NVIC

    TC3->COUNT16.CC[0].reg = COILCHANGE_DELAY_US * MAIN_CLK_FREQ_MHZ;   // Set the compare match value
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);          // Wait for synchronization

    TC3->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ;    // Normal Frequency mode
    while(TC3->COUNT16.STATUS.bit.SYNCBUSY);            // Wait for synchronization

    TC3->COUNT16.CTRLA.bit.ENABLE = 1;          // Enable timer TC3
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);   // Wait for synchronization
}

void setup_TC4() {
    /**
     * @brief Setup the Timer Counter 4.
    */
    // Power Managment: Activate the timer counter 4 peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_TC4;        // Activate the timer counter 4 peripheral

    // Generic clock: Route the generic clocks
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 48MHz GCLK0 to the Timer Counter 4 & 5 clock
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_TC4_TC5;
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

    // TC4: Setup the timer
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;   // Configure TC4/TC5 timers for 16-bit mode
    while(TC4->COUNT16.STATUS.bit.SYNCBUSY);          // Wait for synchronization

    TC4->COUNT16.CTRLC.bit.CPTEN0 = 1;          // Enable capture on CC0
    while(TC4->COUNT16.STATUS.bit.SYNCBUSY);    // Wait for synchronization

    TC4->COUNT16.EVCTRL.reg |= TC_EVCTRL_TCEI |       // Enable input event
                                TC_EVCTRL_EVACT_RETRIGGER;   // Select event action
    while(TC4->COUNT16.STATUS.bit.SYNCBUSY);          // Wait for synchronization

    TC4->COUNT16.CTRLBSET.bit.ONESHOT |= 1;     // Enable oneshot mode
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);   // Wait for synchronization

    TC4->COUNT16.READREQ.reg = TC_READREQ_RCONT |                               // Enable a continuous read request
                                TC_READREQ_ADDR(TC_COUNT16_COUNT_OFFSET);        // Offset of the 32-bit COUNT register
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                                   // Wait for synchronization

    TC4->COUNT16.CTRLA.bit.ENABLE = 1;          // Enable timer TC4
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);   // Wait for synchronization
}

void setup_TCC0() {
    /**
     * @brief Setup the Timer Counter for Control 0.
    */
    // Power Managment: Activate the timer counter 0 peripheral
    PM->APBCMASK.reg |= PM_APBCMASK_TCC0;       // Activate the timer counter 0 peripheral

    // Generic clock: Route the generic clocks
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route the 48MHz GCLK0 to the Timer Counter 0 & 1 clock
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_TCC0_TCC1;
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    
    // PORT: Configure port and pins
    PORT->Group[g_APinDescription[PULSE_PIN].ulPort].PINCFG[g_APinDescription[PULSE_PIN].ulPin].bit.PMUXEN = 1;                     // Enable PORT multiplexer
    PORT->Group[g_APinDescription[PULSE_PIN].ulPort].PMUX[g_APinDescription[PULSE_PIN].ulPin >> 1].reg |= PORT_PMUX_PMUXO_F;        // Select channel F = Timer TCC0 waveform output = TCC0/WO[4]

    // TCC0: Setup the timer
    TCC0->CTRLA.bit.PRESCALER |= TCC_CTRLA_PRESCALER_DIV64_Val;   // Divide 48MHz clock by 64 => 750KHz

    // Normal (single slope) PWM operation: timer countinuouslys count up to PER register value and then is reset to 0
    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM |     // Setup single slope PWM on TCC0
                    TCC_WAVE_POL3;              // Invert output polarity
    while (TCC0->SYNCBUSY.bit.WAVE);              // Wait for synchronization

    TCC0->PER.reg = MAIN_CLK_FREQ_MHZ*1000000/(64 * PULSE_FREQ_HZ)-1;   // Set the frequency of the PWM on TCC0
    while (TCC0->SYNCBUSY.bit.PER);                                     // Wait for synchronization

    TCC0->CC[3].reg = (PULSE_WIDTH_US * MAIN_CLK_FREQ_MHZ)/64;          // Set the pulsewidth of the PWM on TCC0
    while (TCC0->SYNCBUSY.bit.CC3);                                     // Wait for synchronization

    TCC0->EVCTRL.reg |= TCC_EVCTRL_MCEO3;           // Enable Match/Capture event output

    TCC0->CTRLA.bit.ENABLE = 1;                     // Enable the TCC0 counter
    while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

uint16_t map_threshold(uint16_t threshold) {
    /**
     * @brief Map the threshold value to the 10 bit resolution.
     * 
     * @param threshold The threshold to map.
     * @return uint16_t The mapped threshold.
    */
    uint16_t tmp = (1023 - threshold)/1023.0 * MAX_THRESHOLD_MV + MIN_THRESHOLD_MV;

    if(tmp > MAX_THRESHOLD_MV) {tmp = MAX_THRESHOLD_MV;}
    if(tmp < MIN_THRESHOLD_MV) {tmp = MIN_THRESHOLD_MV;}

    return tmp;
}

void select(uint8_t coil) {
    /**
     * @brief Select the coil to pulse.
     *  
     * @param channel The channel to select.
    */ 
    if(coil & 0b100) {digitalWrite(COILSELC_PIN, HIGH);}
    else {digitalWrite(COILSELC_PIN, LOW);}
    if(coil & 0b010) {digitalWrite(COILSELB_PIN, HIGH);}
    else {digitalWrite(COILSELB_PIN, LOW);}
    if(coil & 0b001) {digitalWrite(COILSELA_PIN, HIGH);}
    else {digitalWrite(COILSELA_PIN, LOW);}
    delay(1);
}

void tare_coil(uint8_t coil) {
    /**
     * @brief Tare one coil.
     * 
     * @param coil The coil to tare.
    */
    // Select the coil
    select(coil);
    delay(100);
    meas.tare[coil] = captured_value;
}

uint8_t select_next_coil(uint8_t last_coil) {
    /**
     * @brief Select the next coil to pulse.
     * 
     * @param last_coil The last coil that was pulsed.
    */
    for(uint8_t i = 1; i < NB_COILS; i++){
        uint8_t next_coil = (last_coil + i) % NB_COILS;
        if(active_coils[next_coil]) {
            select(next_coil);
            return next_coil;
        }
    }
    return 0;
}

} // namespace pulse



// Interrupt handler for the Analog Comparator. Must be outside of any namespace
void AC_Handler(void) {
    // Check if compare interrupt
    if(AC->INTFLAG.bit.COMP1 && AC->INTENSET.bit.COMP1) {
        // Read timer value
        pulse::captured_value = TC4->COUNT16.COUNT.reg;

        // Save the captured value
        pulse::meas.captured_value[pulse::pulsing_coil] = pulse::captured_value;
        
        // Clear interrupt flag by writing '1' to it
        AC->INTFLAG.reg = AC_INTFLAG_COMP1;
    }
}

// Interrupt handler for the Timer 3. Must be outside of any namespace
void TC3_Handler(void) {
    // Check if compare interrupt
    if(TC3->COUNT16.INTFLAG.bit.MC0 && TC3->COUNT16.INTENSET.bit.MC0) {
        // Do something
        pulse::pulsing_coil = pulse::select_next_coil(pulse::pulsing_coil);

        // Clear interrupt flag by writing '1' to it
        TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    }
}
