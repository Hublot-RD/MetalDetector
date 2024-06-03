#ifndef PARAMETERS_H
#define PARAMETERS_H

// Debugging options
#define DEBUG true

// Physical constants
#define MAIN_CLK_FREQ_MHZ 48      // Main clock frequency [MHz]. This cannot be modified.

// Timing parameters
#define STARTUP_TIME_MS   500     // Time to let the supply capacitor charge before starting to do pulses [ms]
#define LOOP_FREQ_HZ      10       // Main loop frequency [Hz]

// Pulse parameters
#define PULSE_WIDTH_US    200     // Length of a pulse [us]
#define PULSE_FREQ_HZ     30      // Pulse frequency [Hz]

// Serial communication
#define SERIAL_BAUD_RATE  115200  // Baud rate for the USB communication

#endif // PARAMETERS_H