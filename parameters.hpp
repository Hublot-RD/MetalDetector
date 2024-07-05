#ifndef PARAMETERS_H
#define PARAMETERS_H

// Timing parameters
#define STARTUP_TIME_MS   1000    // Time to let the supply capacitor charge before starting to do pulses [ms]
#define LOOP_FREQ_HZ      5       // Main loop frequency [Hz]

// Serial communication
#define DEBUG             true    // Enable debug messages
#define SERIAL_BAUD_RATE  115200  // Baud rate for the USB communication

#endif // PARAMETERS_H