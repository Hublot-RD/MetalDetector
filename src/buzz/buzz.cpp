#include "buzz.hpp"


#define DURATION2MS(duration)   (60000 * 4) / (BPM * duration)


namespace buzzer {
// Define a structure to hold a note and its duration
struct Note {
  int pitch;
  int duration;
};

// Mario main theme melody
Note mario_melody[] = {
    {NOTE_E7, QUARTER_NOTE}, {NOTE_E7, QUARTER_NOTE}, {NOTE_SILENCE, QUARTER_NOTE}, {NOTE_E7, QUARTER_NOTE},
    {NOTE_SILENCE, QUARTER_NOTE}, {NOTE_C7, QUARTER_NOTE}, {NOTE_E7, QUARTER_NOTE}, {NOTE_SILENCE, QUARTER_NOTE},
    {NOTE_G7, QUARTER_NOTE}, {NOTE_SILENCE, 4}, {NOTE_G6, HALF_NOTE}, {NOTE_SILENCE, WHOLE_NOTE},
};

// Metallica - Master of Puppets melody
Note metallica_melody[] = {
    {NOTE_E5, QUARTER_NOTE}, {NOTE_SILENCE, QUARTER_NOTE}, {NOTE_SILENCE, HALF_NOTE},
    {NOTE_DS6, QUARTER_NOTE}, {NOTE_SILENCE, QUARTER_NOTE},
    {NOTE_D6, QUARTER_NOTE}, {NOTE_SILENCE, QUARTER_NOTE},
    {NOTE_C6, WHOLE_NOTE-1}, {NOTE_SILENCE, WHOLE_NOTE},
};

void setup() {
    /**
     * @brief Setup the buzzer pin
     * 
     * @param buzzerPin The pin where the buzzer is connected
    */
    pinMode(BUZZER_PIN, OUTPUT);
}

void playNote(int pitch, int duration) {
    int noteDuration = DURATION2MS(duration);
    tone(BUZZER_PIN, pitch, noteDuration*0.9);
    delay(noteDuration);
    noTone(BUZZER_PIN); // Stop playing the note 
}

void playMelodyMario() {
    /**
     * @brief Play the melody of Mario's main theme
     * 
     * @param buzzerPin The pin where the buzzer is connected
    */
    size_t noteCount = sizeof(mario_melody)/sizeof(Note);

    for (size_t thisNote = 0; thisNote < noteCount; thisNote++) {
        playNote(mario_melody[thisNote].pitch, mario_melody[thisNote].duration);
    }
}

void playMelodyMetallica() {
    /**
     * @brief Play the melody of Metallica's Master of Puppets
     * 
     * @param buzzerPin The pin where the buzzer is connected
    */
    size_t noteCount = sizeof(metallica_melody)/sizeof(Note);

    for (size_t thisNote = 0; thisNote < noteCount; thisNote++) {
        playNote(metallica_melody[thisNote].pitch, metallica_melody[thisNote].duration);
    }
}

void playMetal(uint32_t time_shifting, uint32_t max_time_shifting, uint32_t lower_threshold, uint32_t beep_duration=40) {
    /**
     * @brief Play a tone according to metal detection
     * 
     * @param time_shifting The time shifting value, measured by the metal detector
     * @param max_time_shifting The time shifting value that will correspond to the highest note. Must be expressed in the same unit as "time_shifting".
     * @param lower_threshold The time shifting value that will correspond to the lowest note. Must be expressed in the same unit as "time_shifting".
     * @param beep_duration How long a beep is, expressed in milliseconds [ms].
    */
    if (time_shifting < lower_threshold) {
        noTone(BUZZER_PIN);
        return;
    } else {
        time_shifting = max(min(time_shifting, max_time_shifting), 0);
        int pitch = map(time_shifting, 0, max_time_shifting, NOTE_METAL_LOWEST, NOTE_METAL_HIGHEST);
        
        tone(BUZZER_PIN, pitch, beep_duration);
        return;
    }
}

} // namespace buzz