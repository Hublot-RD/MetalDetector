#include "buzz.hpp"
#include "pitches.hpp"

#define BPM 168
#define DURATION2MS(duration)   (60000 * 4) / (BPM * duration)

#define WHOLE_NOTE 3
#define HALF_NOTE 6
#define QUARTER_NOTE 12
#define EIGHTH_NOTE 24

#define NOTE_METAL_LOWEST NOTE_E6
#define NOTE_METAL_HIGHEST NOTE_DS7


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

void playNote(uint32_t buzzerPin, int pitch, int duration) {
    int noteDuration = DURATION2MS(duration);
    tone(buzzerPin, pitch, noteDuration*0.9);
    delay(noteDuration);
    noTone(buzzerPin); // Stop playing the note 
}

void playMelodyMario(uint32_t buzzerPin) {
    /**
     * @brief Play the melody of Mario's main theme
     * 
     * @param buzzerPin The pin where the buzzer is connected
    */
    size_t noteCount = sizeof(mario_melody)/sizeof(Note);

    for (size_t thisNote = 0; thisNote < noteCount; thisNote++) {
        playNote(buzzerPin, mario_melody[thisNote].pitch, mario_melody[thisNote].duration);
    }
}

void playMelodyMetallica(uint32_t buzzerPin) {
    /**
     * @brief Play the melody of Metallica's Master of Puppets
     * 
     * @param buzzerPin The pin where the buzzer is connected
    */
    size_t noteCount = sizeof(metallica_melody)/sizeof(Note);

    for (size_t thisNote = 0; thisNote < noteCount; thisNote++) {
        playNote(buzzerPin, metallica_melody[thisNote].pitch, metallica_melody[thisNote].duration);
    }
}

void playMetal(uint32_t buzzerPin, uint32_t time_shifting, uint32_t max_time_shifting, uint32_t lower_threshold, uint32_t beep_duration=40) {
    /**
     * @brief Play a tone according to metal detection
     * 
     * @param buzzerPin The pin where the buzzer is connected
     * @param time_shifting The time shifting value, measured by the metal detector
     * @param max_time_shifting The time shifting value that will correspond to the highest note. Must be expressed in the same unit as "time_shifting".
     * @param lower_threshold The time shifting value that will correspond to the lowest note. Must be expressed in the same unit as "time_shifting".
     * @param beep_duration How long a beep is, expressed in milliseconds [ms].
    */
    if (time_shifting < lower_threshold) {
        noTone(buzzerPin);
        return;
    } else {
        time_shifting = max(min(time_shifting, max_time_shifting), 0);
        int pitch = map(time_shifting, 0, max_time_shifting, NOTE_METAL_LOWEST, NOTE_METAL_HIGHEST);
        
        tone(buzzerPin, pitch, beep_duration);
        return;
    }
}