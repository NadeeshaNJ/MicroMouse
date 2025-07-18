#include <buzzer.h>

buzzer::buzzer(const int pin) : pin(pin) {
    pinMode(pin, OUTPUT);
    ledcSetup(0, 4000, 8);           // Channel 0, 2 kHz base freq, 8-bit resolution
    ledcAttachPin(pin, 0);          // Attach buzzer pin to channel 0
}

void buzzer::startSound(){
    startTime = millis();
    digitalWrite(pin, HIGH);
    beepDuration = 100; // Default duration for the start sound
    buzzing = true;
}

void buzzer::playTone(int frequency, int duration) {
    startTime = millis();
    ledcWriteTone(0, frequency);
    beepDuration = duration;
    buzzing = true;
}

void buzzer::stopSound(){ // Use this in the begining of the loop so the buzzer will try top stop everytime the loop runs if the duration has passed
    if (buzzing && millis() - startTime >= beepDuration) {
    ledcWriteTone(0, 0);  // stop tone
    digitalWrite(pin, LOW);
    buzzing = false;
  }
}
void buzzer::update() {
    stopSound();

    if (sequenceActive && millis() >= nextToneTime) {
        if (toneIndex < sequenceLength) {
            ledcWriteTone(0, tones[toneIndex].freq);
            nextToneTime = millis() + tones[toneIndex].dur;
            toneIndex++;
        } else {
            ledcWriteTone(0, 0);
        }
    }
}
void buzzer::startSequence(const ToneSequence* sequence, int length) {
    memcpy(tones, sequence, sizeof(ToneSequence) * length);
    sequenceLength = length;
    toneIndex = 0;
    nextToneTime = 0;
    sequenceActive = true;
}
void buzzer::beepOnce() {
    ToneSequence seq[] = { {3000, 100} };
    startSequence(seq, 1);
}

void buzzer::beepTwice() {
    ToneSequence seq[] = { {3000, 100}, {0, 100}, {3000, 100} };
    startSequence(seq, 3);
}

void buzzer::errorBeep() {
    ToneSequence seq[] = { {400, 150}, {0, 100}, {400, 150}, {0, 100}, {400, 150} };
    startSequence(seq, 5);
}

void buzzer::sensorFailBeep() {
    ToneSequence seq[] = { {2200, 200}, {0, 100}, {2200, 200} };
    startSequence(seq, 3);
}

void buzzer::successBeep() {
    ToneSequence seq[] = { {3200, 100}, {3400, 100}, {3600, 100} };
    startSequence(seq, 3);
}

void buzzer::startupTone() {
    ToneSequence seq[] = { {2500, 100}, {2700, 100}, {2900, 100}, {3100, 100} };
    startSequence(seq, 4);
}

void buzzer::shutdownTone() {
    ToneSequence seq[] = { {3100, 100}, {2900, 100}, {2700, 100}, {2500, 100} };
    startSequence(seq, 4);
}

void buzzer::warningTone() {
    ToneSequence seq[] = { {2600, 300}, {0, 100}, {2600, 300} };
    startSequence(seq, 3);
}

void buzzer::movementCompleteTone() {
    ToneSequence seq[] = { {2800, 100}, {0, 100}, {3000, 100} };
    startSequence(seq, 3);
}

void buzzer::alertTone() {
    ToneSequence seq[] = { {3500, 100}, {3000, 100}, {3500, 100}, {3000, 100} };
    startSequence(seq, 4);
}
