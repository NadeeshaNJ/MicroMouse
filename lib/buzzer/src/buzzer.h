#ifndef buzzer_h
#define buzzer_h
#include <Arduino.h>

class buzzer {
private:
const int pin;
unsigned long now = millis();
unsigned long startTime = 0;
int frequency;
int duration;
bool buzzing = false;
unsigned long beepDuration;
int toggleInterval;
bool sequenceActive;
int toneIndex;
unsigned long nextToneTime;
struct ToneSequence {
    int freq;
    int dur;
} tones[10];
int sequenceLength;

void startSequence(const ToneSequence* sequence, int length);

public:
    buzzer(const int pin);

    void startSound();
    void playTone(int frequency, int duration);
    void stopSound();
    void update();

    void beepOnce(); //tih
    void beepTwice(); //tih tih
    void errorBeep(); //ti ti tih
    void sensorFailBeep(); //tee teeek
    void successBeep(); // tiririk
    void startupTone(); // tiriririih(up-play)
    void shutdownTone(); //tiriririih(down-play)
    void warningTone(); //treeh treeh (alarming)
    void movementCompleteTone(); //titih
    void alertTone(); //niriririh(up)
    

};
#endif
