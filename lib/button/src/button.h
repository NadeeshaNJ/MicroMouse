#ifndef ButtonHandler_h
#define ButtonHandler_h

#include <Arduino.h>

class button {
  private:
    int pin;
    bool buttonState;
    bool lastButtonState;
    bool waitForSecondPress;
    unsigned long lastPressTime;
    unsigned long doublePressDelay;

    void (*singlePressCallback)();
    void (*doublePressCallback)();

  public:
    button(int pin, unsigned long doubleDelay = 1000);

    void setSinglePressCallback(void (*func)());
    void setDoublePressCallback(void (*func)());
    void update();
};

#endif
