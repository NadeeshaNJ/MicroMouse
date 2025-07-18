#ifndef MotorPIDbyNJ_h
#define MotorPIDbyNJ_h

#include <Arduino.h>

class MotorPIDbyNJ {
private:
    int pin1, pin2;
    int encoderPin1, encoderPin2;

    volatile long encoderValue;
    volatile int lastEncoded;

    float kp, ki, kd; //PID
    float previousError;
    float integral;
    long previousTime;

    int speed; // Speed of the motor, 0-255    
    int direction; // Direction of the motor, 1 for forward, -1 for backward
    bool isDone;
    long target;
    int tolerance;    

public:
    MotorPIDbyNJ(int pin1, int pin2, int encoderPin1, int encoderPin2);

    void attachEncoderInterrupt(void (*ISR)());
    void calculatePID(long error);
    void setPID(float kp, float ki, float kd, int tol);
    void setTarget(long target);

    //control methods
    void updateEncoder();    
    void setEncoderValue(long value) { encoderValue = value; }
    long getEncoderValue() const { return encoderValue; }
    void resetEncoder() { encoderValue = 0; }
    void setDirection(int dir) { direction = dir; }
    bool done() const { return isDone; }

    void runMotor(); // Apply motor power
    void update(); // Run motor with PID control
    void reachTarget(long target); // Set target and reach it
    

};

#endif
