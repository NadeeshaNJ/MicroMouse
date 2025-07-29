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
    float integralEncoderError;
    float diffEncoderError;
    long previousTime;
    int encoderPID;
    int speed; // Speed of the motor, 0-255    
    int direction; // Direction of the motor, 1 for forward, -1 for backward
    bool isDone;
    long target;
    int tolerance = 5; // 5mm Tolerance for PID control, can be adjusted
    long metricConverter = 25.6; // Conversion factor for encoder value, can be adjusted (25.6 encoder = 1 mm)

public:
    MotorPIDbyNJ(int pin1, int pin2, int encoderPin1, int encoderPin2);

    void attachEncoderInterrupt(void (*ISR)());
    void setPID(float kp, float ki, float kd, int tol);

    //control methods
    int calculateEncoderPID();
    void updateEncoder();    
    long getEncoderValue() const { return encoderValue; }
    void resetEncoder(){encoderValue = 0;}
    void setDirection(int dir) { direction = dir; }
    void setSpeed(int speed){this->speed = speed;}; // Set PWM speed for the motor
    void setTarget(long target) { this->target = target; }
    void runMotor(int speed);
    bool checkDone();
};

#endif
