#include <Arduino.h>

int step = 0; // Initial step (forward,backward,left,right,move left move right,stop)

const int LIN1 = 25;
const int LIN2 = 26;
const int RIN1 = 14;
const int RIN2 = 27;

int encoderPin1L = 18; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2L = 5; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncodedL = 0; // Here updated value of encoder store.
volatile long encoderValueL = 0; // Raw encoder value
int encoderPin1R = 19; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2R = 23; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncodedR = 0; // Here updated value of encoder store.
volatile long encoderValueR = 0; // Raw encoder value

struct EncoderMotor {
  int pin1;
  int pin2;
  int speed1;
  int speed2;
  volatile long *encoderValue;
  long targetTicks;
  bool running;
};

EncoderMotor leftForwardMotor = {LIN1, LIN2, 255, 0, &encoderValueL, 2000, false};
EncoderMotor rightForwardMotor = {RIN1, RIN2, 255, 0, &encoderValueR, 2000, false};
EncoderMotor leftBackwardMotor = {LIN1, LIN2, 0, 255, &encoderValueL, 2000, false};
EncoderMotor rightBackwardMotor = {RIN1, RIN2, 0, 255, &encoderValueR, 2000, false};

void updateEncoderL(){
  int MSB = digitalRead(encoderPin1L); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2L); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedL << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueL --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueL ++;

  lastEncodedL = encoded; //store this value for next time
}
void updateEncoderR(){
  int MSB = digitalRead(encoderPin1R); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2R); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedR << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueR --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueR ++;

  lastEncodedR = encoded; //store this value for next time
}

void runMotors(EncoderMotor &motorL, EncoderMotor &motorR){
  if (!motorL.running && !motorR.running) {
    analogWrite(motorL.pin1, motorL.speed1);
    analogWrite(motorL.pin2, motorL.speed2);
    motorL.running = true;

    analogWrite(motorR.pin1, motorR.speed1);
    analogWrite(motorR.pin2, motorR.speed2);
    motorR.running = true;
  }

  if (motorL.running && abs(*motorL.encoderValue) >= motorL.targetTicks) {
    analogWrite(motorL.pin1, 0);
    analogWrite(motorL.pin2, 0);
    motorL.running = false;
    Serial.println("Target ticks reached — left motor stopped.");
  }
  if (motorR.running && abs(*motorR.encoderValue) >= motorR.targetTicks) {
    analogWrite(motorR.pin1, 0);
    analogWrite(motorR.pin2, 0);
    motorR.running = false;
    Serial.println("Target ticks reached — right motor stopped.");
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(LIN1,OUTPUT);
  pinMode(LIN2,OUTPUT);
  pinMode(RIN1,OUTPUT);
  pinMode(RIN2,OUTPUT);
  
  pinMode(encoderPin1L, INPUT_PULLUP); 
  pinMode(encoderPin2L, INPUT_PULLUP);
  pinMode(encoderPin1R, INPUT_PULLUP); 
  pinMode(encoderPin2R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin1L), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2L), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1R), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2R), updateEncoderR, CHANGE);
  
  //runMotorForward(leftEncoderMotor,rightEncoderMotor);
}

void loop() {
  Serial.print("ENC L: ");
  Serial.print(encoderValueL);
  Serial.print(" | ENC R: ");
  Serial.println(encoderValueR);

  switch (step) {
    case 0://forward
      runMotors(leftForwardMotor, rightForwardMotor);
      if (!leftForwardMotor.running && !rightForwardMotor.running) {
        *leftForwardMotor.encoderValue = 0;
        *rightForwardMotor.encoderValue = 0;
        step = 7;  // go to next stage
      }
      break;

    case 1://backward
      runMotors(leftBackwardMotor, rightBackwardMotor);
      if (!leftBackwardMotor.running && !rightBackwardMotor.running) {
        *leftBackwardMotor.encoderValue = 0;
        *rightBackwardMotor.encoderValue = 0;
        step = 7;  // go to next stage
      }
      break;

    case 2://turn left
      runMotors(leftBackwardMotor, rightForwardMotor);
      if (!leftBackwardMotor.running && !rightForwardMotor.running) {
        *leftBackwardMotor.encoderValue = 0;
        *rightForwardMotor.encoderValue = 0;
        step = 7;
      }
      break;

    case 3://turn right
      runMotors(leftForwardMotor, rightBackwardMotor);
      if (!leftForwardMotor.running && !rightBackwardMotor.running) {
        *leftForwardMotor.encoderValue = 0;
        *rightBackwardMotor.encoderValue = 0;
        step = 7;  // go to next stage
      }
      break;  
    case 4://move left (INCOMLETE)
      runMotors(leftBackwardMotor, rightForwardMotor);
      if (!leftBackwardMotor.running && !rightForwardMotor.running) {
        *leftBackwardMotor.encoderValue = 0;
        *rightForwardMotor.encoderValue = 0;
        step = 7;
      }
      break;
    case 7://stop
      // add more behavior here (like stop forever)
      Serial.println("All movements done.");
      while (1);
      break;
  }

  delay(100);
}

