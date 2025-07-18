#include <Arduino.h>

int step = 0; // Initial step

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

int pos=0;
long prevT=0;
float eprevL=0;
float eprevR=0;
float eintegralL=0;
float eintegralR=0;

struct EncoderMotor {
  int pin1;
  int pin2;
  int speed;
  int direction;
  volatile long *encoderValue;
  long targetTicks;
};

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


bool leftDone = false;
bool rightDone = false;

void runMotors(EncoderMotor &motorL, EncoderMotor &motorR){
  
  if(motorL.direction == 1) {
    analogWrite(motorL.pin1, motorL.speed); // Forward
    analogWrite(motorL.pin2, 0);
  } else {
    analogWrite(motorL.pin1, 0);
    analogWrite(motorL.pin2, motorL.speed); // Backward
  }  
  if(motorR.direction == 1) {
    analogWrite(motorR.pin1, motorR.speed); // Forward
    analogWrite(motorR.pin2, 0);
  }else {
    analogWrite(motorR.pin1, 0);
    analogWrite(motorR.pin2, motorR.speed); // Backward
  }
  
}

EncoderMotor leftMotor = {LIN1, LIN2, 0, 1, &encoderValueL, 2000};
EncoderMotor rightMotor = {RIN1, RIN2, 0, 1, &encoderValueR, 2000};

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
  
  int target = 4000;
  float kpL = 0.68;
  float kdL = 0.04;
  float kiL = 0.0;
  float kpR = 0.68;
  float kdR = 0.04;
  float kiR = 0.0;

  long currT=micros();
  float deltaT=((float)(currT-prevT))/1.0e6;
  if (deltaT <= 0.000001) deltaT = 0.000001;
  prevT = currT;
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

  int eL = target - encoderValueL; // Error for left motor
  int eR = target - encoderValueR; // Error for right motor

  //derivative
  float dedtL = (eL-eprevL)/deltaT;
  eprevL = eL;
  float dedtR = (eR-eprevR)/deltaT;
  eprevR = eR;
  //integral
  eintegralL += eL*deltaT;
  eintegralR += eR*deltaT;

  eintegralL = constrain(eintegralL, -300, 300); //to clamp the integral term
  eintegralR = constrain(eintegralR, -300, 300);

  float uL = kpL*eL + kdL*dedtL + kiL*eintegralL;
  float uR = kpR*eR + kdR*dedtR + kiR*eintegralR;

  int pwrL = constrain(fabs(uL), 0, 255);
  int pwrR = constrain(fabs(uR), 0, 255);

  // if (abs(eL) < 150) pwrL = 0; // stop left motor
  // if (abs(eR) < 150) pwrR = 0; // stop right motor
  
  int dirL = (uL < 0) ? -1 : 1;
  int dirR = (uR < 0) ? -1 : 1;

  if (!leftDone && abs(eL) < 50) {
    analogWrite(leftMotor.pin1, 0);
    analogWrite(leftMotor.pin2, 0);
    leftDone = true;
    Serial.println("Left motor done");
  }
  if (!rightDone && abs(eR) < 50) {
    analogWrite(rightMotor.pin1, 0);
    analogWrite(rightMotor.pin2, 0);
    rightDone = true;
    Serial.println("Right motor done");
  }

  if (leftDone) pwrL = 0;
  if (rightDone) pwrR = 0;

  Serial.print("error L: "); Serial.print(eL);
  Serial.print(" | error R: "); Serial.println(eR);  
  Serial.print("uL: "); Serial.print(uL);
  Serial.print(" | uR: "); Serial.println(uR);
  Serial.print("PWM L: "); Serial.print(pwrL); Serial.print("("); Serial.print(dirL); Serial.print(")");
  Serial.print(" | PWM R: "); Serial.print(pwrR); Serial.print("("); Serial.print(dirR); Serial.println(")");

  Serial.println("-------------------------------------------");
  // Set the motor speeds and directions
  // EncoderMotor leftMotor = {LIN1, LIN2, pwrL, dirL, &encoderValueL, target, false};
  // EncoderMotor rightMotor = {RIN1, RIN2, pwrR, dirR, &encoderValueR, target, false};

  leftMotor.speed = pwrL;
  leftMotor.direction = dirL;
  leftMotor.targetTicks = target;
  rightMotor.speed = pwrR;
  rightMotor.direction = dirR;
  rightMotor.targetTicks = target;

  switch (step) {
    case 0://forward
      runMotors(leftMotor, rightMotor);
      if (leftDone && rightDone) {
        Serial.print("FIN ENC L: ");
        Serial.print(encoderValueL);
        Serial.print(" | FIN ENC R: ");
        Serial.println(encoderValueR);
        eprevL = eprevR = eintegralL = eintegralR = 0;
        encoderValueL = encoderValueR = 0;
        leftDone = rightDone = false;
        step = 7;
      }
      
      break;

    case 1://backward
      runMotors(leftMotor, rightMotor);
      eprevL = eprevR = eintegralL = eintegralR = 0;
      encoderValueL = encoderValueR = 0;
      step = 7;  // go to next stage

      break;

    case 2://turn left
      runMotors(leftMotor, rightMotor);
      eprevL = eprevR = eintegralL = eintegralR = 0;
      step = 7;

      break;

    case 3://turn right
      runMotors(leftMotor, rightMotor);
      eprevL = eprevR = eintegralL = eintegralR = 0;
      step = 7;  // go to next stage

      break;  
    case 4://move left (INCOMLETE)
      runMotors(leftMotor, rightMotor);
      eprevL = eprevR = eintegralL = eintegralR = 0;
      step = 7;

      break;
    case 7://stop
      // add more behavior here (like stop forever)
      Serial.println("All movements done.");
      Serial.print("FINAL ENC L: ");
      Serial.print(encoderValueL);
      Serial.print(" | FINAL ENC R: ");
      Serial.println(encoderValueR);
      while (1);
      break;
  }

  delay(10);
}

