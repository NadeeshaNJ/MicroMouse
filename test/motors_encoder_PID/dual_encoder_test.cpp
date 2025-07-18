  #include <Arduino.h>
   /*
  DRV8833-Dual-Motor-Driver-Module
  made on 23 Nov 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

int encoderPin1L = 18; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2L = 5; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncodedL = 0; // Here updated value of encoder store.
volatile long encoderValueL = 0; // Raw encoder value
int encoderPin1R = 19; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2R = 23; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncodedR = 0; // Here updated value of encoder store.
volatile long encoderValueR = 0; // Raw encoder value

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

void setup() {
  Serial.begin(115200);
  
  pinMode(encoderPin1L, INPUT_PULLUP); 
  pinMode(encoderPin2L, INPUT_PULLUP);
  pinMode(encoderPin1R, INPUT_PULLUP); 
  pinMode(encoderPin2R, INPUT_PULLUP);

  //digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  //digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  attachInterrupt(digitalPinToInterrupt(encoderPin1L), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2L), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin1R), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2R), updateEncoderR, CHANGE);
  
}
 
void loop() {

  Serial.print(encoderValueL);
  Serial.print(" L  R ");
  Serial.println(encoderValueR);
  delay(1000);
}