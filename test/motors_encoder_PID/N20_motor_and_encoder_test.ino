  #include <Arduino.h>
   /*
  DRV8833-Dual-Motor-Driver-Module
  made on 23 Nov 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

#define BIN1 14
#define BIN2 25
#define AIN1 26 //RIGHT REVERSE
#define AIN2 27 //RIGHT FORWARD


int encoderPin1 = 34; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 35; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value

void setup() {
  Serial.begin(115200);
  
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  
}
 
void loop() {
  
  analogWrite(AIN1,255); 
  analogWrite(AIN2,0);
  analogWrite(BIN1,255); 
  analogWrite(BIN2,0);

  Serial.println(encoderValue);

  delay(1000);
  
  analogWrite(AIN1,0); 
  analogWrite(AIN2,255);
  analogWrite(BIN1,0); 
  analogWrite(BIN2,255);
  Serial.println(encoderValue);
  delay(1000);
  
  analogWrite(AIN1,0); 
  analogWrite(AIN2,0);
  analogWrite(BIN1,0); 
  analogWrite(BIN2,0);
  Serial.println(encoderValue);
  delay(1000);
  
  analogWrite(AIN1,0); 
  analogWrite(AIN2,0);
  analogWrite(BIN1,0); 
  analogWrite(BIN2,0);
    Serial.println(encoderValue);
  delay(1000);
   
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  lastEncoded = encoded; //store this value for next time

}
