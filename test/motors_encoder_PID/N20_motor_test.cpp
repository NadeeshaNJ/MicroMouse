 #include <Arduino.h>
   /*
  DRV8833-Dual-Motor-Driver-Module
  made on 23 Nov 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

*/

const int LIN1 = 25;
const int LIN2 = 26;
const int RIN1 = 14;
const int RIN2 = 27;


void setup() {
  Serial.begin(115200);

  pinMode(LIN1,OUTPUT);
  pinMode(LIN2,OUTPUT);
  pinMode(RIN1,OUTPUT);
  pinMode(RIN2,OUTPUT);
  
}
 
void loop() {
  //forward
  analogWrite(LIN1,255); 
  analogWrite(LIN2,0);
  analogWrite(RIN1,255); 
  analogWrite(RIN2,0);
  delay(1000);

  //backward
  analogWrite(LIN1,0); 
  analogWrite(LIN2,255);
  analogWrite(RIN1,0); 
  analogWrite(RIN2,255);
  delay(3000);
  
  // analogWrite(AIN1,255); 
  // analogWrite(AIN2,0);
  // analogWrite(BIN1,0); 
  // analogWrite(BIN2,255);
  // delay(1000);
  
  // analogWrite(AIN1,0); 
  // analogWrite(AIN2,255);
  // analogWrite(BIN1,255); 
  // analogWrite(BIN2,0);
  // delay(1000);
   
}
