// Measuring Current Using ACS712 - 5A model

#include <Servo.h>
Servo myservo;


int scale = 185;  // mV/A
int offSet = 2500; // mV
int rawValue;
double voltage;
double amps;


void setup(){
 Serial.begin(9600);

 myservo.attach(3);
 myservo.writeMicroseconds(1000);
}


void loop(){
 rawValue = analogRead(A0);
 amps = (((RawValue / 1024.0) * 5000) - offSet) / scale;

 Serial.println(amps,3);
 delay(10);
}
