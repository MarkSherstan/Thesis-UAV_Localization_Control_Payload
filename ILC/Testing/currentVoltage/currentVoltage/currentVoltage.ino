// Measuring Current Using ACS712 - 5A model and custom voltage divider
// http://forums.trossenrobotics.com/tutorials/how-to-diy-128/cheap-battery-monitor-using-resistive-voltage-divider-3264/

#include <Servo.h>
Servo myservo;


int scale = 185;  // mV/A
int offSet = 2500; // mV
int currentSensor;
int voltageSensor;
double amps;


void setup(){
 Serial.begin(115200);

 myservo.attach(3);
 myservo.writeMicroseconds(1000);
}


void loop(){
 currentSensor = analogRead(A0);
 currentSensor = analogRead(A1);

 amps = (((currentSensor / 1024.0) * 5000) - offSet) / scale;
 volts = 0; // Add voltage divider calculations here...

 Serial.print(amps*1000,3); Serial.print(",");
 Serial.println(volts);

 delay(1);
}
