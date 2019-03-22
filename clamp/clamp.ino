#include <Servo.h>

int scale = 185;    // mV/A
int offSet = 2500;  // mV
int currentSensor;
double milliAmps;
Servo myservo;
bool outSwitch;
bool inSwitch;


void setup(){
  // Serial coms
  Serial.begin(9600);

  // Limit Switches
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  // Servo
  myservo.attach(9);
  myservo.writeMicroseconds(1520);
}


void loop(){
  // Read limit switches
  outSwitch = digitalRead(2);
  inSwitch = digitalRead(3);

  // Read current sensor and calculate a value
  currentSensor = analogRead(A0);
  milliAmps = (((currentSensor / 1024.0) * 5000) - offSet) / scale;
  Serial.println(milliAmps*1000,3);

  // Write micro second pulse based on limit switches or a current spike
  if (outSwitch == 0){
    myservo.writeMicroseconds(700);
    delay(1500);
  }

  if (inSwitch == 0){
    myservo.writeMicroseconds(2300);
    delay(1500);
  }

  if (average >= 0.4){
    myservo.writeMicroseconds(1520);
  }
}
