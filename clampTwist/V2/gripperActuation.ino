#include <Servo.h>

//Declaring Variables
int scale = 185;    // mV/A
int offSet = 2500;  // mV
int currentSensor;
double milliAmps;
byte lastChannel;
int receiverInputChannel;
unsigned long timer;
Servo myservo;


void setup(){
  // Serial coms
  Serial.begin(9600);

  // Set Atmega pin default to input. Scan and set PCINT0 (D53) interrupt.
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  // Servo
  myservo.attach(9);
  myservo.writeMicroseconds(1520);

  // Initialize LED and turn off
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
}


void loop(){
  // Read current sensor and calculate a value
  currentSensor = analogRead(A8);
  milliAmps = (((currentSensor / 1024.0) * 5000) - offSet) / scale;
  Serial.print(milliAmps);

  // If current high flash LED
  if (milliAmps >= 0.2){
    for (int i = 0; i <= 15; i++) {
      digitalWrite(10, HIGH);
      delay(100);
      digitalWrite(10, LOW);
      delay(100);
    }
  }

  // Reading RC signal and sending signal to servo
  if (receiverInputChannel - 1480 < 0){
    myservo.writeMicroseconds(1000);
  } else if (receiverInputChannel - 1520 > 0){
    myservo.writeMicroseconds(2000);
  } else {
    myservo.writeMicroseconds(1520);
  }

  Serial.print("\t"); Serial.println(receiverInputChannel);
}


ISR(PCINT0_vect){
  // Input changed from 0 to 1
  if(lastChannel == 0 && PINB & B00000001 ){
    lastChannel = 1;
    timer = micros();
  }
  // Input changed from 1 to 0
  else if(lastChannel == 1 && !(PINB & B00000001)){
    lastChannel = 0;
    receiverInputChannel = micros() - timer;
  }
}
