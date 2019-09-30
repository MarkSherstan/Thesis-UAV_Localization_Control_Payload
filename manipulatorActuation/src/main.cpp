#include <Arduino.h>
#include <Servo.h>
#include "controlAndSense.h"

// Pinout
#define forceAnalog  2
#define clampDigital 9

// Variables
float force;

// Setup classes
controlAndSense *CaS;
Servo clamp;

// Run once
void setup(){
  // Start serial port
  Serial.begin(115200);

  // Attach clamping servo
  clamp.attach(clampDigital);
  clamp.writeMicroseconds(1520);
}

// Run forever
void loop(){
  force = CaS->readFSR(forceAnalog);
}
