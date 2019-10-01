#include <Arduino.h>
#include <Servo.h>
#include "controlAndSense.h"

// Pinout
#define forceAnalog     0
#define currentAnalog   2
#define blueLED         3
#define limitSwitchA    5
#define limitSwitchB    6
#define receiver        8
#define clampServo      9

// Variables
long loopTimeMicroSec = 10000;
float force, current;
byte lastChannel;
int receiverInputChannel;
bool switchStateA, switchStateB;
unsigned long timer;

// Setup classes
controlAndSense CaS;
Servo clamp;

// Run once
void setup(){
  // Start serial port
  Serial.begin(115200);

  // Configure digital pins
  CaS.setUpDigitalPins(limitSwitchA, limitSwitchB, blueLED);

  // Set Atmega for interupt (condifugred for D8)
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  // Set up clamping servo and set to off
  clamp.attach(clampServo);
  clamp.writeMicroseconds(1520);

  // Start time sync (10000->100Hz, 5000->200Hz)
  CaS.startTimeSync(loopTimeMicroSec);
}

// Run forever
void loop(){
  // Acquire new data
  force = CaS.readFSR(forceAnalog);
  current = CaS.readCurrent(currentAnalog);
  switchStateA = CaS.readSwitch(limitSwitchA);
  switchStateB = CaS.readSwitch(limitSwitchB);

  // Print data
  CaS.printData(force, current, switchStateA, switchStateB, receiverInputChannel);

  // Stabilize sampling rate and flicker LED
  CaS.LED_ON(blueLED);
  CaS.timeSync();
  CaS.LED_OFF(blueLED);
}

// Inturpt function for receiver
ISR(PCINT0_vect){
  // Input changed from 0 to 1
  if(lastChannel == 0 && PINB & B00000001){
    lastChannel = 1;
    timer = micros();
  }
  // Input changed from 1 to 0
  else if(lastChannel == 1 && !(PINB & B00000001)){
    lastChannel = 0;
    receiverInputChannel = micros() - timer;
  }
}
