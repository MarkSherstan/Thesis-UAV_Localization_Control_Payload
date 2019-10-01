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
unsigned long timer, currentTime, trackedTime;
long timeToDelay;

// Functions
void timeSync();

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

  // Start time sync timer
  micros();
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
  timeSync();
  CaS.LED_OFF(blueLED);
}

// Time stabilization
void timeSync(){
  // Calculate required delay
  currentTime = micros();
  timeToDelay = loopTimeMicroSec - (currentTime - trackedTime);

  // Execute the delay
  if (timeToDelay > 5000){
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  } else if (timeToDelay > 0){
    delayMicroseconds(timeToDelay);
  } else {}

  // Update the tracked time
  trackedTime = currentTime + timeToDelay;
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
