#include "capPayload.h"
#include <arduino.h>

void CapPayload::setUpDigitalPins(int limitSwitchA, int limitSwitchB, int ledA, int ledB){
  // Limit Switches
  pinMode(limitSwitchA, INPUT_PULLUP);
  pinMode(limitSwitchB, INPUT_PULLUP);

  // Initialize LEDs and turn off
  pinMode(ledA, OUTPUT);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledA, LOW);
  digitalWrite(ledB, LOW);
}

void CapPayload::startTimeSync(long loopTimeMicroSec){
   // Save sampling rate and start timer
   _loopTimeMicroSec = loopTimeMicroSec;
   micros();
 }

float CapPayload::readCurrentHall(int analogPin){
  // Read analog pin and process value
  hallADC = analogRead(analogPin);
  hallCurrentRaw = (((((float)hallADC / 1023.0) * 5000.0) - offSet) / scale) * 1000.0;

  // Return the result
  return hallCurrentRaw;
}

float CapPayload::readCurrentOpAmp(int analogPin){
  // Read analog pin and process value
  opAmpADC = analogRead(analogPin);
  opAmpCurrentRaw = (((float)opAmpADC * (5.0 / 1023.0)) / (rSense * gain)) * 1000.0;

  // Return the result
  return opAmpCurrentRaw;
}

float CapPayload::readCurrent(int hallPin, int opAmpPin){
  // Get the two current readings and average
  hallCurrent = readCurrentHall(hallPin);
  opAmpCurrent = readCurrentOpAmp(opAmpPin);
  current = (hallCurrent + opAmpCurrent) / 2.0;

  // Return the average result
  return current;
}

float CapPayload::readFSR(int analogPin){
  // Read analog pin
  fsrADC = analogRead(analogPin);

  // Use ADC reading to calculate voltage
  fsrV = (float)fsrADC * VCC / 1023.0;

  // Use voltage and static resistor value to calculate FSR resistance
  fsrR = ((VCC - fsrV) * RES) / fsrV;

  // Guesstimate force based on slopes in figure 3 of FSR datasheet (conductance)
  fsrG = 1.0 / fsrR;

  // Break parabolic curve down into two linear slopes
  if (fsrR <= 600)
    force = (fsrG - 0.00075) / 0.00000032639;
  else
    force =  fsrG / 0.000000642857;

  // Return the result
  return force;
}

bool CapPayload::readSwitch(int limitSwitch){
  return digitalRead(limitSwitch);
}

void CapPayload::LED_ON(int LED){
  digitalWrite(LED, HIGH);
}

void CapPayload::LED_OFF(int LED){
  digitalWrite(LED, LOW);
}

void CapPayload::timeSync(){
   // Calculate required delay
   currentTime = micros();
   timeToDelay = _loopTimeMicroSec - (currentTime - _trackedTime);

   // Execute the delay
   if (timeToDelay > 5000){
     delay(timeToDelay / 1000);
     delayMicroseconds(timeToDelay % 1000);
   } else if (timeToDelay > 0){
     delayMicroseconds(timeToDelay);
   } else {}

   // Update the tracked time
   _trackedTime = currentTime + timeToDelay;
 }
