#include "controlAndSense.h"
#include <arduino.h>

void ControlAndSense::setUpDigitalPins(int limitSwitchA, int limitSwitchB, int LED){
  // Limit Switches
  pinMode(limitSwitchA, INPUT_PULLUP);
  pinMode(limitSwitchB, INPUT_PULLUP);

  // Initialize LED and turn off
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void ControlAndSense::startTimeSync(long loopTimeMicroSec){
   // Save sampling rate and start timer
   _loopTimeMicroSec = loopTimeMicroSec;
   micros();
 }

float ControlAndSense::readCurrent(int analogPin){
  // Read analog pin and process value
  currentADC = analogRead(analogPin);
  amps = ((((float)currentADC / 1024.0) * 5000.0) - offSet) / scale;
  milliAmps = amps * 1000;

  // Return the result
  return milliAmps;
}

float ControlAndSense::readFSR(int analogPin){
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

bool ControlAndSense::readSwitch(int limitSwitch){
  return digitalRead(limitSwitch);
}

void ControlAndSense::LED_ON(int LED){
  digitalWrite(LED, HIGH);
}

void ControlAndSense::LED_OFF(int LED){
  digitalWrite(LED, LOW);
}

void ControlAndSense::printData(float force, float current, bool switchStateA, bool switchStateB, int receiverInputChannel){
  Serial.print(micros());                   Serial.print(",");
  Serial.print(force,1);                    Serial.print(",");
  Serial.print(current,1);                  Serial.print(",");
  Serial.print(switchStateA);               Serial.print(",");
  Serial.print(switchStateB);               Serial.print(",");
  Serial.println(receiverInputChannel);
}

void ControlAndSense::timeSync(){
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
