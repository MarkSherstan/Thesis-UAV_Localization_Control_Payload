#include "masterPayload.h"
#include <arduino.h>

void MasterPayload::setUpDigitalPins(int ledA, int ledB){
  // Initialize LEDs and turn off
  pinMode(ledA, OUTPUT);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledA, LOW);
  digitalWrite(ledB, LOW);
}

void MasterPayload::startTimeSync(long loopTimeMicroSec){
   // Save sampling rate and start timer
   _loopTimeMicroSec = loopTimeMicroSec;
   micros();
 }

void MasterPayload::LED_ON(int LED){
  digitalWrite(LED, HIGH);
}

void MasterPayload::LED_OFF(int LED){
  digitalWrite(LED, LOW);
}

void MasterPayload::timeSync(){
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
