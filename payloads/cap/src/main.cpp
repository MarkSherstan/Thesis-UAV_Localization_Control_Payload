#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "capPayload.h"

// Pinout Analog
#define forceAnalog     0
#define currentAnalog   2

// Pinout Digital
#define limitSwitchA    2
#define limitSwitchB    3
#define clampServo      4
#define radioCE         7
#define radioCSN        8
#define gLED            9
#define rLED            10

// Variables
long loopTimeMicroSec = 10000;
float force, current;
bool switchStateA, switchStateB;

// Functions
void radioSetup(const byte address[6]);

// Setup classes
CapPayload CP;
Servo clamp;
RF24 radio(radioCE, radioCSN);

// Run once
void setup(){
  // Start serial port
  Serial.begin(115200);

  // Configure digital pins
  CP.setUpDigitalPins(limitSwitchA, limitSwitchB, gLED, rLED);

  // Set up clamping servo and set to off
  clamp.attach(clampServo);
  clamp.writeMicroseconds(1520);

  // Set up radio on specified address
  radioSetup("00001");

  // Start time sync (10000->100Hz, 5000->200Hz)
  CP.startTimeSync(loopTimeMicroSec);
}

// Run forever
void loop(){
  // Acquire new data
  force = CP.readFSR(forceAnalog);
  current = CP.readCurrent(currentAnalog);
  switchStateA = CP.readSwitch(limitSwitchA);
  switchStateB = CP.readSwitch(limitSwitchB);

  // Actuate
  if (switchStateA == 0){
    clamp.writeMicroseconds(1000);
    CP.LED_ON(gLED);
    CP.LED_OFF(rLED);
  } else if (switchStateB == 0){
    clamp.writeMicroseconds(2000);
    CP.LED_ON(rLED);
    CP.LED_OFF(gLED);
  }

  // Print data
  CP.printData(force, current, switchStateA, switchStateB);

  // Stabilize sampling rate
  CP.timeSync();
}

// Radio Setup
void radioSetup(const byte address[6]){
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}
