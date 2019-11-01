#include <Arduino.h>
#include <Servo.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
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

// Radio and timer
#define channel           90
#define masterNode        00
#define thisNode          01
#define loopTimeMicroSec  10000

// Serial port flag
bool serialFlag = true;

// Variables
float force, current;
bool switchStateA, switchStateB;
unsigned long dataOut;

// Setup classes
CapPayload CP;
Servo clamp;
RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

// Run once
void setup(){
  if (serialFlag == true){
    // Start serial port
    Serial.begin(115200);
  }

  // Configure digital pins
  CP.setUpDigitalPins(limitSwitchA, limitSwitchB, gLED, rLED);

  // Set up clamping servo and set to off
  clamp.attach(clampServo);
  clamp.writeMicroseconds(1520);

  // Set up radio
  SPI.begin();
  radio.begin();
  network.begin(channel, thisNode);
  radio.setDataRate(RF24_2MBPS);

  // Start time sync (10000->100Hz, 5000->200Hz)
  CP.startTimeSync(loopTimeMicroSec);
}

// Run forever
void loop(){
  // Update the network
  network.update();

  // Receiving data
  while (network.available()){
    RF24NetworkHeader header;
    unsigned long dataIn;
    network.read(header, &dataIn, sizeof(dataIn));
  }

  // Acquire new data from onboard sensors
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

  // Print data based on flag state
  if (serialFlag == true){
    CP.printData(force, current, switchStateA, switchStateB);
  }

  // Transmit data back
  RF24NetworkHeader header(masterNode);
  bool ok = network.write(header, &dataOut, sizeof(dataOut));

  // Stabilize sampling rate
  CP.timeSync();
}
