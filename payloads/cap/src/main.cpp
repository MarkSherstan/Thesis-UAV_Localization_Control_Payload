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

// Servo pulses, threshold, and timer
#define clampOpen         2000
#define clampClose        1000
#define clampStop         1520
#define currentThresh     200
#define loopTimeMicroSec  10000

// Serial port flag
bool serialFlag = true;

// Variables
float force, forceDesL, forceDesH, current;
bool switchStateA, switchStateB;
byte dataIn, dataOut;
int servoControl;

// Setup classes
CapPayload CP;
Servo clamp;
RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

// Functions
void sendMessage(byte dataOut);
void receiveMessage();
void boundaryControl();

// Run once
void setup(){
  if (serialFlag == true){
    // Start serial port
    Serial.begin(115200);
  }

  // Configure digital pins
  CP.setUpDigitalPins(limitSwitchA, limitSwitchB, rLED, gLED);

  // Set up clamping servo and set to off
  clamp.attach(clampServo);
  clamp.writeMicroseconds(clampStop);

  // Set up radio and network
  SPI.begin();
  radio.begin();
  network.begin(CHANNEL, THISNODE);
  radio.setDataRate(RF24_2MBPS);

  // Start time sync (10000->100Hz, 5000->200Hz)
  CP.startTimeSync(loopTimeMicroSec);
}

// Run forever
void loop(){
  // Check for incoming message
  receiveMessage();

  // Enage, release, or pass based on incoming message
  if (dataIn == ENGAGE){
    while(true){
      // Close the clamps
      clamp.writeMicroseconds(clampClose);

      // Acquire new data from onboard sensors
      force = CP.readFSR(forceAnalog);
      current = CP.readCurrent(currentAnalog);

      if (current >= currentThresh){
        // Send clamped message
        sendMessage(CLAMPED);

        // Update control variables
        forceDesH = force * 1.05;
        forceDesL = force * 0.95;
        servoControl = clampStop;

        while(true){
          // Beware of limt switches
          boundaryControl();

          // Get fresh force data
          force = CP.readFSR(forceAnalog);

          // Do some control for tightness
          if (force >= forceDesH){
            servoControl -= 2;
          } else if (force <= forceDesL){
            servoControl += 2;
          }

          // Send the servo command
          clamp.writeMicroseconds(servoControl);

          // Can the cap be released?
          receiveMessage();
          if (dataIn == RELEASE){
            break;
          }

          // Stabilize sampling rate
          CP.timeSync();
        }
      }

      // Not yet engaged
      sendMessage(FLOATING);

      // Beware of limit switches
      boundaryControl();

      // Stabilize sampling rate
      CP.timeSync();
    }
  } else if (dataIn == RELEASE){
    while(true){
      // Open the clamp
      clamp.writeMicroseconds(clampOpen);

      // Get fresh force data
      force = CP.readFSR(forceAnalog);

      // Check if released and begin protocal
      if (force < 10){
        // Transmit released state
        sendMessage(RELEASED);

        // Open the jaws the entire way
        while (CP.readSwitch(limitSwitchB) != 0){
          CP.timeSync();
        }

        // Break the loop and wait for next command
        break;
      }

      // Beaware of limit switches
      boundaryControl();

      // Stabilize sampling rate
      CP.timeSync();
    }
  } else{
    // Floating state message
    sendMessage(FLOATING);
  }

  // Stabilize sampling rate
  CP.timeSync();
}

void sendMessage(byte dataOut){
    // Send message
    RF24NetworkHeader header(MASTERNODE);
    network.write(header, &dataOut, sizeof(dataOut));
}

void receiveMessage(){
  // Update the network
  network.update();

  // Receiving data if available
  while (network.available()){
    RF24NetworkHeader header;
    network.read(header, &dataIn, sizeof(dataIn));
  }
}

void boundaryControl(){
  // Get limit switch reading
  switchStateA = CP.readSwitch(limitSwitchA);
  switchStateB = CP.readSwitch(limitSwitchB);

  // Open or close the jaws based on the response
  if (switchStateA == 0){
    clamp.writeMicroseconds(clampOpen);
    CP.LED_ON(rLED);
    delay(1000);
    CP.LED_OFF(rLED);
  } else if (switchStateB == 0){
    clamp.writeMicroseconds(clampClose);
    CP.LED_ON(rLED);
    delay(1000);
    CP.LED_ON(rLED);
  }
}
