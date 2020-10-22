#include <Arduino.h>
#include <Servo.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "capPayload.h"

// Pinout Analog
#define forceAnalog         0
#define currentHallAnalog   6
#define currentOpAmpAnalog  2

// Pinout Digital
#define limitSwitchA    2
#define limitSwitchB    3
#define clampServo      4
#define radioCE         7
#define radioCSN        8
#define rLED            9
#define gLED            10

// Servo pulses, threshold, and timer
#define clampOpen         2000
#define clampClose        1000
#define clampStop         1520
#define forceThresh       300
#define loopTimeMicroSec  10000

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
  // Serial port for debug
  // Serial.begin(9600);

  // Configure digital pins
  CP.setUpDigitalPins(limitSwitchA, limitSwitchB, rLED, gLED);

  // Set up clamping servo and set to off
  clamp.attach(clampServo);
  clamp.writeMicroseconds(clampStop);

  // Set up radio and network
  SPI.begin();
  radio.begin();
  network.begin(CHANNEL, THIS_NODE);
  radio.setDataRate(RF24_2MBPS);

  // Start time sync (10000->100Hz, 5000->200Hz)
  CP.startTimeSync(loopTimeMicroSec);
}

// Run forever
void loop(){
  // Check for incoming message
  receiveMessage();

  // Use incoming byte to determine state
  switch(dataIn) {
    case CLOSE:
      while(true){
        // Close the clamps
        clamp.writeMicroseconds(clampClose);

        // Not yet closed
        sendMessage(FLOATING);

        // Check if there is an updated command
        receiveMessage();
        if (dataIn == OPEN){
          break;
        }

        // Beware of limit switches
        boundaryControl();

        // Acquire new data from onboard sensors
        force = CP.readFSR(forceAnalog);
        current = CP.readCurrent(currentHallAnalog, currentOpAmpAnalog);

        // If threshold is met keep clamped
        if (force >= forceThresh){
          // Send clamped message
          sendMessage(CLAMPED);

          // Update control variables
          forceDesH = force * 1.4;
          forceDesL = force * 1;
          servoControl = clampStop;

          while(true){
            // Beware of limit switches
            boundaryControl();

            // Get fresh force data
            force = CP.readFSR(forceAnalog);

            // Do some control for clamping force
            if (force >= forceDesH){
              servoControl += 1;
            } else if (force <= forceDesL){
              servoControl -= 1;
            }

            servoControl = constrain(servoControl, clampClose, clampOpen);

            // Send the servo command
            clamp.writeMicroseconds(servoControl);

            // Update message
            sendMessage(CLAMPED);

            // Can the cap be released?
            receiveMessage();
            if (dataIn == OPEN){
              break;
            }

            // Stabilize sampling rate
            CP.timeSync();
          }
        }

        // Stabilize sampling rate
        CP.timeSync();
      }

      // Break from switch case
      break;

    case OPEN:
      while(CP.readSwitch(limitSwitchB) != 0){
        // Open the clamp
        clamp.writeMicroseconds(clampOpen);

        // Check if there is an updated command
        receiveMessage();
        if (dataIn == CLOSE){
          break;
        }

        // Get fresh force data
        force = CP.readFSR(forceAnalog);

        // Check if released and send corresponding message
        if (force < 50){
          sendMessage(RELEASED);
        } else {
          sendMessage(FLOATING);
        }

        // Stabilize sampling rate
        CP.timeSync();
      }

      // Stop the clamp and break
      clamp.write(clampStop);
      break;

    default:
      boundaryControl();
      clamp.write(clampStop);
      sendMessage(FLOATING);
      break;
  }

  // Stabilize sampling rate
  CP.timeSync();
}

void sendMessage(byte dataOut){
    // Send message
    RF24NetworkHeader header(MASTER_NODE);
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
    delay(500);
    CP.LED_OFF(rLED);
  } else if (switchStateB == 0){
    clamp.writeMicroseconds(clampClose);
    CP.LED_ON(rLED);
    delay(500);
    CP.LED_OFF(rLED);
  }
}
