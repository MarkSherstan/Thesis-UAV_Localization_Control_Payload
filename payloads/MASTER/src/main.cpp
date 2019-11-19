#include <Arduino.h>
#include <Servo.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "masterPayload.h"

// Pinout Digital
#define gLED            4
#define rLED            5
#define radioCE         7
#define radioCSN        8
#define lockServo       9

// Radio, timer, and servo pulses
#define channel           90
#define lockClose         900
#define lockOpen          1400
#define loopTimeMicroSec  10000

const uint16_t masterNode = 00;
const uint16_t NODE01 = 01;       // Cap
const uint16_t NODE02 = 02;       // Fluid
const uint16_t NODE03 = 03;       // Vibration
const uint16_t NODE04 = 04;       // Camera
const uint16_t NODE05 = 05;       // Future port

// Variables
byte radioByteIn, radioByteOut;
byte serialByteIn, serialByteOut;

// Setup classes
MasterPayload MP;
Servo lock;
RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

// Functions
byte readSerialPort();
void payloadReady(byte floating);
void sendRadioMessage(uint16_t node);
void receiveRadioMessage();
void capPayload();
void fluidPayload();
void vibrationPayload();
void cameraPayload();

// Run once
void setup() {
  // Start the serial port
  Serial.begin(9600);

  // Configure digital pins
  MP.setUpDigitalPins(rLED, gLED);

  // Set up clamping servo and set to release
  lock.attach(lockServo);
  lock.writeMicroseconds(lockOpen);

  // Set up radio
  SPI.begin();
  radio.begin();
  network.begin(channel, masterNode);
  radio.setDataRate(RF24_2MBPS);

  // Start time sync (10000->100Hz, 5000->200Hz)
  MP.startTimeSync(loopTimeMicroSec);
}

// Run forever
void loop() {
  // Read incoming byte from the serial port and do something
  if (Serial.available() > 0) {
    // read the incoming byte
    serialByteIn = Serial.read();

    // Switch statment based on serial input
    switch (serialByteIn) {
      case P_ENGAGE:
        lock.writeMicroseconds(lockOpen);
        Serial.write(serialByteIn);
        break;

      case P_RELEASE:
        lock.writeMicroseconds(lockClose);
        Serial.write(serialByteIn);
        break;

      case CAP:
        Serial.write(serialByteIn);

        // Confirm payload is ready
        payloadReady(C_FLOATING);

        capPayload();
        break;

      case FLUID:
        Serial.write(serialByteIn);
        void fluidPayload();
        break;

      case VIBRATION:
        Serial.write(serialByteIn);
        void vibrationPayload();
        break;

      case CAMERA:
        Serial.write(serialByteIn);
        void cameraPayload();
        break;
    }
  }

  // Stabilize sampling rate
  MP.timeSync();
}

void receiveRadioMessage(){
  // Update the network
  network.update();

  // Receiving data if available
  while (network.available()){
    RF24NetworkHeader header;
    network.read(header, &radioByteIn, sizeof(radioByteIn));
  }
}

void sendRadioMessage(uint16_t node){
  RF24NetworkHeader header(node);
  network.write(header, &radioByteOut, sizeof(radioByteOut));
}

byte readSerialPort(){
  if (Serial.available() > 0) {
    serialByteIn = Serial.read();
    return serialByteIn;
  }
    return 0x00;
}

void payloadReady(byte floating){
  // Make sure there is a connection established
  int count;

  while (count < 10){
    // Check for incoming radio messages
    receiveRadioMessage();

    // If floating state
    if (radioByteIn == floating){
      count++;
    }

    // Update serial port and stabilize sampling rate
    Serial.write(WAIT);
    MP.timeSync();
  }

  // Wait for a command
  while(true){
    // Tell serial port we are ready
    Serial.write(READY);

    // Break when ready
    serialByteOut = readSerialPort();

    if (serialByteOut == READY){
      break;
    }
  }
}

void capPayload(){
  while(true){
    // Get info from serial port
    serialByteIn = readSerialPort();
    Serial.write(serialByteIn);

    // Send to payload if its good data
    if (serialByteIn != 0x00){
      radioByteOut = serialByteIn;
      sendRadioMessage(NODE01);
    }

    // Check for incoming radio messages
    receiveRadioMessage();

    // Send the radio message to the serial port
    Serial.write(radioByteIn);
  }
}

void fluidPayload(){}

void vibrationPayload(){}

void cameraPayload(){}
