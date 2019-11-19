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
#define masterNode        00
#define channel           90
#define lockClose         900
#define lockOpen          1400
#define loopTimeMicroSec  10000

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

      case P_RELEASE:
        lock.writeMicroseconds(lockClose);
        Serial.write(serialByteIn);

      case CAP:
        void capPayload();
        Serial.write(serialByteIn);

      case FLUID:
        void fluidPayload();
        Serial.write(serialByteIn);

      case VIBRATION:
        void vibrationPayload();
        Serial.write(serialByteIn);

      case CAMERA:
        void cameraPayload();
        Serial.write(serialByteIn);
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
  // Send the message to the correct node
  switch (node) {
    case NODE01:
      RF24NetworkHeader header01(NODE01);
      network.write(header01, &radioByteOut, sizeof(radioByteOut));

    case NODE02:
      RF24NetworkHeader header02(NODE02);
      network.write(header02, &radioByteOut, sizeof(radioByteOut));

    case NODE03:
      RF24NetworkHeader header03(NODE03);
      network.write(header03, &radioByteOut, sizeof(radioByteOut));

    case NODE04:
      RF24NetworkHeader header04(NODE04);
      network.write(header04, &radioByteOut, sizeof(radioByteOut));

    case NODE05:
      RF24NetworkHeader header05(NODE05);
      network.write(header04, &radioByteOut, sizeof(radioByteOut));
  }
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
  // Confirm payload is ready
  payloadReady(C_FLOATING);

  while(true){
    // Get info from serial port
    serialByteIn = readSerialPort();

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
