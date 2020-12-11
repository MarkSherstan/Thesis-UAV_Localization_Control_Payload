#include <Arduino.h>
#include <Servo.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "masterPayload.h"

// Pinout Digital
#define LOCK_SERVO_PIN   4
#define RED_LED          9
#define GREEN_LED        10
#define RADIO_CE         7
#define RADIO_CSN        8

// Servo pulses and timer
#define LOCK_CLOSE       900
#define LOCK_OPEN        1400
#define LOOP_TIME_US     20000

// Variables
byte radioByteIn, radioByteOut;
byte serialByteIn, serialByteOut;

// Setup classes
Servo lock;
MasterPayload MP;
RF24 radio(RADIO_CE, RADIO_CSN);
RF24Network network(radio);

// Functions
byte readSerialPort();
void sendRadioMessage(uint16_t node);
void receiveRadioMessage();
void payloadReady();
void COMS(uint16_t node);

// Run once
void setup() {
  // Start the serial port
  Serial.begin(9600);

  // Configure digital pins
  MP.setUpDigitalPins(RED_LED, GREEN_LED);

  // Set up clamping servo and set to release
  lock.attach(LOCK_SERVO_PIN);
  lock.writeMicroseconds(LOCK_OPEN);

  // Set up radio
  SPI.begin();
  radio.begin();
  network.begin(CHANNEL, MASTER_NODE);
  radio.setDataRate(RF24_2MBPS);

  // Start time sync (10000->100Hz, 20000->50Hz)
  MP.startTimeSync(LOOP_TIME_US);
}

// Run forever
void loop() {
  // Read incoming byte from the serial port and do something
  if (Serial.available() > 0) {
    // read the incoming byte
    serialByteIn = Serial.read();

    // Switch statment based on serial input
    switch (serialByteIn) {
      case ENGAGE:
        lock.writeMicroseconds(LOCK_CLOSE);
        payloadReady();
        break;

      case RELEASE:
        lock.writeMicroseconds(LOCK_OPEN);
        break;

      case CAP:
        COMS(NODE01);
        break;

      case FLUID:
        COMS(NODE02);
        break;

      case VIBRATION:
        COMS(NODE03);
        break;

      case CAMERA:
        COMS(NODE04);
        break;

      case FUTURE:
        COMS(NODE05);
        break;

      default:
        receiveRadioMessage();
        serialByteIn = readSerialPort();
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

void payloadReady(){
  // Make sure there is a connection established
  int count;

  while (count < 10){
    // Check for incoming radio messages
    receiveRadioMessage();

    // If floating state
    if (radioByteIn == FLOATING){
      count++;
    }

    // Allow for exit if there is a timeout
    serialByteOut = readSerialPort();
    if (serialByteOut == EXIT){
      break;
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

    // Stabilize sampling rate
    MP.timeSync();
  }
}

void COMS(uint16_t node){
  while(true){
    // Get info from serial port
    serialByteIn = readSerialPort();

    // Send serial message to payload if its good data
    if (serialByteIn != 0x00){
      radioByteOut = serialByteIn;
      sendRadioMessage(node);
    }

    // Check for incoming radio messages
    receiveRadioMessage();

    // Send the radio message to the serial port
    Serial.write(radioByteIn);

    // Exit if command is given
    if (serialByteIn == EXIT){
      break;
    }
  }
}
