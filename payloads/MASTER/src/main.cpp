#include <Arduino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>
#include "masterPayload.h"

// Pinout Digital
#define limitSwitchA    2
#define limitSwitchB    3
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

// Payload nodes
const uint16_t masterNode = 00;
const uint16_t node01 = 01;       // Cap
const uint16_t node02 = 02;       // Fluid
const uint16_t node03 = 03;       // Vibration
const uint16_t node04 = 04;       // Camera
const uint16_t node05 = 05;       // Future port

// Variables
unsigned long analog;
char incomingByte;

// Setup classes
MasterPayload MP;
Servo lock;
RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

// Run Once
void setup() {
  // Start the serial port
  Serial.begin(9600);

  // Configure digital pins
  MP.setUpDigitalPins(limitSwitchA, limitSwitchB, gLED, rLED);

  // Set up clamping servo and set to release
  lock.attach(lockServo);
  lock.writeMicroseconds(lockOpen);
  MP._engagedState = false;

  // Set up radio
  SPI.begin();
  radio.begin();
  network.begin(channel, masterNode);
  radio.setDataRate(RF24_2MBPS);

  // Start time sync (10000->100Hz, 5000->200Hz)
  MP.startTimeSync(loopTimeMicroSec);

  // MP.LED_ON(gLED); MP.LED_ON(rLED);
  // delay(2000);
  // MP.LED_OFF(gLED); MP.LED_OFF(rLED);
}


void loop() {
  // Update that network
  network.update();

  // Receiving data
  while (network.available()){
    RF24NetworkHeader header;
    unsigned long incomingData;
    network.read(header, &incomingData, sizeof(incomingData));
    digitalWrite(rLED, !incomingData);
    Serial.println(incomingData);
  }

  // Read incoming bytes from the serial port
    if (Serial.available() > 0) {
      // read the incoming byte:
      incomingByte = Serial.read();

      if (incomingByte == 0x0A) {
        // Open command
        lock.writeMicroseconds(lockOpen);
        MP.LED_ON(gLED);
        MP.LED_OFF(rLED);
      } else if (incomingByte == 0x0C) {
        // Open command
        lock.writeMicroseconds(lockClose);
        MP.LED_ON(rLED);
        MP.LED_OFF(gLED);
      }
    }mike

  MP.payloadEngaged(limitSwitchA, limitSwitchB);

  // Transmitting
  analog = analogRead(A0);

  RF24NetworkHeader header01(node01);
  bool ok1 = network.write(header01, &analog, sizeof(analog));

  RF24NetworkHeader header02(node02);
  bool ok2 = network.write(header02, &analog, sizeof(analog));

  RF24NetworkHeader header03(node03);
  bool ok3 = network.write(header03, &analog, sizeof(analog));

  RF24NetworkHeader header04(node04);
  bool ok4 = network.write(header04, &analog, sizeof(analog));


  MP.timeSync();
}
