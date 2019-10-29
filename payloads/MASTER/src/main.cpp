#include <Arduino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>

#define button    2
#define led       3
#define radioCE   10
#define radioCSN  9

RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

const uint16_t masterNode = 00;
const uint16_t node01 = 01;       // Cap
const uint16_t node02 = 02;       // Fluid
const uint16_t node03 = 03;       // Vibration
const uint16_t node04 = 04;       // Camera
const uint16_t node05 = 05;       // Future port
const uint16_t node06 = 06;       // Future port


void setup() {
  SPI.begin();
  radio.begin();
  network.begin(90, masterNode);
  radio.setDataRate(RF24_2MBPS);

  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
}


void loop() {
  // Update that network
  network.update();

  // Receiving
  while (network.available()){
    RF24NetworkHeader header;
    unsigned long incomingData;
    network.read(header, &incomingData, sizeof(incomingData));
    Serial.println(incomingData);
  }

  // Transmitting
  unsigned long buttonState = digitalRead(button);

  RF24NetworkHeader header01(node01);
  bool ok1 = network.write(header01, &buttonState, sizeof(buttonState));

  RF24NetworkHeader header02(node02);
  bool ok2 = network.write(header02, &buttonState, sizeof(buttonState));

  RF24NetworkHeader header03(node03);
  bool ok3 = network.write(header03, &buttonState, sizeof(buttonState));

  RF24NetworkHeader header04(node04);
  bool ok4 = network.write(header04, &buttonState, sizeof(buttonState));
}
