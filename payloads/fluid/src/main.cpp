#include <Arduino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#define button    2
#define led       3
#define radioCSN  9
#define radioCE   10
#define channel   90

RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

const uint16_t masterNode = 00;
const uint16_t thisNode = 02;

void setup() {
  SPI.begin();
  radio.begin();
  network.begin(channel, thisNode);
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
    unsigned long buttonState;
    network.read(header, &buttonState, sizeof(buttonState));
    digitalWrite(led, !buttonState);
  }

  // Sending
  unsigned long buttonState = digitalRead(button);
  RF24NetworkHeader header(masterNode);
  bool ok = network.write(header, &buttonState, sizeof(buttonState));
}
