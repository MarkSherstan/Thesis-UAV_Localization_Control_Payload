#include <Arduino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#define button    2
#define led       3
#define radioCSN  8
#define radioCE   7
#define channel   90

byte var, radioByteIn;

RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

const uint16_t masterNode = 00;
const uint16_t thisNode = 02;


void receiveRadioMessage();

void setup() {
  Serial.begin(9600);

  SPI.begin();
  radio.begin();
  network.begin(channel, thisNode);
  radio.setDataRate(RF24_2MBPS);
}

void loop() {
  // Update that network
  // network.update();

  // // Receiving
  // while (network.available()){
  //   RF24NetworkHeader header;
  //   unsigned long buttonState;
  //   network.read(header, &buttonState, sizeof(buttonState));
  //   Serial.println(buttonState);
  // }

  receiveRadioMessage();
  Serial.println(radioByteIn, HEX);

  // Sending
  // var = 0x4E;
  // RF24NetworkHeader header(masterNode);
  // network.write(header, &var, sizeof(var));

  delay(300);
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
