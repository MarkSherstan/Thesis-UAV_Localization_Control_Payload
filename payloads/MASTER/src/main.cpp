#include <Arduino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>

#define button    2
#define led       3
#define radioCSN  8
#define radioCE   7

// byte lastChannel;
// int receiverInputChannel;
// unsigned long timer;
unsigned long analog;

RF24 radio(radioCE, radioCSN);
RF24Network network(radio);

const uint16_t masterNode = 00;
const uint16_t node01 = 01;       // Cap
const uint16_t node02 = 02;       // Fluid
const uint16_t node03 = 03;       // Vibration
const uint16_t node04 = 04;       // Camera
const uint16_t node05 = 05;       // Future port


void setup() {
  Serial.begin(9600);

  SPI.begin();
  radio.begin();
  network.begin(90, masterNode);
  radio.setDataRate(RF24_2MBPS);

  // // Set Atmega for interupt (condifugred for D8)
  // PCICR |= (1 << PCIE0);
  // PCMSK0 |= (1 << PCINT0);
}


void loop() {
  // Update that network
  network.update();

  // Receiving
  while (network.available()){
    RF24NetworkHeader header;
    unsigned long incomingData;
    network.read(header, &incomingData, sizeof(incomingData));
    digitalWrite(led, !incomingData);
    Serial.println(incomingData);
  }

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


  delay(500);
}


// // Inturpt function for receiver
// ISR(PCINT0_vect){
//   // Input changed from 0 to 1
//   if(lastChannel == 0 && PINB & B00000001){
//     lastChannel = 1;
//     timer = micros();
//   }
//   // Input changed from 1 to 0
//   else if(lastChannel == 1 && !(PINB & B00000001)){
//     lastChannel = 0;
//     receiverInputChannel = micros() - timer;
//   }
// }
