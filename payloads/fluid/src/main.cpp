#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define led 12

RF24 radio(7, 8); // CE, CSN

const byte addresses[][6] = {"00001", "00002"};
boolean buttonState = 0;

void setup() {
  pinMode(12, OUTPUT);
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  delay(5);
  radio.stopListening();

  int potValue = analogRead(A0);
  int angleValue = map(potValue, 0, 1023, 0, 180);

  radio.write(&angleValue, sizeof(angleValue));
  delay(5);
  radio.startListening();

  while (!radio.available());
  radio.read(&buttonState, sizeof(buttonState));
  if (buttonState == HIGH) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}
