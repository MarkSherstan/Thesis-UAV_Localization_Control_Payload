// Include gaurd
#ifndef CAPPAYLOAD_H
#define CAPPAYLOAD_H

// Required libraries
#include <Servo.h>
#include <arduino.h>

// Define some contstants for communcation protocal
#define ENGAGE     0x0A
#define RELEASE    0x0B
#define CLAMPED    0x0C
#define RELEASED   0x0D
#define BETWEEN    0x0E

// Radio values
#define channel    90
#define masterNode 00
#define thisNode   01

class CapPayload {

private:
  // FSR variables
  int fsrADC;
  float fsrV, fsrR, fsrG;
  float VCC = 4.98;       // volts
  float RES = 10000.0;    // ohms

  // Current sensor variables
  int currentADC;
  float amps;
  float scale = 185.0;    // mV/A
  float offSet = 2500.0;  // mV

  // Time sync
  long _loopTimeMicroSec;
  unsigned long currentTime;
  long timeToDelay;
  unsigned long _trackedTime;

public:
  // Constructor
  CapPayload() = default;

  // Functions
  void setUpDigitalPins(int limitSwitchA, int limitSwitchB, int ledA, int ledB);
  void startTimeSync(long loopTimeMicroSec);
  float readFSR(int analogPin);
  float readCurrent(int analogPin);
  bool readSwitch(int limitSwitch);
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void printData(float force, float current, bool switchStateA, bool switchStateB);
  void timeSync();
  void fnc(Servo *s);

  // Variables out
  float force;
  float milliAmps;
};

#endif //CAPPAYLOAD_H
