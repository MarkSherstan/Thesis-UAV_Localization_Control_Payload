// Include gaurd
#ifndef MASTERPAYLOAD_H
#define MASTERPAYLOAD_H

// Payloads protocal
#define P_ENGAGE     0xFF
#define P_RELEASE    0xFE
#define CAP          0x01
#define FLUID        0x02
#define VIBRATION    0x03
#define CAMERA       0x04

// Cap communication protocal
#define C_ENGAGE     0x0A
#define C_RELEASE    0x0B
#define C_CLAMPED    0x0C
#define C_RELEASED   0x0D
#define C_FLOATING   0x0E

// Node assignment
#define NODE01       01       // Cap
#define NODE02       02       // Fluid
#define NODE03       03       // Vibration
#define NODE04       04       // Camera
#define NODE05       05       // Future port


class MasterPayload {

private:
  // Time sync
  long _loopTimeMicroSec;
  unsigned long currentTime;
  long timeToDelay;
  unsigned long _trackedTime;

public:
  // Constructor
  MasterPayload() = default;

  // Functions
  void setUpDigitalPins(int ledA, int ledB);
  void startTimeSync(long loopTimeMicroSec);
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void timeSync();
};

#endif //MASTERPAYLOAD_H
