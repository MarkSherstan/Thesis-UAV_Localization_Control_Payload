// Include gaurd
#ifndef MASTERPAYLOAD_H
#define MASTERPAYLOAD_H

// Payload protocal
#define ENGAGE       0xFE
#define RELEASE      0xFF

// Payload states
#define FLOATING     0x0E
#define READY        0x5A
#define WAIT         0x5F
#define EXIT         0xEE

// Payload ids
#define CAP          0x01
#define FLUID        0x02
#define VIBRATION    0x03
#define CAMERA       0x04
#define FUTURE       0x05

// Radio parameters
#define MASTER_NODE  00
#define CHANNEL      90

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
