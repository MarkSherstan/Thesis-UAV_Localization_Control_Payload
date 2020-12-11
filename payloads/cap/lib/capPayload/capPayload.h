// Include gaurd
#ifndef CAPPAYLOAD_H
#define CAPPAYLOAD_H

// Communcation protocal
#define OPEN         0x0A
#define CLOSE        0x0B
#define CLAMPED      0x0C
#define RELEASED     0x0D
#define FLOATING     0x0E

// Radio values
#define MASTER_NODE  00
#define THIS_NODE    01
#define CHANNEL      90

class CapPayload {

private:
  // FSR variables
  int fsrADC;
  float fsrV, fsrR, fsrG;
  float VCC = 4.98;       // volts
  float RES = 10000.0;    // ohms

  // Hall effect current sensor variables
  float hallCurrentVal;
  float scale = 400.0;    // mV/A
  float offSet = 2500.0;  // mV

  // Op amp current sensor variables
  float opAmpCurrentVal;
  float rSense = 0.016;   // ohm
  float gain = 100.0;     // V/V

  // Current
  float hallCurrent, opAmpCurrent, current;

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
  float readCurrent(int hallPin, int opAmpPin, bool print = false);
  float readCurrentHall(int analogPin, int window);
  float readCurrentOpAmp(int analogPin, int window);
  bool readSwitch(int limitSwitch);
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void timeSync();

  // Variables out
  float force;
  float milliAmps;
};

#endif //CAPPAYLOAD_H
