// Include gaurd
#ifndef CONTROLANDSENSE_H
#define CONTROLANDSENSE_H

class ControlAndSense {

private:
  // FSR variables
  // int dummyVar;
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
  ControlAndSense() = default;

  // Functions
  void setUpDigitalPins(int limitSwitchA, int limitSwitchB, int LED);
  void startTimeSync(long loopTimeMicroSec);
  float readFSR(int analogPin);
  float readCurrent(int analogPin);
  bool readSwitch(int limitSwitch);
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void printData(float force, float current, bool switchStateA, bool switchStateB, int receiverInputChannel);
  void timeSync();

  // Variables out
  float force;
  float milliAmps;
};

#endif //MPUXX50_H
