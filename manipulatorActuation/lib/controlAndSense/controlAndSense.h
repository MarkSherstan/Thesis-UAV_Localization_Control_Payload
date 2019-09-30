// Include gaurd
#ifndef CONTROLANDSENSE_H
#define CONTROLANDSENSE_H

class controlAndSense {

private:
  // FSR variables
  int fsrADC;
  float fsrV, fsrR, fsrG;
  float VCC = 4.98;     // volts
  float RES = 10000.0;  // ohms

  // Current sensor variables
  int currentADC;
  int scale = 185;      // mV/A
  int offSet = 2500;    // mV

  // Time sync
  long _loopTimeMicroSec;
  unsigned long currentTime;
  long timeToDelay;
  unsigned long _trackedTime;

public:
  // Constructor
  controlAndSense() = default;

  // Functions
  void setUpDigitalPins(int limitSwitchA, int limitSwitchB, int LED);
  void startTimeSync(long loopTimeMicroSec);
  float readFSR(int analogPin);
  float readCurrent(int analogPin);
  bool readSwitch(int limitSwitch);
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void printData(float force, float current, int receiverInputChannel);
  void timeSync();

  // Variables
  float force;
  float milliAmps;
};

#endif //MPUXX50_H
