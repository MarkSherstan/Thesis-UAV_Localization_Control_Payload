// Include gaurd
#ifndef MASTERPAYLOAD_H
#define MASTERPAYLOAD_H

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
  
  void setUpDigitalPins(int limitSwitchA, int limitSwitchB, int ledA, int ledB);
  void startTimeSync(long loopTimeMicroSec);
  bool readSwitch(int limitSwitch);
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void printData();
  void timeSync();
};

#endif //MASTERPAYLOAD_H
