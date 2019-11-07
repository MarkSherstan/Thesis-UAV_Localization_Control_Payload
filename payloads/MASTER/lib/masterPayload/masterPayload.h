// Include gaurd
#ifndef MASTERPAYLOAD_H
#define MASTERPAYLOAD_H

class MasterPayload {

private:
  // Serial port
  unsigned char headerA = 0x9F;
  unsigned char headerB = 0x6E;
  unsigned char engaged, payload, msg;


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
  void payloadEngaged(int limitSwitchA, int limitSwitchB);
  void sendSerialMsg();
  void LED_ON(int LED);
  void LED_OFF(int LED);
  void printData();
  void timeSync();

  // Variables
  bool _engagedState;
};

#endif //MASTERPAYLOAD_H
