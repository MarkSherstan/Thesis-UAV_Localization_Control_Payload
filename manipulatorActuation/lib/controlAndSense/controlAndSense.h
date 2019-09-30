// Include gaurd
#ifndef CONTROLANDSENSE_H
#define CONTROLANDSENSE_H

// FSR constant values
#define VCC 4.98
#define RES 10000.0

class controlAndSense {
private:
  // FSR variables
  int fsrADC;
  float fsrV, fsrR, fsrG;
public:
  // Constructor
  controlAndSense() = default;

  // Functions
  float readFSR(int analogPin);

  // Variables
  float force;
};

#endif //MPUXX50_H
