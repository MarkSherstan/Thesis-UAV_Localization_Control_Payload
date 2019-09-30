#include "controlAndSense.h"
#include <arduino.h>

float controlAndSense::readFSR(int analogPin){
  // Read analog pin
  fsrADC = analogRead(analogPin);

  // Use ADC reading to calculate voltage
  fsrV = (float)fsrADC * VCC / 1023.0;

  // Use voltage and static resistor value to calculate FSR resistance
  fsrR = ((VCC - fsrV) * RES) / fsrV;

  // Guesstimate force based on slopes in figure 3 of FSR datasheet (conductance)
  fsrG = 1.0 / fsrR;

  // Break parabolic curve down into two linear slopes
  if (fsrR <= 600)
    force = (fsrG - 0.00075) / 0.00000032639;
  else
    force =  fsrG / 0.000000642857;

  // Return the result
  return force;
}
