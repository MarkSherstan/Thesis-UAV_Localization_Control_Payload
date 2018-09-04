# UAV-arduino-sketches

Arduino code for UAV oil sampler. The system is broken into four parts:

* Clamp - Complete - Used for clamping lid, controling range of motion, and stopping motion based off of current spike (clamp secured).
* Twist - In progress - Used for applying a torque to twist on/off lid once the jaws are secured.
* Raise/Lower - Not complete
* Sample - Not complete

Files are seperated for individual systems for testing and devlopment. "Main.ino" brings all systems together.

## Hardware and Pin Allocation
Using an Arduino Uno the following hardware is used on the following pins:
* Clamp limit switches - Pin 2 and 3
* Clamp Servo (HS-645MG Hitec Ultra Torque) - Pin 8
* Clamp Current Sensor (ACS714 with 220uF capacitor) - Pin A0
* Twist Servo (HS-645MG Hitec Ultra Torque) - Pin 9
* Twist Current Sensor (ACS714 with 220uF capacitor) - Pin A1

Pin 0 and 1 reserved for TX/RX with Odroid. 

## Additional Information 
PWM values for modified HS-645MG Hitec Ultra Torque Servo:
* 500-1500 --> Open Jaws and Tighten Cap
* 1520 --> Stationary 
* 1500-2500 --> Closes Jaws and Loosen Cap

## Wiring Diagram
Under development, using [Fritzing](http://fritzing.org/home/).
