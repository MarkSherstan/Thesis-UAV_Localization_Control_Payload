// Smoothing for current sensor
const unsigned int numReadings = 50;         // Number of readings to be averaged
unsigned int readings[numReadings];          // The readings from the analog input
unsigned int readIndex = 0;                  // The index of the current reading
unsigned int total = 0;                      // The running total
double average = 0;                          // The average
int inputPin = A1;                           // Current sensor analog pin
double minaverage = 1024;                    // For visualizing max and min
double maxaverage = 0;                       // For visualizing max and min
int count = 0;

// Gear and speed information
double sun = 40;                    // Number of teeth on sun gear
double planet = 50;                 // Number of teeth on planet(s) gear(s)
double ring = 140;                  // Number of teeth on ring gear
double bevelGear = 30;              // Number of teeth on bevel gear (to shaft)
double bevelPinion = 15;            // Number of teeth on bevel pinion (from servo)
double servoSpeed = 0.20;           // [s/60 degrees]

double gearRatioPlanet = 1 + (ring / sun);        // Gear ratio if ring gear is held stationary
double speedRatioPlanet = 1 / gearRatioPlanet;    // Speed ratio of carrier
double gearRatioBevel = bevelGear / bevelPinion;  // Gear ratio of worm wheel pair
double speedRatioBevel = 1 / gearRatioBevel;      // Speed ratio of worm wheel pair
double servoRPM = (1 / (servoSpeed * 6)) * 60;    // RPM's of servo
double outputRPM = servoRPM * speedRatioPlanet * speedRatioBevel;


//Other
bool unscrew = true;
double numberOfTwists = 1;
#include <Servo.h>
Servo myservo;


void setup() {
  Serial.begin(9600);

  // Servo
  myservo.attach(9);
  myservo.writeMicroseconds(1520); // Turn servo off

  // Initialize all the readings
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = analogRead(inputPin);
    total += readings[thisReading];
  }
}


void loop() {
  if (unscrew == true) {
    myservo.writeMicroseconds(2200);                    // Unscrew... Tighty righty, loosey lefty
    delay((numberOfTwists / (outputRPM / 60)) * 1000);  // Let the cap twist off
    myservo.writeMicroseconds(1520);                    // Turn servo off

    // Get fluid sample... Next system

    unscrew = false;
    myservo.writeMicroseconds(700);                     // Screw on... Tighty righty, loosey lefty
    delay(2000);
  }

  // Weighted Average
  total = total - readings[readIndex];          // Subtract the last reading
  readings[readIndex] = analogRead(inputPin);   // Read Sensor
  total = total + readings[readIndex];          // Add reading to total
  readIndex = readIndex + 1;                    // Advance to the next position in the array

  if (readIndex >= numReadings) {               // If at the end of the array...wrap around to beginning
    readIndex = 0;
  }

  average = total / (float) numReadings;        // Calculate the average
  //valuePlotter();                             // Find optimal value for if statment below
 
  if (average >= 529) {                         // If current spikes (the cap is on tight) turn off servo
    myservo.writeMicroseconds(1520);
  }

  delay(1);                                     // Delay for stability

}


void valuePlotter(){
  if (average > maxaverage)
    maxaverage = average;
  else if ( average < minaverage)
    minaverage = average;

  Serial.print(maxaverage);
  Serial.print(",");
  Serial.print(minaverage);
  Serial.print(",");
  Serial.println(average);
}
