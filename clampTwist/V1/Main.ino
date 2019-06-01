// Smoothing for current sensor
const unsigned int numReadings = 50;              // Number of readings to be averaged
unsigned int readings[numReadings];               // The readings from the analog input
unsigned int readIndex = 0;                       // The index of the current reading
unsigned int total = 0;                           // The running total
double average = 0;                               // The average
int inputPinA0 = A0;                              // Current sensor analog pin
int inputPinA1 = A1;                              // Current sensor analog pin
double minaverage = 1024;                         // For visualizing max and min
double maxaverage = 0;                            // For visualizing max and min
int count = 0;                                    // Counter

// Gear and speed information
double sun = 40;                                  // Number of teeth on sun gear
double planet = 50;                               // Number of teeth on planet(s) gear(s)
double ring = 140;                                // Number of teeth on ring gear
double bevelGear = 30;                            // Number of teeth on bevel gear (to shaft)
double bevelPinion = 15;                          // Number of teeth on bevel pinion (from servo)
double servoSpeed = 0.20;                         // [s/60 degrees]

double gearRatioPlanet = 1 + (ring / sun);        // Gear ratio if ring gear is held stationary
double speedRatioPlanet = 1 / gearRatioPlanet;    // Speed ratio of carrier
double gearRatioBevel = bevelGear / bevelPinion;  // Gear ratio of bevel gears
double speedRatioBevel = 1 / gearRatioBevel;      // Speed ratio of bevel gears
double servoRPM = (1 / (servoSpeed * 6)) * 60;    // RPM's of servo
double outputRPM = servoRPM * speedRatioPlanet * speedRatioBevel;

double numberOfTwists = 1.25;

// Boolean
bool clampRun = false;
bool twistOnRun = false;

// Servo
#include <Servo.h>
Servo clampServo;                 // Pin 8
Servo twistServo;                 // Pin 9


void setup() {
  Serial.begin(9600);

  // Limit Switches,
  pinMode(2, INPUT_PULLUP);              // Little Switch, pullup to stop floating
  pinMode(3, INPUT_PULLUP);              // Big Switch, pullup to stop floating

  // Servos
  clampServo.attach(8);
  twistServo.attach(9);

  clampServo.writeMicroseconds(1520);      // Turn servo off
  twistServo.writeMicroseconds(1520);      // Turn servo off

  fullOpen();                              // Open jaws all the way

  // Function for raising/lowering mechanism

  clamp();

  twistOff();

  delay(5000);

  // Sample fluid

  twistOn();

  //fullOpen(); Commented for testing purposes
}


void loop() {
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void fullOpen() {
  bool bigSwitch = digitalRead(3);

  while (bigSwitch != 0) {
    clampServo.writeMicroseconds(700);      // Open Jaws
    bigSwitch = digitalRead(3);
  }

  clampServo.writeMicroseconds(1520);      // Turn servo off
  Serial.println("Clamps are open");
}

//////////////////////////////////////////////////////////////////////////////////////////////

void clamp() {
  // Initialize all the readings for clamp servo current draw
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = analogRead(inputPinA0);
    total += readings[thisReading];
  }

  while (clampRun != true) {
    bool littleSwitch = digitalRead(2);
    bool bigSwitch = digitalRead(3);

    if (littleSwitch == 0) {                      // Are jaws fully closed?
      clampServo.writeMicroseconds(700);          // Open jaws
      delay(1500);                                // Set delay to not trip current sensor on startup
    }

    if (bigSwitch == 0) {                         // Are jaws fully open?
      clampServo.writeMicroseconds(2300);         // Close jaws
      delay(1500);                                // Set delay to not trip current sensor on startup
    }

    // Weighted Average
    total = total - readings[readIndex];          // Subtract the last reading
    readings[readIndex] = analogRead(inputPinA0); // Read Sensor
    total = total + readings[readIndex];          // Add reading to total
    readIndex = readIndex + 1;                    // Advance to the next position in the array

    if (readIndex >= numReadings) {               // If at the end of the array...wrap around to beginning
      readIndex = 0;
    }

    average = total / (float) numReadings;        // Calculate the average
    Serial.print("Clamp current: ");
    Serial.println(average);

    if (average >= 515) {                         // If current spikes (jaws clamped) turn off servo
      clampServo.writeMicroseconds(1520);
      break;
    }

    delay(1);                                     // Delay for stability
  }
  Serial.println("Clamps are secure");
}

//////////////////////////////////////////////////////////////////////////////////////////////

void twistOff() {
  twistServo.writeMicroseconds(2200);                 // Unscrew... Tighty righty, loosey lefty
  delay((numberOfTwists / (outputRPM / 60)) * 1000);  // Let the cap twist off
  twistServo.writeMicroseconds(1520);                 // Turn servo off
  Serial.println("Cap is unthreaded");
}

//////////////////////////////////////////////////////////////////////////////////////////////

void twistOn() {
  // Reset all the readings for twist servo current draw
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = analogRead(inputPinA1);
    total += readings[thisReading];
  }

  while (twistOnRun != true) {
    twistServo.writeMicroseconds(700);            // Screw... Tighty righty, loosey lefty

    // Weighted Average
    total = total - readings[readIndex];          // Subtract the last reading
    readings[readIndex] = analogRead(inputPinA1); // Read Sensor
    total = total + readings[readIndex];          // Add reading to total
    readIndex = readIndex + 1;                    // Advance to the next position in the array

    if (readIndex >= numReadings) {               // If at the end of the array...wrap around to beginning
      readIndex = 0;
    }

    average = total / (float) numReadings;        // Calculate the average
    Serial.print("Twist current: ");
    Serial.println(average);

    if (average >= 1034) {                         // If current spikes (the cap is on tight) turn off servo - Value from twist ValuePlotter
      twistServo.writeMicroseconds(1520);
      break;
    }

    delay(1);                                     // Delay for stability
  }
  Serial.println("Cap is secure");
}
