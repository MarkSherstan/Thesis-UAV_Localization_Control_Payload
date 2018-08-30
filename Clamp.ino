// Smoothing for current sensor
const int numReadings = 30;     // Number of readings to be averaged 
int readings[numReadings];      // The readings from the analog input
int readIndex = 0;              // The index of the current reading
int total = 0;                  // The running total
int average = 0;                // The average
int inputPin = A0;              // Current sensor analog pin 

#include <Servo.h> 
Servo myservo;


void setup() {
  Serial.begin(9600);

  // Limit Switches
  pinMode(2, INPUT_PULLUP);     // Little Switch, pullup to stop floating 
  pinMode(3, INPUT_PULLUP);     // Big Switch, pullup to stop floating 

  // Servo
  myservo.attach(8);
  myservo.writeMicroseconds(1520);  

  // Initialize all the readings to 0 for weighted average
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}


void loop() {
  bool littleSwitch = digitalRead(2);
  bool bigSwitch = digitalRead(3);

  if (littleSwitch == 0) {                      // Are jaws fully closed?
    myservo.writeMicroseconds(700);             // Open jaws
    delay(1500);                                // Set delay to not trip current sensor on startup 
  }

  if (bigSwitch == 0) {                         // Are jaws fully open?
    myservo.writeMicroseconds(2300);            // Close jaws
    delay(1500);                                // Set delay to not trip current sensor on startup  
  }
       
  // Weighted Average 
  total = total - readings[readIndex];          // Subtract the last reading
  readings[readIndex] = analogRead(inputPin);   // Read Sensor
  total = total + readings[readIndex];          // Add reading to total
  readIndex = readIndex + 1;                    // Advance to the next position in the array

  if (readIndex >= numReadings) {               // If at the end of the array...wrap around to beginning
    readIndex = 0;
  }

  average = total / numReadings;                // Calculate the average
  Serial.println(average);                      // Print average 

  if (average >= 483) {                         // If current spikes (jaws clamped) turn off servo  
    myservo.writeMicroseconds(1520);
  }

  delay(1);                                     // Delay for stability
}

