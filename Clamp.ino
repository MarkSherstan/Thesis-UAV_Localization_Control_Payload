/* 
Used for clamping mechansim to control range of motion and to grip lid based off of current spike

Limit Switches - Pin 2 and 3
Servo (HS-645MG Hitec Ultra Torque) - Pin 9
LED's - Pins 12 and 13
Current Sensor (ACS714 with 220uF capacitor) - Pin A0

500-1500 --> Open Jaws - Green
1520 --> Stationary 
1500-2500 --> Closes Jaws - No LED

Weighted average is based on David A. Mellis / Tom Igoe smoothing tutorial...
http://www.arduino.cc/en/Tutorial/Smoothing
*/


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

  //LED's
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  // Limit Switches
  pinMode(2, INPUT_PULLUP);     // Little Switch, pullup to stop floating 
  pinMode(3, INPUT_PULLUP);     // Big Switch, pullup to stop floating 

  // Servo
  myservo.attach(9);
  myservo.writeMicroseconds(1520);  

  // Initialize all the readings to 0 for weighted average
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}


void loop() {
  bool littleSwitch = digitalRead(2);
  bool bigSwitch = digitalRead(3);

  if (littleSwitch == 0) {            // Are jaws fully closed?
    digitalWrite(12,HIGH);            // Turn on signal LED
    myservo.writeMicroseconds(1000);  // Open jaws medium speed
    delay(1000);                      // Set delay to not trip current sensor on startup 
  } else {
    digitalWrite(12,LOW);
  }

  if (bigSwitch == 0) {               // Are jaws fully open?
    digitalWrite(13,HIGH);            // Turn on signal LED
    myservo.writeMicroseconds(2000);  // Close jaws medium speed 
    delay(1000);                      // Set delay to not trip current sensor on startup  
  } else {
    digitalWrite(13,LOW);
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

  if (average >= 520) {                         // If current spikes (jaws clamped) turn off servo  
    myservo.writeMicroseconds(1520);
  }

  delay(1);                                     // Delay for stability
}

