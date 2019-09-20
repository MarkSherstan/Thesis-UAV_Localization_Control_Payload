// Required libraries
#include <Arduino.h>
#include "HX711.h"

// Pin selection
#define DT_PIN  2
#define SCK_PIN 3

// Calibration (0) or measuring (1)
#define mode 1

// Define variables
float calibrationFactor = 460;
unsigned long time;

// Custom functions
void calibration();
void measureTorque();

// Make a class to use
HX711 loadCell;

void setup(){
  // Start serial port
  Serial.begin(57600);

  // Display message to user depending what program is running
  if (mode == 0){
    // Calibration usage notes
    Serial.println("Remove any load from apparatus.");
    Serial.println("---------------------------------------------------");
    Serial.println("Use + to increase calibration factor");
    Serial.println("Use - to decrease calibration factor");
    Serial.println("Use q to exit current calibration factor tuning");
    Serial.println("---------------------------------------------------");
    delay(4000);
  } else if (mode == 1) {
    // Measuring usage notes
    Serial.println("Remove load from apparatus if applicable.");
    Serial.println("All readings are in grams");
    delay(4000);
  } else {
    // Error usage notes
    Serial.println("Mode selection error.");
  }

  // Initialize library (default gain is 128 on channel A)
  loadCell.begin(DT_PIN, SCK_PIN);
}

void loop(){
  if (mode == 0){
    // Run the calibration continously for different calibration masses
    calibration();
  } else if (mode == 1){
    // Log data to serial continously
    measureTorque();
  } else {
    // Dislay error to user
    Serial.println("Mode selection error.");
    delay(1000);
  }
}

void calibration(){
  // Reset scale and then zero readings
  loadCell.set_scale();
  loadCell.tare();

  // Display baseline reading
  Serial.print("Baseline reading: ");
  Serial.println(loadCell.read_average(10));

  // Signal to user to place mass
  Serial.println("Place calibration mass.");
  delay(5000);

  // Run until quit by user
  while(true){
    // Adjust the calibration factor
    loadCell.set_scale(calibrationFactor);

    // Display the average of 3 readings, minus tare weight, divided by scale
    Serial.print(loadCell.get_units(3), 1); Serial.print("\t");

    // Display the current calibration factor
    Serial.println(calibrationFactor);

    // Get input from the user (+, -, q) to adjust calibrationFactor
    if(Serial.available()){
      char temp = Serial.read();

      if (temp == '+')
        calibrationFactor += 5;
      else if (temp == '-')
        calibrationFactor -= 5;
      else if (temp == 'q')
        break;
    }

    // Short delay
    delay(100);
  }

  // Get the user to remove the mass in prep for the next cycle
  Serial.println("\nRemove calibration mass!");
  delay(5000);

  // Post baseline reading
  Serial.print("Baseline reading: ");
  Serial.println(loadCell.read_average(10));

  // Run empty loop forever
  Serial.println("Load next calibration and restart program.");
  while(true);
}

void measureTorque(){
  // Set calibration factor and then zero readings
  Serial.println("Zeroing...");
  delay(2000);
  loadCell.set_scale(calibrationFactor);
  loadCell.tare();
  Serial.println("Wait...");
  delay(3000);
  Serial.println("Start!");

  // Start a timer
  time = millis();

  // Acquire data forever
  while(true){
    Serial.print(millis()); Serial.print(",");
    Serial.println(loadCell.get_units());
  }
}
