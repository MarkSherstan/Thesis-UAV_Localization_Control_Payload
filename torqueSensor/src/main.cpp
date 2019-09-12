// Required libraries
#include <Arduino.h>
#include "HX711.h"

// Pin selection
#define DT_PIN  2
#define SCK_PIN 3

// Calibration (0) or measuring (1)
#define mode 0

// Define variables
float calibrationFactor = 0;

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
    delay(2000);
  } else if (mode == 1) {
    // Measuring usage notes
    Serial.println("Remove any load from apparatus.");
    Serial.println("All readings are in Newton meters [Nm]");
    delay(4000);
  } else {
    // Error usage notes
    Serial.println("mode selection error");
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
    Serial.println("mode selection error");
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

  // Signal to user to place mass (they have 5s)
  Serial.println("Place calibration mass...");

  for (int ii = 0; ii <= 10; ii++) {
    Serial.print(".");
    delay(500);
  }

  // Run until broken by user
  while(true){
    // Adjust the calibration factor
    loadCell.set_scale(calibrationFactor);

    // Display the average of 3 readings, minus tare weight, divided by scale
	  Serial.print("Reading: ");
    Serial.print(loadCell.get_units(3), 1);
    Serial.println(" Nm");

    // Display the current calibration factor
    Serial.print("Calibration factor: ");
    Serial.println(calibrationFactor);

    // Get input from the user (+, -, q) to adjust calibrationFactor
    if(Serial.available()){
      char temp = Serial.read();

      if (temp == '+')
        calibrationFactor += 10;
      else if (temp == '-')
        calibrationFactor -= 10;
      else if (temp == 'q')
        break;
    }
  }

  // Get the user to remove the mass in prep for the next cycle
  Serial.println("\nRemove calibration mass!");
  delay(3000);

  // Post baseline reading
  Serial.print("Baseline reading: ");
  Serial.println(loadCell.read_average(10));
}

void measureTorque(){
  // Set calibration factor and then zero readings
  loadCell.set_scale(calibrationFactor);
  loadCell.tare();

  // Acquire data forever
  while(true){
    Serial.println(loadCell.get_units(), 1);
    delay(100);
  }
}
