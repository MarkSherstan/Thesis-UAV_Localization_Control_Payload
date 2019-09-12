// Required libraries
#include <Arduino.h>
#include "HX711.h"

// Pin selection
const int DT_PIN = 2;
const int SCK_PIN = 3;

// Define variables
float calibrationFactor = 0;

// Custom functions
void calibration();

// Make a class to use
HX711 loadCell;


void setup(){
  // Start serial port
  Serial.begin(57600);

  // Display message to user
  Serial.println("Remove all weight from scale.");

  // Initialize library (default gain is 128 on channel A)
  loadCell.begin(DT_PIN, SCK_PIN);
}

void loop(){
  // Run the calibration continously for different calibration masses
  calibration();
}


void calibration(){
  // Set scale and zero readings
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
}
