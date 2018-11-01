#include <Wire.h>
#include <Servo.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
Servo myservo;

uint32_t currentFrequency;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

void setup(void){
  Serial.begin(115200);

  myservo.attach(3);
  myservo.writeMicroseconds(1520);

  ina219.begin(); //32V, 2A range
  //ina219.setCalibration_32V_1A(); // 32V, 1A range
  //ina219.setCalibration_16V_400mA(); // 16V, 400mA range

  myservo.writeMicroseconds(1000);
}

void loop(void){
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  //Serial.print(busvoltage); Serial.println(" V");
  //Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print(loadvoltage); Serial.print(",");
  Serial.print(current_mA); Serial.print(",");
  Serial.println(power_mW);

  delay(1);
}
