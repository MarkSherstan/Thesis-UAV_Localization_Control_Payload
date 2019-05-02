// Include header files
#include <fstream>
#include <iostream>
#include <ctime>
#include "serial_port.h"

// Use the following
using namespace std;
using byte = unsigned char;

// Get class
Serial_Port serial_port;

// Declare functions
void readData();
double controller(int actual, int actualPrevious, int desired, clock_t deltaTime);

// Declare variables
int northDistActual;
int northDistActualPrev = 0;
int northDistDesired = 30;
double northControl;

uint8_t nKByte = 0;
byte buf[2];

// Set timer and its family
clock_t timer = clock();
clock_t deltaTime;


int main(int argc, const char **argv){
  // Set up the serial port
  serial_port.uart_name = "/dev/cu.usbmodem14101";
  serial_port.baudrate = 9600;
  serial_port.start();

  while (true){
    // Timer
    deltaTime = clock() - timer;
    timer = clock();

    // Get data
    readData();

    // Bitwise shift bytes to int
    northDistActual = buf[0] | buf[1] << 8;

    // Position controller
    northControl = controller(northDistActual, northDistActualPrev, northDistDesired, deltaTime);
    northDistActualPrev = northDistActual;

    // Print results
    cout << northDistActual << " cm " << northControl << endl;
  }
}


void readData(){
  // Header check first byte
  serial_port._read_port(nKByte);
  if (nKByte == 0x9F){

    // Header check secound byte
    serial_port._read_port(nKByte);
    if (nKByte == 0x6E){

      // If all checks pass store the useful bytes
      for(int ii = 0; ii < 2; ii++) {
        serial_port._read_port(nKByte);
        buf[ii] = nKByte;
      }
    }
  }
}


int error;
double Kp = 1;
double Ki = 0.01;
double Kd = 4;
double P, I, D;
double controllerOutput;


double controller(int actual, int actualPrevious, int desired, clock_t deltaTime){
  error = actual - desired;

  P = Kp * (double)error;
  I += Ki * (double)error;
  D = Kd * (double)(actual - actualPrevious) / (double)(deltaTime);

  controllerOutput = P + I + D;

  return controllerOutput;
}
