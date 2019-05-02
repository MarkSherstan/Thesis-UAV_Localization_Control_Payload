// Include header files
#include <fstream>
#include <iostream>
#include "serial_port.h"

// Use the following
using namespace std;
using byte = unsigned char;

// Get serial port class
Serial_Port serial_port;

// Declare functions
void readData();

// Declare variables
int northDistance;
uint8_t nKByte = 0;
byte buf[2];


int main(int argc, const char **argv){
  // Set up the serial port
  serial_port.uart_name = "/dev/cu.usbmodem14101";
  serial_port.baudrate = 9600;
  serial_port.start();


  while (true){
    // Get data
    readData();

    // Bitwise shift bytes to int
    northDistance = buf[0] | buf[1] << 8;

    // Print results
    cout << northDistance << " cm" << endl;
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
