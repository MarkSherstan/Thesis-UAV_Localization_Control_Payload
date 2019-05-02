#include <fstream>
#include <iostream>
#include "serial_port.h"

using namespace std;
using byte = unsigned char;

int northDistance;
uint8_t nKByte = 0;
byte buf[2];

Serial_Port serial_port;


int main(int argc, const char **argv){
  serial_port.uart_name = "/dev/cu.usbmodem14101";
  serial_port.baudrate = 9600;
  serial_port.start();

  while (true){

    // Header check
    serial_port._read_port(nKByte);

    if (nKByte == 0x9F){
      serial_port._read_port(nKByte);

      if (nKByte == 0x6E){

        for( int ii = 0; ii < 2; ii++) {
          serial_port._read_port(nKByte);
          buf[ii] = nKByte;
        }
      }
    }

    northDistance = buf[0] | buf[1] << 8;

    cout << northDistance << " cm" << endl;

  }
}
