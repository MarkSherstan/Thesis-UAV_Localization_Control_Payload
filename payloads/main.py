from threading import Thread
import time
import struct
import serial

class COMS:
    def __init__(self, serialPort, serialBaud, numBytes):
        # Class / object / constructor setup
        self.port = serialPort
        self.baud = serialBaud
        self.numBytes = numBytes
        self.isReceiving = False
        self.isRun = True
        self.thread = None
        self.dataOut = []

        # Connect to serial port
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print('Failed to connect with ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def readSerialStart(self):
        # Create a thread
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()

            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def backgroundThread(self):
        # Pause and clear buffer to start with a good connection
        time.sleep(2)
        self.serialConnection.reset_input_buffer()

        # Read until closed
        while (self.isRun):
            self.getSerialData()
            self.isReceiving = True

    def getSerialData(self):
        # Initialize data out
        tempData = []

        # Check for header bytes and then read bytearray if header satisfied
        if (struct.unpack('B', self.serialConnection.read())[0] is 0x9F) and
                (struct.unpack('B', self.serialConnection.read())[0] is 0x6E):

            rawData = self.serialConnection.read(self.numBytes)

        # Check if data is usable otherwise repeat (recursive function)
        if tempData:
            self.dataOut = rawData
        else:
            return self.getSerialData()

    def close(self):
        # Close the serial port connection
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected from ' + str(self.port))

def main():
    # Connect to serial port
    portName = '/dev/cu.usbmodem14101'
    baudRate = 9600
    numBytes = 5

    # Set up the class and start the serial port
    s = COMS(portName, baudRate, numBytes)
    s.readSerialStart()

    # Print data for 15 seconds
    startTime = time.time()
    while (time.time() < startTime + 15):
        print(s.dataOut)

    # Close the serial port
    s.close()

# Main loop
if __name__ == '__main__':
	main()
