from threading import Thread
import time
import struct
import serial

class COMS:
    def __init__(self, serialPort, serialBaud):
        # Class / object / constructor setup
        self.port = serialPort
        self.baud = serialBaud
        self.isReceiving = False
        self.isRun = True
        self.thread = None
        self.byteOut = 0

        # Connect to serial port
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=0)
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
        try:
            temp = self.serialConnection.read()
            self.byteOut = struct.unpack('B', temp)[0]
        except:
            pass

    def writeSerialData(self, msg):
        try:
            self.serialConnection.write(bytes([msg]))
        except:
            print('fail')

    def close(self):
        # Close the serial port connection
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected from ' + str(self.port))

class QuickConnect:
    def __init__(self):
        # Payload protocal
        self.ENGAGE   = 0xFE
        self.RELEASE  = 0xFF

        # Payload states
        self.FLOATING = 0x0E
        self.READY    = 0x5A
        self.WAIT     = 0x5F
        self.EXIT     = 0xEE

    def engage(self, s):
        # Send the command to engage
        s.writeSerialData(self.ENGAGE)

        # Wait till ready command is received
        while(s.byteOut != self.READY):
            pass

        # Acknowledge ready
        s.writeSerialData(self.READY)

    def release(self, s):
        # Send the command to engage
        s.writeSerialData(self.RELEASE)

def main():
    # Connect to serial port
    portName = '/dev/cu.wchusbserial1410'
    baudRate = 9600

    # Set up the class and start the serial port
    s = COMS(portName, baudRate)
    s.readSerialStart()

    # Classes
    q = QuickConnect()

    # Run till quit
    while(True):
        state = input('What would you like to do? ')

        # Switch case options
        if state is 'q':
            break
        elif state is 'e':
            q.engage(s)
        elif state is 'r':
            q.release(s)

    # Close the serial port
    s.close()

# Main loop
if __name__ == '__main__':
	main()
