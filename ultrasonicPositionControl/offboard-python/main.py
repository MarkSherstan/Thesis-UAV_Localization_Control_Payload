from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
from threading import Thread
import numpy as np
import time
import math
import struct
import copy
import serial

class Controller:
    def __init__(self):
        # Initial conditions (rads)
        self.rollAngle = 0.0
        self.pitchAngle = 0.0
        self.yawAngle = None
        self.yawRate = 0.0
        self.useYawRate = True
        self.thrust = 0.5
        self.duration = 0.1

        # Angle Constraint
        self.minVal = -3.1415/12
        self.maxVal = 3.1415/12

        # Controller PD Gains
        self.kp = 0.2
        self.kd = 5.4

        # Controller
        self.northDesired = None
        self.northPreviousPos = None
        self.eastDesired = None
        self.eastPreviousPos = None
        self.startTime = None

    def sendAttitudeTarget(self, vehicle):
        # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        #
        # useYawRate: the yaw can be controlled using yawAngle OR yawRate.
        #
        # thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
        #         Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
        #         the code for maintaining current altitude.
        #             Thrust >  0.5: Ascend
        #             Thrust == 0.5: Hold the altitude
        #             Thrust <  0.5: Descend

        # Update the yaw angle
        if self.yawAngle is None:
            self.yawAngle = vehicle.attitude.yaw

        # Create the mavlink message
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            0, # Target system
            0, # Target component
            0b00000000 if self.useYawRate else 0b00000100, # If bit is set corresponding input ignored
            self.euler2quaternion(self.rollAngle, self.pitchAngle, self.yawAngle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            self.yawRate, # Body yaw rate in radian/second
            self.thrust # Thrust
        )

        # Send the constructed message
        vehicle.send_mavlink(msg)

    def setAttitude(self, vehicle):
        # Send the command
        self.sendAttitudeTarget(vehicle)

        # SET_ATTITUDE_TARGET has a timeout of 1s for Arducopter 3.3 and higher so keep it running
        start = time.time()
        while time.time() - start < self.duration:
            self.sendAttitudeTarget(vehicle)
            time.sleep(0.025)

        # Print the angle before resetting
        print 'Roll: ', round(math.degrees(vehicle.attitude.roll),3), \
            ' Pitch: ', round(math.degrees(vehicle.attitude.pitch),3), \
            ' Yaw: ', round(math.degrees(vehicle.attitude.yaw),3)

    def euler2quaternion(self, roll, pitch, yaw):
        # Euler angles (rad) to quaternion
        yc = math.cos(yaw * 0.5)
        ys = math.sin(yaw * 0.5)
        rc = math.cos(roll * 0.5)
        rs = math.sin(roll * 0.5)
        pc = math.cos(pitch * 0.5)
        ps = math.sin(pitch * 0.5)

        q0 = yc * rc * pc + ys * rs * ps
        q1 = yc * rs * pc - ys * rc * ps
        q2 = yc * rc * ps + ys * rs * pc
        q3 = ys * rc * pc - yc * rs * ps

        return [q0, q1, q2, q3]

    def constrain(self, val):
        return max(min(self.maxVal, val), self.minVal)

    def PD(self, error, current, previous, deltaT):
        # Run the PD controller
        P = self.kp * error
        D = self.kd * ((current - previous) / deltaT)
        return(P + D)

    def control(self, vehicle):
        # Get current values
        northCurrentPos = vehicle.location.local_frame.north
        eastCurrentPos = vehicle.location.local_frame.east
        deltaT = time.time() - self.startTime

        # Error caculations
        errorNorth = northCurrentPos - self.northDesired
        errorEast = eastCurrentPos - self.eastDesired

        # Update previous position(s) if none
        if self.northPreviousPos is None:
            self.northPreviousPos = northCurrentPos

        if self.eastPreviousPos is None:
            self.eastPreviousPos = eastCurrentPos

        # Run PD control
        pitchControl = self.PD(errorNorth, northCurrentPos, self.northPreviousPos, deltaT)
        rollControl = self.PD(errorEast, eastCurrentPos, self.eastPreviousPos, deltaT)

        # Save the previous position
        self.northPreviousPos = northCurrentPos
        self.eastPreviousPos = eastCurrentPos

        # Execute the controller and print results
        self.pitchAngle = self.constrain(pitchControl)
        self.rollAngle = self.constrain(-rollControl)
        self.setAttitude(vehicle)

    def altitudeTest(self, vehicle, s):
        # Set parameters
        goal = 1
        kp = 0.5
        self.minVal = -2
        self.maxVal = 2

        try:
            while(True):
                # Get actual altitiude from ultrasonic sensor
                actual = s.dataOut[0] * 0.01

                # Calculate error and constrain the value
                error = actual - goal
                control = -self.constrain(kp * error)

                # Set the thrust value between 0 and 1 and send command
                self.thrust = np.interp(control,[-2,-1,0,1,2],[0, 0.25, 0.5, 0.75, 1])
                self.setAttitude(vehicle)

                # Print values to screen
                print 'Actual: ', round(actual,3), 'Error: ', round(error,3), 'Control: ', round(control,3), 'Command: ', round(self.thrust,3)
                print(" ")
        except KeyboardInterrupt:
            s.close()

class DAQ:
    def __init__(self, serialPort, serialBaud, dataNumBytes, numSignals):
        # Class / object / constructor setup
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numSignals = numSignals
        self.rawData = bytearray(numSignals * dataNumBytes)
        self.dataType = 'h'
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.dataOut = []

        # Connect to serial port
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

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
        if (struct.unpack('B', self.serialConnection.read())[0] is 0x9F) and (struct.unpack('B', self.serialConnection.read())[0] is 0x6E):
            self.rawData = self.serialConnection.read(self.numSignals * self.dataNumBytes)

            # Copy raw data to new variable and set up the data out variable
            privateData = copy.deepcopy(self.rawData[:])

            # Loop through all the signals and decode the values
            for i in range(self.numSignals):
                data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
                value, = struct.unpack(self.dataType, data)
                tempData.append(value)

        # Check if data is usable otherwise repeat (recursive function)
        if tempData:
            self.dataOut = tempData
        else:
            return self.getSerialData()

    def close(self):
        # Close the serial port connection
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()

        print('Disconnected from Arduino...')

def main():
    # Connect to the Vehicle
    connection_string = "/dev/ttyS1"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=57600)

    # Connect to serial port Arduino
    portName = '/dev/ttyACM0'
    baudRate = 9600
    dataNumBytes = 2
    numSignals = 1

    s = DAQ(portName, baudRate, dataNumBytes, numSignals)
    s.readSerialStart()

    # Set up controller class
    C = Controller()

    # # Run pitchTest forever
    # while(True):
    #     # print str(s.dataOut[0]), ' [cm]'
    #     C.pitchTest(vehicle)
    #     time.sleep(4)
    C.altitudeTest(vehicle, s)

    s.close()

# Main loop
if __name__ == '__main__':
	main()
