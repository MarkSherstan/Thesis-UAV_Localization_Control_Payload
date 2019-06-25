from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib
import datetime
import time
import math
import random

class Simulate:
    def __init__(self):
        self.vehicleArmed = True
        self.vehicleMode = "GUIDED" #GUIDED_NOGPS wont display position for simulating

    def armAndTakeOff(self, C, vehicle, targetAltitude):
        # Give the user some info
        print("Basic pre-arm checks")
        print("--------------------------------------------------")

        # Wait till vehicle is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialize...")
            time.sleep(1)

        print("Arming motors")

        # Set vehicle mode
        vehicle.mode = VehicleMode(self.vehicleMode)
        vehicle.armed = self.vehicleArmed

        # Arm the vehicle
        while not vehicle.armed:
            print(" Waiting for arming...")
            vehicle.armed = self.vehicleArmed
            time.sleep(1)

        # Take off
        print("Taking off!")
        print("--------------------------------------------------")

        # Send take off command
        vehicle.simple_takeoff(targetAltitude)

        # Wait until actual altitude is achieved
        while abs(vehicle.location.local_frame.down) <= targetAltitude:
            print '  ', round(vehicle.location.local_frame.down,3)
            time.sleep(0.2)

        # Small pause and display message
        time.sleep(0.5)
        print '\nAltitude achieved\n'

        # double check the yaw is aligned
        while not ((358 <= vehicle.heading <= 360) or (0 <= vehicle.heading <= 2)):
            C.sendAttitudeTarget(vehicle)
            print '  ', vehicle.heading
            time.sleep(0.1)

        # Small pause and display message
        time.sleep(0.5)
        print '\nYaw achieved\n'

class Controller:
    def __init__(self):
        # Initial conditions (rads)
        self.rollAngle = 0.0
        self.pitchAngle = 0.0
        self.yawAngle = 0.0
        self.yawRate = 0.0
        self.thrust = 0.5
        self.duration = 0.08

        # Constraints for roll, pitch, and thrust
        self.minValNE = -3.1415/8
        self.maxValNE = 3.1415/8
        self.minValD = -1
        self.maxValD = 1

        # PID Controller Gains
        self.kp = 0.3
        self.ki = 0
        self.kd = 0.2

        # PID variables
        self.northPreviousError = 0
        self.eastPreviousError = 0
        self.northI = 0
        self.eastI = 0

        # Controller
        self.northDesired = None
        self.eastDesired = None
        self.downDesired = None
        self.startTime = None

        # Thrust Control
        self.kThrottle = 0.5
        self.one2one = [-1,0,1]
        self.zero2one = [0, 0.5, 1]

        # Save values for plotting
        self.northDesiredList = []
        self.northActualList = []
        self.eastDesiredList = []
        self.eastActualList = []
        self.downDesiredList = []
        self.downActualList = []
        self.rollList = []
        self.pitchList = []
        self.timeList = []

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
		#
		# Mappings: If any of these bits are set, the corresponding input should be ignored.
		# bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate.
		# bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

		# Create the mavlink message
		msg = vehicle.message_factory.set_attitude_target_encode(
			0, # time_boot_ms
			0, # Target system
			0, # Target component
			0b00000100, # If bit is set corresponding input ignored (mappings)
			self.euler2quaternion(self.rollAngle, self.pitchAngle, self.yawAngle), # Quaternion
			0, # Body roll rate in radian
			0, # Body pitch rate in radian
			self.yawRate, # Body yaw rate in radian/second
			self.thrust # Thrust
		)

		# Send the constructed message
		vehicle.send_mavlink(msg)

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

    def constrain(self, val, minVal, maxVal):
        return max(min(maxVal, val), minVal)

    def PID(self, error, errorPrev, I, dt):
        # Run the PD controller
        P = self.kp * error
        I = self.ki * (I + error * dt)
        D = self.kd * ((error - errorPrev) / dt)
        return (P + I + D), I

    def northEastTest(self, vehicle):
        # Run for 10 seconds
        while (time.time() < self.startTime + 10):

            # Get current values
            northCurrentPos = vehicle.location.local_frame.north
            eastCurrentPos = vehicle.location.local_frame.east
            deltaT = time.time() - self.startTime

            # Save values for plotting
            self.northDesiredList.append(self.northDesired)
            self.northActualList.append(northCurrentPos)
            self.eastDesiredList.append(self.eastDesired)
            self.eastActualList.append(eastCurrentPos)
            self.timeList.append(deltaT)

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
            self.pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
            self.rollAngle = self.constrain(-rollControl, self.minValNE, self.maxValNE)
            self.setAttitude(vehicle)

            print 'N: ', round(vehicle.location.local_frame.north,3), \
                '\tE: ', round(vehicle.location.local_frame.east,3), \
                '\tD: ', round(vehicle.location.local_frame.down,3)
            print 'N Error: ', round(errorNorth,3), 'E Error: ', round(errorEast,3)
            print 'Input N: ', round(math.degrees(self.constrain(pitchControl, self.minValNE, self.maxValNE)),3), \
                'Input E: ', round(math.degrees(self.constrain(rollControl, self.minValNE, self.maxValNE)),3), '[deg]'
            print(" ")

        # Make subplot and plot
        fig, ax = plt.subplots()
        ax.plot(self.timeList, self.northActualList, self.timeList, self.northDesiredList,
            self.timeList, self.eastActualList, self.timeList, self.eastDesiredList)

        # Set labels, titles, info, legend and grid
        ax.set(xlabel='Time (s)',
                ylabel='position (m)',
                title='Position Control\n' +
                ' Kp: ' + str(self.kp) +
                ' Kd: ' + str(self.kd) +
                ' Command Rate: ' + str(self.duration) + ' s')

        plt.gca().legend(('North Actual','North Desired', 'East Actual', 'East Desired'))
        ax.grid()

        # Show results and save the plot
        fig.savefig("positionController.png")
        plt.show()

    def altitudeTest(self, vehicle):
        # Run for 10 seconds
        while (time.time() < self.startTime + 10):

            # Get actual value and add to list
            actual = vehicle.location.local_frame.down
            self.timeList.append(time.time() - self.startTime)
            self.downActualList.append(actual)
            self.downDesiredList.append(self.downDesired)

            # Calculate error and constrain the value
            error = actual - self.downDesired
            control = self.constrain(error * self.kThrottle, self.minValD, self.maxValD)

            # Set the thrust value between 0 and 1 and send command
            self.thrust = np.interp(control, self.one2one, self.zero2one)
            self.setAttitude(vehicle)

            # Print values to screen
            print 'Actual: ', round(actual,3), 'Error: ', round(error,3), 'Control: ', round(control,3), 'Command: ', round(self.thrust,3)
            print(" ")

        # Plot the results
        fig, ax = plt.subplots()
        ax.plot(self.timeList, self.downActualList, self.timeList, self.downDesiredList)

        # Set labels, titles, and grid
        ax.set(xlabel='Time (s)', ylabel='Altitude (m)',
            title='Altitude Control\n' + ' kT: ' + str(self.kThrottle) +
            ' Command Rate: ' + str(self.duration) + ' s')
        plt.gca().legend(('Actual Down','Desired Down'))
        ax.grid()

        # Show results and save the plot
        fig.savefig("altitudeController.png")
        plt.show()

    def fullTest(self, vehicle):
        # Run for 8 seconds. Old values before trajectory:
        # self.kp = 0.2       self.kd = 5.4       self.kThrottle = 0.5
        while (time.time() < self.startTime + 8):

            # Get current values
            northCurrentPos = vehicle.location.local_frame.north
            eastCurrentPos = vehicle.location.local_frame.east
            downCurrentPos = vehicle.location.local_frame.down
            deltaT = time.time() - self.startTime

            # Save values for plotting
            self.northDesiredList.append(self.northDesired)
            self.northActualList.append(northCurrentPos)
            self.eastDesiredList.append(self.eastDesired)
            self.eastActualList.append(eastCurrentPos)
            self.downDesiredList.append(self.downDesired)
            self.downActualList.append(downCurrentPos)
            self.timeList.append(deltaT)

            # Error caculations
            errorNorth = northCurrentPos - self.northDesired
            errorEast = eastCurrentPos - self.eastDesired
            errorDown = downCurrentPos - self.downDesired

            # Update previous position(s) if none
            if self.northPreviousPos is None:
                self.northPreviousPos = northCurrentPos

            if self.eastPreviousPos is None:
                self.eastPreviousPos = eastCurrentPos

            # Run some control
            pitchControl = self.PD(errorNorth, northCurrentPos, self.northPreviousPos, deltaT)
            rollControl = self.PD(errorEast, eastCurrentPos, self.eastPreviousPos, deltaT)
            thrustControl = self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

            # Save the previous position
            self.northPreviousPos = northCurrentPos
            self.eastPreviousPos = eastCurrentPos

            # Execute the controller
            self.pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
            self.rollAngle = self.constrain(-rollControl, self.minValNE, self.maxValNE)
            self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)
            self.sendAttitudeTarget(vehicle)
            time.sleep(self.duration)

        # Plot the results
        fig, ax = plt.subplots()
        ax.plot(self.timeList, self.northActualList, self.timeList, self.northDesiredList,
            self.timeList, self.eastActualList, self.timeList, self.eastDesiredList,
            self.timeList, self.downActualList, self.timeList, self.downDesiredList)

        # Set labels and titles
        fig.suptitle('NED Attitude Control', fontsize=14, fontweight='bold')
        ax.set_title('$K_p:$ ' + str(self.kp) + '\t$K_d:$ ' + str(self.kd) +
            '\t$k_T:$ ' + str(self.kThrottle) + '\t$Cmd Rate:$ ' + str(self.duration) + '$s$')
        ax.set_xlabel('Time (s)', fontweight='bold')
        ax.set_ylabel('Position (m)', fontweight='bold')

        # Set ylim, legend, and grid
        bottom, top = ax.get_ylim()
        ax.set_ylim(bottom=bottom-1)
        plt.gca().legend(('North Actual','North Desired',
            'East Actual', 'East Desired',
            'Down Actual', 'Down Desired'), ncol=3, loc='lower center')
        ax.grid()

        # Show the plot and save
        fig.savefig('masterController.png')
        plt.show()

    def commandTest(self, vehicle):
        # Run for 10 seconds
        while (time.time() < self.startTime + 13):
            # 15 Degrees
            self.rollAngle = math.radians(15)
            for ii in range(5):
                self.sendAttitudeTarget(vehicle)
                self.northActualList.append(vehicle.location.local_frame.north)
                self.eastActualList.append(vehicle.location.local_frame.east)
                self.rollList.append(math.degrees(vehicle.attitude.roll))
                self.pitchList.append(math.degrees(vehicle.attitude.pitch))
                self.timeList.append(time.time() - self.startTime)
                time.sleep(0.2)

            # 0 Degrees
            self.rollAngle = 0
            for ii in range(5):
                self.sendAttitudeTarget(vehicle)
                self.northActualList.append(vehicle.location.local_frame.north)
                self.eastActualList.append(vehicle.location.local_frame.east)
                self.rollList.append(math.degrees(vehicle.attitude.roll))
                self.pitchList.append(math.degrees(vehicle.attitude.pitch))
                self.timeList.append(time.time() - self.startTime)
                time.sleep(0.2)

            # -15 Degrees
            self.rollAngle = math.radians(-15)
            for ii in range(5):
                self.sendAttitudeTarget(vehicle)
                self.northActualList.append(vehicle.location.local_frame.north)
                self.eastActualList.append(vehicle.location.local_frame.east)
                self.rollList.append(math.degrees(vehicle.attitude.roll))
                self.pitchList.append(math.degrees(vehicle.attitude.pitch))
                self.timeList.append(time.time() - self.startTime)
                time.sleep(0.2)

            # 0 Degrees
            self.rollAngle = 0
            for ii in range(5):
                self.sendAttitudeTarget(vehicle)
                self.northActualList.append(vehicle.location.local_frame.north)
                self.eastActualList.append(vehicle.location.local_frame.east)
                self.rollList.append(math.degrees(vehicle.attitude.roll))
                self.pitchList.append(math.degrees(vehicle.attitude.pitch))
                self.timeList.append(time.time() - self.startTime)
                time.sleep(0.2)

        # Plot
        ax1 = plt.subplot(211)
        plt.title('Attitude Command Test')
        plt.plot(self.timeList, self.northActualList, self.timeList, self.eastActualList)
        plt.setp(ax1.get_xticklabels(), fontsize=6)
        plt.gca().legend(('North','East'))
        plt.ylabel('Position [m]')

        ax2 = plt.subplot(212, sharex=ax1)
        plt.plot(self.timeList, self.rollList, self.timeList, self.pitchList)
        plt.setp(ax2.get_xticklabels(), visible=False)
        plt.gca().legend(('Roll','Pitch'))
        plt.ylabel('Angle [deg]')
        plt.xlabel('time (s)')

        plt.show()

    def trajectoryControl(self, vehicle):
        # Initialize variables
        counter = 0
        T = 3

        # Get the current postition
        northCurrentPos = vehicle.location.local_frame.north
        eastCurrentPos = vehicle.location.local_frame.east

        # Set the desired NED locations
        self.northDesired = vehicle.location.local_frame.north + 0.5
        self.eastDesired = vehicle.location.local_frame.east - 0.5
        self.downDesired = -0.4

        # Generate a trajectory that should take T seconds
        northIC = [northCurrentPos, self.northDesired, 0, 0, 0, 0]
        eastIC = [eastCurrentPos, self.eastDesired, 0, 0, 0, 0]

        pN = self.trajectoryGen(northIC, T, self.duration, False)
        pE = self.trajectoryGen(eastIC, T, self.duration, False)

        # Start timers
        self.startTime = time.time()
        prevTime = time.time()

        # Run for 3*T seconds
        while (time.time() < self.startTime + 3*T):
            # Get current values
            northCurrentPos = vehicle.location.local_frame.north
            eastCurrentPos = vehicle.location.local_frame.east
            downCurrentPos = vehicle.location.local_frame.down #+ (random.random()-0.5)*0.1
            timeStamp = time.time() - self.startTime

            # Set the desired position based on time counter index
            if counter < len(pN):
                desiredN = pN[counter]
                desiredE = pE[counter]
            else:
                desiredN = self.northDesired
                desiredE = self.eastDesired

            # Save values for plotting
            self.northDesiredList.append(desiredN)
            self.northActualList.append(northCurrentPos)
            self.eastDesiredList.append(desiredE)
            self.eastActualList.append(eastCurrentPos)
            self.downDesiredList.append(self.downDesired)
            self.downActualList.append(downCurrentPos)
            self.timeList.append(timeStamp)

            # Error caculations
            errorNorth = desiredN - northCurrentPos
            errorEast = desiredE - eastCurrentPos
            errorDown = self.downDesired - downCurrentPos

            # Time elapsed
            dt = time.time() - prevTime
            prevTime = time.time()

            # Run some control
            pitchControl, self.northI = self.PID(errorNorth, self.northPreviousError, self.northI, dt)
            rollControl, self.eastI = self.PID(errorEast, self.eastPreviousError, self.eastI, dt)
            thrustControl = -self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

            # Update previous errror
            self.northPreviousError = errorNorth
            self.eastPreviousError = errorEast

            # Set the controller values
            self.pitchAngle = -self.constrain(pitchControl, self.minValNE, self.maxValNE)
            self.rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
            self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)

            # Send the command, sleep, and increase counter
            self.sendAttitudeTarget(vehicle)
            time.sleep(self.duration)
            counter += 1

        # Send the land command before plotting to decrease downtime
        vehicle.mode = VehicleMode("LAND")

        # Plot the results
        fig, ax = plt.subplots()
        ax.plot(self.timeList, self.northActualList, self.timeList, self.northDesiredList,
            self.timeList, self.eastActualList, self.timeList, self.eastDesiredList,
            self.timeList, self.downActualList, self.timeList, self.downDesiredList)

        # Set labels and titles
        fig.suptitle('NED Attitude Control w/ Trajectory Generation', fontsize=14, fontweight='bold')
        ax.set_title('$K_p:$ ' + str(self.kp) + '\t$K_i:$ ' + str(self.ki) +
            '\t$k_d:$ ' + str(self.kd) + '\t$Refresh Rate:$ ' + str(self.duration) + '$s$')
        ax.set_xlabel('Time (s)', fontweight='bold')
        ax.set_ylabel('Position (m)', fontweight='bold')

        # Set ylim, legend, and grid
        bottom, top = ax.get_ylim()
        ax.set_ylim(bottom=bottom-0.5)
        plt.gca().legend(('North Actual','North Desired', 'East Actual', 'East Desired',
            'Down Actual', 'Down Desired'), ncol=3, loc='lower center')
        ax.grid()

        # Show the plot
        plt.show()

    def trajectoryGen(self, IC, T, sampleRate, plotFlag):
        # Define time array and storage variables
        tt = np.linspace(0, T, round(T/sampleRate), endpoint=True)
        pos = []; vel = []; acc = [];

        # Find coeffcients of 5th order polynomial using matrix operations
    	A = np.array([[0, 0, 0, 0, 0, 1],
    				[np.power(T,5), np.power(T,4), np.power(T,3), np.power(T,2), T, 1],
    				[0, 0, 0, 0, 1, 0],
    				[5*np.power(T,4), 4*np.power(T,3), 3*np.power(T,2), 2*T, 1, 0],
    				[0, 0, 0, 2, 0, 0],
    				[20*np.power(T,3), 12*np.power(T,2), 6*T, 2, 0, 0]])

    	b = np.array([IC[0], IC[1], IC[2], IC[3], IC[4], IC[5]])

    	x = np.linalg.solve(A, b)

        # Unpack coeffcients
        A = x[0]; B = x[1]; C = x[2]; D = x[3]; E = x[4]; F = x[5];

        # Calculate the trajectory properties for each time step and store
        for t in tt:
        	pos.append(A*np.power(t,5) + B*np.power(t,4) + C*np.power(t,3) + D*np.power(t,2) + E*t + F)
        	vel.append(5*A*np.power(t,4) + 4*B*np.power(t,3) + 3*C*np.power(t,2) + 2*D*t + E)
        	acc.append(20*A*np.power(t,3) + 12*B*np.power(t,2) + 6*C*t + 2*D)

        # Plot the results if prompt
        if plotFlag is True:
            ax1 = plt.subplot(311)
            plt.title('Position | Velocity | Acceleration')
            plt.plot(tt, pos)
            plt.setp(ax1.get_xticklabels(), visible=False)
            plt.ylabel('Pos [m]')

            ax2 = plt.subplot(312, sharex=ax1)
            plt.plot(tt, vel)
            plt.setp(ax2.get_xticklabels(), visible=False)
            plt.ylabel('Vel [m/s]')

            ax3 = plt.subplot(313, sharex=ax1)
            plt.plot(tt, acc)
            plt.setp(ax3.get_xticklabels(), fontsize=10)
            plt.ylabel('Acc [m/s^2]')

            plt.xlabel('time (s)')
            plt.show()

        # Return the resulting position
        return pos

def main():
    # Connect to the Vehicle
    connection_string = "127.0.0.1:14551"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    # Set up controller class
    C = Controller()

    # Setup simulation class and take off
    sim = Simulate()
    sim.armAndTakeOff(C, vehicle, 0.5)

    # Simulate testing options
    # C.northEastTest(vehicle)
    # C.altitudeTest(vehicle)
    # C.fullTest(vehicle)
    # C.commandTest(vehicle)
    C.trajectoryControl(vehicle)

    # Land the UAV and close connection
    print("Closing vehicle connection\n")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()

# Main loop
if __name__ == '__main__':
	main()
