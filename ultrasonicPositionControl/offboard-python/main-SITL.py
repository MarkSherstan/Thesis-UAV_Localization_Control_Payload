from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time
import math
import numpy as np

class Simulate:
    def __init__(self):
        self.vehicleArmed = True
        self.vehicleMode = "GUIDED" #GUIDED_NOGPS

    def armAndTakeOff(self, vehicle, targetAltitude):
        # Give the user some info
        print("Basic pre-arm checks")
        print("--------------------------------------------------")

        # Wait till vehicle is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
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

        vehicle.simple_takeoff(targetAltitude)
        time.sleep(2)

        # Display the results
        print 'Took off to: ', targetAltitude, ' m'
        print 'N: ', round(vehicle.location.local_frame.north,4), ' E: ', round(vehicle.location.local_frame.east,4), ' D: ', round(vehicle.location.local_frame.down,4)
        print("\n")

    def disarmAndLand(self, vehicle):
        # Land the drone
        self.vehicleMode = "RTL"
        print("\nSetting RTL mode...")
        vehicle.mode = VehicleMode(self.vehicleMode)
        time.sleep(4)

        # Close vehicle object
        print("Closing vehicle connection\n")
        vehicle.close()

class Controller:
    def __init__(self):
        # Initial conditions (rads)
        self.rollAngle = 0.0
        self.pitchAngle = 0.0
        self.yawAngle = None
        self.yawRate = 0.0
        self.useYawRate = True
        self.thrust = 0.5
        self.duration = 0.3

        # Angle Constraint
        self.minVal = -3.1415/6
        self.maxVal = 3.1415/6

        # Controller PD Gains
        self.kp = 0.17
        self.kd = 0.90

        # Controller
        self.northDesired = None
        self.northPreviousPos = None
        self.eastDesired = None
        self.eastPreviousPos = None
        self.startTime = None

        # Save values for plotting
        self.northDesiredList = []
        self.northActualList = []
        self.eastDesiredList = []
        self.eastActualList = []
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

        # Update the yaw angle
        # if self.yawAngle is None:
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
            time.sleep(0.1)

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
        self.pitchAngle = self.constrain(pitchControl)
        self.rollAngle = self.constrain(-rollControl)
        self.setAttitude(vehicle)

        print 'N: ', round(vehicle.location.local_frame.north,3), \
            ' E: ', round(vehicle.location.local_frame.east,3), \
            ' D: ', round(vehicle.location.local_frame.down,3)
        print 'N Error: ', round(errorNorth,3), 'E Error: ', round(errorEast,3)
        print 'Input N: ', round(math.degrees(self.constrain(pitchControl)),3), \
            'Input E: ', round(math.degrees(self.constrain(rollControl)),3), '[deg]'
        print(" ")

    def pitchTest(self, vehicle):
        self.pitchAngle = math.radians(5.0)
        self.setAttitude(vehicle)
        time.sleep(1)

        self.pitchAngle = 0.0
        self.setAttitude(vehicle)
        time.sleep(1)

        self.pitchAngle = math.radians(-5.0)
        self.setAttitude(vehicle)
        time.sleep(1)

        self.pitchAngle = 0.0
        self.setAttitude(vehicle)
        time.sleep(1)

    def altitudeTest(self, vehicle):
        # Sleep for stability and set desired parameters
        time.sleep(5)
        goal = -1
        kp = 0.5
        self.minVal = -2
        self.maxVal = 2
        down = []
        startTime = time.time()

        for ii in range(60):
            # Get actual value and add to list
            actual = vehicle.location.local_frame.down
            self.timeList.append(time.time() - startTime)
            down.append(actual)

            # Calculate error and constrain the value
            error = actual - goal
            control = self.constrain(kp * error)

            # Set the thrust value between 0 and 1 and send command
            self.thrust = np.interp(control,[-2,-1,0,1,2],[0, 0.25, 0.5, 0.75, 1])
            self.setAttitude(vehicle)

            # Print values to screen
            print 'Actual: ', round(actual,3), 'Error: ', round(error,3), 'Control: ', round(control,3), 'Command: ', round(self.thrust,3)
            print(" ")

        # Import packages and plot the results
        import matplotlib.pyplot as plt
        import matplotlib

        fig, ax = plt.subplots()
        ax.plot(self.timeList, down)

        # Set labels, titles, and grid
        ax.set(xlabel='Time (s)', ylabel='Altitude (m)', title='Altitude Control')
        ax.grid()

        # Show results and save the plot
        plt.show()


    def plotController(self):
        # Import required modules
        import matplotlib.pyplot as plt
        import matplotlib

        # Make subplot and actually plot
        fig, ax = plt.subplots()
        ax.plot(self.timeList, self.northActualList, self.timeList, self.northDesiredList,
            self.timeList, self.eastActualList, self.timeList, self.eastDesiredList)

        # Set labels, titles, info, legend and grid
        ax.set(xlabel='Time (s)',
                ylabel='position (m)',
                title='Position Control\n' +
                ' Kp: ' + str(self.kp) +
                ' Kd: ' + str(self.kd))

        plt.gca().legend(('North Actual','North Desired', 'East Actual', 'East Desired'))
        ax.grid()

        # Show results and save the plot
        fig.savefig("positionController.png")
        plt.show()

def main():
    # Connect to the Vehicle
    connection_string = "127.0.0.1:14551"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    # Setup simulation class and take off
    sim = Simulate()
    sim.armAndTakeOff(vehicle, 0.4)

    # Set up controller class
    C = Controller()
    C.northDesired = vehicle.location.local_frame.north + 1.0
    C.eastDesired = vehicle.location.local_frame.east - 0.5
    C.startTime = time.time()

    # Run the controller for 50 itterations
    # for ii in range(50):
    C.altitudeTest(vehicle)

    # Land the UAV and close connection
    sim.disarmAndLand(vehicle)

    # # Plot the results
    # C.plotController()


# Main loop
if __name__ == '__main__':
	main()
