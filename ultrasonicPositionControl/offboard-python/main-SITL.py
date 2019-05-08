from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time
import math

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
        print(" ")

    def disarmAndLand(self, vehicle):
        # Land the drone
        self.vehicleMode = "RTL"
        print("\nSetting RTL mode...\n")
        vehicle.mode = VehicleMode(self.vehicleMode)
        time.sleep(4)

        # Close vehicle object
        print("Closing vehicle connection")
        vehicle.close()

class Controller:
    def __init__(self):
        # Initial conditions (rads)
        self.rollAngle = 0.0
        self.pitchAngle = 0.0
        self.yawAngle = None
        self.yawRate = 0.0
        self.useYawRate = False
        self.thrust = 0.5
        self.duration = 0.5

        # Angle Constraint
        self.minVal = -3.1415/6
        self.maxVal = 3.1415/6

        # Controller PID Gains
        self.kp = 1
        self.ki = 1
        self.kd = 1

        # Controller
        self.northDesired = None
        self.northPreviousPos = None
        self.startTime = None
        self.I = 0

        # Data logging for controller
        self.desiredList = []
        self.actualList = []
        self.errorList = []
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
        if self.yawAngle is None:
            self.yawAngle = vehicle.attitude.yaw

        # Create the mavlink message
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
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

        # Reset attitude, or it will persist for 1s more due to the timeout
        self.rollAngle = 0
        self.pitchAngle = 0
        self.yawAngle = 0
        self.sendAttitudeTarget(vehicle)

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

    def PID(self, vehicle):
        # Get important values and begin calcs
        northCurrentPos = vehicle.location.local_frame.north
        error = northCurrentPos - self.northDesired
        deltaT = time.time() - self.startTime

        if self.northPreviousPos is None:
            self.northPreviousPos = northCurrentPos

        # Append values
        self.desiredList.append(self.northDesired)
        self.actualList.append(northCurrentPos)
        self.errorList.append(error)
        self.timeList.append(deltaT)

        # Run the PID controller
        P = self.kp * error
        self.I += self.ki * error
        D = self.kd * ((northCurrentPos - self.northPreviousPos) / deltaT)
        controller = P + self.I + D

        # Save the previous position
        self.northPreviousPos = northCurrentPos

        # Execute the controller and print results
        self.pitchAngle = self.constrain(controller)
        self.setAttitude(vehicle)

        print 'N: ', round(vehicle.location.local_frame.north,3), ' E: ', round(vehicle.location.local_frame.east,3), ' D: ', round(vehicle.location.local_frame.down,3)
        print 'Actual: ', round(vehicle.location.local_frame.north,3), 'Error: ', round(error,3), 'Input [deg]: ', round(math.degrees(self.constrain(controller)),3)
        print(" ")

    def plotController(self):
        # Import required modules
        import matplotlib.pyplot as plt
        import matplotlib

        # Make subplot and actually plot
        fig, ax = plt.subplots()
        ax.plot(self.timeList, self.actualList, self.timeList, self.desiredList)

        # Set labels, titles, info, legend and grid
        ax.set(xlabel='Time (s)',
                ylabel='position (m)',
                title='Position Control North\n' +
                ' Kp: ' + str(self.kp) +
                ' Ki: ' + str(self.ki) +
                ' Kd: ' + str(self.kd))

        plt.gca().legend(('actual','desired'))
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
    sim.armAndTakeOff(vehicle, 2.5)

    # Set up controller class
    C = Controller()
    C.northDesired = 1.0
    C.startTime = time.time()

    # Run the controller for 50 itterations
    for ii in range(40):
        C.PID(vehicle)

    # Land the UAV and close connection
    sim.disarmAndLand(vehicle)

    # Plot the results
    C.plotController()


# Main loop
if __name__ == '__main__':
	main()
