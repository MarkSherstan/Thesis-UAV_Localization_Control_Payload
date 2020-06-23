import numpy as np

class SetPoints:
    def __init__(self):
        # Initial position
        self.north0 = None
        self.east0  = None

    def initialPosition(self, Q, pts=10):
        # Initialize counter
        north = 0
        east  = 0

        # Sum points 
        for _ in range(pts):
            temp = Q.get()
            north += temp[0]
            east  += temp[1]
        
        # Save the average
        self.north0 = north / pts
        self.east0  = east / pts       
        
    def trajectoryControl(self, vehicle):
            # Initialize variables
            counter = 0
            T = 5

            # Set the start position
            northStart = 1.1
            eastStart = 0.8

            # Set the desired NED locations
            self.northDesired = 0.5
            self.eastDesired = 0.3
            self.downDesired = -0.4

            # Generate a trajectory that should take T seconds
            northIC = [northStart, self.northDesired, 0, 0, 0, 0]
            eastIC = [eastStart, self.eastDesired, 0, 0, 0, 0]

            pN = self.trajectoryGen(northIC, T, self.duration, False)
            pE = self.trajectoryGen(eastIC, T, self.duration, False)

            # Combined local frame with actual initial position
            northStart -= vehicle.location.local_frame.north
            eastStart -= vehicle.location.local_frame.east

            # Start timers
            self.startTime = time.time()
            prevTime = time.time()

            # Run for 2.5*T seconds
            while (time.time() < self.startTime + 2.5*T):
                # Get current values and transform to trajectory defined start location
                northCurrentPos = vehicle.location.local_frame.north + northStart # + (random.random()-0.5)*0.1
                eastCurrentPos = vehicle.location.local_frame.east + eastStart # + (random.random()-0.5)*0.1
                downCurrentPos = vehicle.location.local_frame.down # + (random.random()-0.5)*0.1
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

    def trajectoryGen(self, startPos, endPos, T, sampleRate=1/30):
        # Define time array and storage variables
        tt = np.linspace(0, T, round(T/sampleRate), endpoint=True)
        pos = []

        # Find coeffcients of 5th order polynomial using matrix operations. Zero vel and acc boundary conditions
        A = np.array([[0, 0, 0, 0, 0, 1],
                    [np.power(T,5), np.power(T,4), np.power(T,3), np.power(T,2), T, 1],
                    [0, 0, 0, 0, 1, 0],
                    [5*np.power(T,4), 4*np.power(T,3), 3*np.power(T,2), 2*T, 1, 0],
                    [0, 0, 0, 2, 0, 0],
                    [20*np.power(T,3), 12*np.power(T,2), 6*T, 2, 0, 0]])

        b = np.array([startPos, endPos, 0, 0, 0, 0])

        x = np.linalg.solve(A, b)

        # Unpack coeffcients
        A = x[0]; B = x[1]; C = x[2]; D = x[3]; E = x[4]; F = x[5]

        # Calculate the trajectory properties for each time step and store
        for t in tt:
            pos.append(A*np.power(t,5) + B*np.power(t,4) + C*np.power(t,3) + D*np.power(t,2) + E*t + F)

        # Return the resulting position
        return pos
