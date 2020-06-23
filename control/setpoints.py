import numpy as np

class SetPoints:
    def __init__(self, northDesired, eastDesired, downDesired):
        # Desired positions
        self.northDesired = northDesired 
        self.eastDesired  = eastDesired
        self.downDesired  = downDesired
        
        # Trajectory list
        self.northDesiredList = None
        self.eastDesiredList  = None
        self.index = 0

    def selectMethod(self, Q, trajectory):
            if (trajectory == True):
                # Find the initial position
                north0, east0 = self.initialPosition(Q)
                
                # Calculate the trajectories
                self.northDesiredList = self.trajectoryGen(north0, self.northDesired, T=5)
                self.eastDesiredList = self.trajectoryGen(east0, self.eastDesired, T=5)
                print('Trajectory ready')
            else:
                print('Standard setpoints ready')
                    
    def getDesired(self):
        # Extract set points
        if (self.index > len(self.northDesiredList)):
            return [self.northDesired, self.eastDesired, self.downDesired]
        else:
            northTemp = self.northDesiredList[self.index]
            eastTemp  = self.eastDesiredList[self.index]
            self.index += 1
            return [northTemp, eastTemp, self.downDesired]
    
    def initialPosition(self, Q, pts=10):
        # Initialize counter
        north = 0
        east  = 0

        # Sum points 
        for _ in range(pts):
            temp = Q.get()
            north += temp[0]
            east  += temp[1]
        
        # Calc the average and return 
        north0 = north / pts
        east0  = east / pts       
        return north0, east0

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