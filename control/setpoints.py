import numpy as np

class SetPoints:
    def __init__(self, state, nDesired=0, eDesired=0, dDesired=0, yDesired=0, args=None):
        '''
        Example uses:
        1. SP = SetPoints(state='Trajectory', nDesired=-12, eDesired=40, dDesired=10, yDesired=0)
        2. SP = SetPoints(state='Step')
        3. SP = SetPoints(state='Wave', args='Y')
            * Complete args list = 'Y', 'RP', 'T'
            * Must replace:
                rollControl, pitchControl, yawControl, thrustControl, landState = C.positionControl(actualPos, desiredPos)
               with
                rollControl = desiredPos[1]; pitchControl = desiredPos[0]; yawControl = desiredPos[3]; thrustControl = desiredPos[2]; 
        '''
        
        # Save arguments for some functions
        self.args = args
        
        # Desired pose
        self.northDesired = nDesired 
        self.eastDesired  = eDesired
        self.downDesired  = dDesired
        self.yawDesired   = yDesired

        # Desired pose list
        self.northDesiredList = []
        self.eastDesiredList  = []
        self.downDesiredList  = []
        self.yawDesiredList   = []
        self.index = 0

        # Save the desired state
        if (state not in ['Trajectory', 'Step', 'Wave']):
            print('Selected state does not exist. Use one of the following: \
                  \n\t Trajectory \
                  \n\t Step       \
                  \n\t Wave')
            exit()
        else:
            self.state = state
    
    def reset(self, nDesired, eDesired, dDesired, yDesired):
        # Desired Pose
        self.northDesired = nDesired 
        self.eastDesired  = eDesired
        self.downDesired  = dDesired
        self.yawDesired   = yDesired
        
    def update(self, posIC, velIC, accIC):
        # Reset lists
        self.northDesiredList = []
        self.eastDesiredList  = []
        self.downDesiredList  = []
        self.yawDesiredList   = []
        self.index = 0
        
        # State selection
        if self.state == 'Trajectory':
            self.createTrajectory(posIC, velIC, accIC)
        elif self.state == 'Step':
            self.createStep(posIC)
        elif self.state == 'Wave':
            self.downDesired = 0.5
            self.createWave(axis=self.args)
        else:
            print('State setpoint error')
            
    def getDesired(self):
        # North
        if (self.index >= len(self.northDesiredList)):
            northSP = self.northDesired
        else:
            northSP = self.northDesiredList[self.index]
        
        # East
        if (self.index >= len(self.eastDesiredList)):
            eastSP = self.eastDesired
        else:
            eastSP = self.eastDesiredList[self.index]
          
        # Down  
        if (self.index >= len(self.downDesiredList)):
            downSP = self.downDesired
        else:
            downSP = self.downDesiredList[self.index]

        # Yaw  
        if (self.index >= len(self.yawDesiredList)):
            yawSP = self.yawDesired
        else:
            yawSP = self.yawDesiredList[self.index]

        # Increment counter and return values
        self.index += 1
        return [northSP, eastSP, downSP, yawSP]
    
    def createTrajectory(self, posIC, velIC, accIC):
        # Calculate the trajectories
        self.northDesiredList = self.trajectoryGen(posIC[0], velIC[0], accIC[0], self.northDesired, T=5)
        self.eastDesiredList  = self.trajectoryGen(posIC[1], velIC[1], accIC[1], self.eastDesired,  T=5)
        self.downDesiredList  = self.trajectoryGen(posIC[2], velIC[2], accIC[2], self.downDesired,  T=7)

    def trajectoryGen(self, pos0, vel0, acc0, endPos, T, sampleRate=1/30):
        # Define time array and storage variables
        tt = np.linspace(0, T, round(T/sampleRate), endpoint=True)

        # Find coeffcients of 5th order polynomial using matrix operations.
        A = np.array([[0, 0, 0, 0, 0, 1],
                    [np.power(T,5), np.power(T,4), np.power(T,3), np.power(T,2), T, 1],
                    [0, 0, 0, 0, 1, 0],
                    [5*np.power(T,4), 4*np.power(T,3), 3*np.power(T,2), 2*T, 1, 0],
                    [0, 0, 0, 2, 0, 0],
                    [20*np.power(T,3), 12*np.power(T,2), 6*T, 2, 0, 0]])

        b = np.array([pos0, endPos, vel0, 0, acc0, 0])
        x = np.linalg.solve(A, b)

        # Unpack coeffcients
        A = x[0]; B = x[1]; C = x[2]; D = x[3]; E = x[4]; F = x[5]

        # Calculate the trajectory properties for each time step and store
        pos = A*np.power(tt,5) + B*np.power(tt,4) + C*np.power(tt,3) + D*np.power(tt,2) + E*tt + F

        # Return the resulting position
        return pos.tolist()

    def createStep(self, posIC, sampleRate=1/30):
        # Two second steady state
        n = int(2.0 / sampleRate)
        
        # Update the lists
        self.northDesiredList = [posIC[0]]
        self.eastDesiredList  = [posIC[1]] * n
        self.downDesiredList  = [posIC[2]]
        
        # Update set points
        self.northDesired = posIC[0] 
        self.eastDesired  = posIC[1] + 30
        self.downDesired  = posIC[2] 

    def createWave(self, axis):
        # Actual controller inputs NOT desired positions!!!
        if (axis == 'Y'):
            # Oscillate just yaw
            self.northDesired = 0 
            self.eastDesired  = 0
            self.downDesired  = 0.5
            self.yawDesiredList = self.sineWaveGenerator(30)
        elif (axis == 'RP'):
            # Oscillate roll and pitch
            self.northDesiredList = self.sineWaveGenerator(3)
            self.eastDesiredList = self.sineWaveGenerator(3)
            self.downDesired = 0.5
            self.yawDesired  = 0
        elif (axis == 'T'):
            # scillate thrust
            self.northDesired = 0
            self.eastDesired  = 0
            self.downDesiredList = self.sineWaveGenerator(A=0.05, b=0.5)
            self.yawDesired   = 0
        else:
            print('Error in selected state')     

    def sineWaveGenerator(self, A, b=0, T=5, sampleRate=1/30, plotFlag=False):
        # Time array
        x = np.linspace(0, T, round(T/sampleRate), endpoint=True)

        # Function parameters
        f = 30
        fs = 30

        # Output
        y = A*np.sin(2*np.pi*f * (x/fs)) + b

        # Visualizing
        if plotFlag is True:
            # Import packages
            import matplotlib.pyplot as plt

            # Display the results
            plt.plot(x,y)
            plt.show()
        
        # Return the result 
        return y

    def dampedSineWaveGenerator(self, A, b=0, T=6, sampleRate=1/30, plotFlag=False):
        # Time array
        x = np.linspace(0, T/3, round((T/3)/sampleRate), endpoint=True)
        xx = np.linspace(0, T, round((T/sampleRate)), endpoint=True)

        # Function parameters
        f = 30
        fs = 30

        # Output
        y1 = np.exp(-x) * A*np.sin(2*np.pi*f * (x/fs)) + b          # Decay fnc
        y2 = A*np.sin(2*np.pi*f * (x/fs)) + b                       # Sine fnc
        y3 = np.concatenate((np.zeros(1), -np.flip(y1), y2[1:-1], y1, np.zeros(1)), axis=0)   # Combine
        
        # Visualizing
        if plotFlag is True:
            # Import packages
            import matplotlib.pyplot as plt

            # Display the results
            plt.plot(xx, y3)
            plt.show()
        
        # Return the result 
        return y3
    
def main():
    SP = SetPoints('Wave')
    SP.dampedSineWaveGenerator(A=10, plotFlag=True)
    
if __name__ == "__main__":
    main()
