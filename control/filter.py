from simdkalman.primitives import update, predict_observation, predict
import numpy as np
import time

class MovingAverage:
    def __init__(self, windowSize):
        self.windowSize = windowSize
        self.values = []
        self.sum = 0

    def update(self, value):
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.windowSize:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)

class KalmanFilter1x:
    def __init__(self, sigmaX, sigmaXdotdot):
        # Standard deviations
        self.sigmaX = sigmaX              # Position
        self.sigmaXdotdot = sigmaXdotdot  # Acceleration

        # Configure the filter
        self.observationModel = np.array([[1, 0]])          # H
        self.observationNoise = np.array([[sigmaX**2]])     # R

        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dt, dataIn):
        stateTransition  = np.array([[1, dt],[0,1]])                            # A
        processNoise = np.diag([0.25*(dt**4), dt**2]) * self.sigmaXdotdot**2    # Q

        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, stateTransition, processNoise)

        return dataOut[0][0]

class KalmanFilter2x:
    def __init__(self, sigmaX, sigmaXdot, sigmaXdotdot):
        # Standard deviations
        self.sigmaX = sigmaX              # Position
        self.sigmaXdot = sigmaXdot        # Velocity
        self.sigmaXdotdot = sigmaXdotdot  # Acceleration

        # Configure the filter
        self.observationModel = np.diag([1, 1])                                 # H
        self.observationNoise = np.diag([self.sigmaX**2, self.sigmaXdot**2])    # R

        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dt, dataIn):
        stateTransition  = np.array([[1, dt],[0,1]])                            # A
        processNoise = np.diag([0.25*(dt**4), dt**2]) * self.sigmaXdotdot**2    # Q

        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, stateTransition, processNoise)

        return dataOut[0][0]

class KalmanFilterNxN:
    def __init__(self, sigmaX, sigmaXdot, sigmaXdotdot):
        # Standard deviations
        self.sigmaX = sigmaX              # Position
        self.sigmaXdot = sigmaXdot        # Velocity
        self.sigmaXdotdot = sigmaXdotdot  # Acceleration

        # Configure the filter
        self.observationModel = np.diag([1, 1, 1, 1, 1, 1, 1, 1])               # H
        self.observationNoise = np.diag([self.sigmaX**2, self.sigmaXdot**2,     # R
                                         self.sigmaX**2, self.sigmaXdot**2,
                                         self.sigmaX**2, self.sigmaXdot**2,
                                         self.sigmaX**2, self.sigmaXdot**2]) 

        # Initial state
        self.m = np.array([0, 1, 0, 1, 0, 1, 0, 1])
        self.P = np.eye(8)

    def update(self, dt, dataIn):
        stateTransition  = np.array([[1, dt,  0,   0,  0,  0,  0,  0],          # A
                                     [0,  1,  0,   0,  0,  0,  0,  0], 
                                     [0,  0,  1,  dt,  0,  0,  0,  0], 
                                     [0,  0,  0,   1,  0,  0,  0,  0],
                                     [0,  0,  0,   0,  1, dt,  0,  0],
                                     [0,  0,  0,   0,  0,  1,  0,  0],
                                     [0,  0,  0,   0,  0,  0,  1, dt],
                                     [0,  0,  0,   0,  0,  0,  0,  1]])      
        processNoise = np.diag([0.25*(dt**4), dt**2,                            # Q
                                0.25*(dt**4), dt**2,
                                0.25*(dt**4), dt**2,
                                0.25*(dt**4), dt**2]) * self.sigmaXdotdot**2    
        
        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, stateTransition, processNoise)

        return [dataOut[0][0], dataOut[2][0], dataOut[4][0], dataOut[6][0]]
    
class TimeSync:
    def __init__(self, samplingRate):
        self.previousTime = None
        self.samplingRate = samplingRate

    def startTimer(self):
        self.previousTime = time.time()

    def stabilize(self):
        # Calculate time difference
        tempTime = time.time()
        timeDiff = tempTime - self.previousTime
        time2delay = self.samplingRate - timeDiff

        # Delay if loop is too fast
        if (time2delay > 0):
            time.sleep(time2delay)
            self.previousTime = time.time()
            return time2delay
        else:
            # Return delay time
            self.previousTime = time.time()
            return 0
