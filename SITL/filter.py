from simdkalman.primitives import update, predict_observation, predict
import numpy as np

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

class Olympic:
    def __init__(self, windowSize):
        self.windowSize = windowSize
        self.values = []
        self.sum = 0
    
    def update(self, value):
        # Update master list
        self.values.append(value)
        if len(self.values) > self.windowSize:
            self.values.pop(0)
    
        # Remove the largest and smallest value
        temp = self.values.copy()
        if (len(temp) >= 3):
            temp.remove(max(temp))
            temp.remove(min(temp))
        
        # Calculate the average
        return float(sum(temp)) / len(temp)
    
class KalmanFilter:
    def __init__(self):
        # Standard deviations 
        self.sigmaE = 1.0        # deg / s / s
        sigmaTheta = 50          # deg
        sigmaOmega = 0.1         # deg / s
        
        # Configure the filter
        self.observationModel = np.diag([1, 1])                             # H
        self.observationNoise = np.diag([sigmaTheta**2, sigmaOmega**2])     # R (Measurment noise)

        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dt, dataIn):
        stateTransition  = np.array([[1, dt],[0,1]])                        # A
        processNoise = np.diag([0.25*(dt**4), dt**2]) * self.sigmaE         # Q
        
        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, stateTransition, processNoise)

        return dataOut[0][0]
