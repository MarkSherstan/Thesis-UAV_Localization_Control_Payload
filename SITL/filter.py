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
        # Configure the filter
        self.stateTransition  = np.array([[1, 1/30],[0,1]])  # A
        self.processNoise     = np.diag([0.03, 0.003])       # Q (Deviation from assumed model)
        self.observationModel = np.diag([1, 1])              # H
        self.observationNoise = np.diag([200, 10])           # R (Measurment noise)

        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dataIn):
        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, self.stateTransition, self.processNoise)

        return dataOut[0][0]
