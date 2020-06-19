from simdkalman.primitives import update, predict_observation, predict
import numpy as np

class MovingAverage:
    def __init__(self, windowSize):
        self.windowSize = windowSize
        self.values = []
        self.sum = 0

    def avg(self, value):
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.windowSize:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)

class KalmanFilter:
    def __init__(self):
        # Configure the filter
        self.stateTransition  = np.array([[1,1],[0,1]])  # A
        self.processNoise     = np.diag([0.03, 0.003])   # Q (Deviation from assumed model)
        self.observationModel = np.array([[1,0]])        # H
        self.observationNoise = np.array([[200]])         # R (Measurment noise)

        # 0.03, 0.003
        # 500


        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dataIn):
        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array([dataIn]))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, self.stateTransition, self.processNoise)

        return dataOut[0][0]