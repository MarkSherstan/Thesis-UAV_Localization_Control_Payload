from simdkalman.primitives import update, update_with_nan_check, predict_observation, predict
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

class KalmanFilterRot:
    def __init__(self):
        # Standard deviations 
        self.sigmaE = 10.0     # deg / s / s
        sigmaTheta  = 10.0     # deg
        sigmaOmega  = 0.5      # deg / s
        
        # Configure the filter
        self.observationModel = np.diag([1, 1])                             # H
        self.observationNoise = np.diag([sigmaTheta**2, sigmaOmega**2])     # R

        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dt, dataIn):
        stateTransition  = np.array([[1, dt],[0,1]])                        # A
        processNoise = np.diag([0.25*(dt**4), dt**2]) * self.sigmaE**2      # Q 
        
        m, P = update(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, stateTransition, processNoise)

        return dataOut[0][0]

class KalmanFilterPos:
    def __init__(self):
        # Standard deviations 
        self.sigmaE = 10.0   # cm / s / s
        sigmaPos  = 1.0      # cm
        
        # Configure the filter
        self.observationModel = np.array([[1, 0]])            # H
        self.observationNoise = np.array([[sigmaPos**2]])     # R

        # Initial state
        self.m = np.array([0, 1])
        self.P = np.eye(2)

    def update(self, dt, dataIn):
        stateTransition  = np.array([[1, dt],[0,1]])                        # A
        processNoise = np.diag([0.25*(dt**4), dt**2]) * self.sigmaE**2      # Q 
        
        m, P = update_with_nan_check(self.m, self.P, self.observationModel, self.observationNoise, np.array(dataIn))
        dataOut, _ = predict_observation(m, P, self.observationModel, self.observationNoise)
        self.m, self.P = predict(m, P, stateTransition, processNoise)

        return dataOut[0][0]
    