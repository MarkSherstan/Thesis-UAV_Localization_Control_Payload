# https://github.com/oseiskar/simdkalman/issues/3
import numpy as np
import simdkalman
import pandas as pd
import time
import matplotlib.pyplot as plt
import statistics
from simdkalman.primitives import update

# Function defintions
def movAvg(a, n) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def kf_smooth(y):
    return kf.smooth(y).observations.mean

def kf_filter(y):
    return kf.compute(y, 0, filtered=True).filtered.observations.mean

# Import data
fileName = '2020-06-14__02-30-58.csv' 
try:
    df = pd.read_csv(fileName, header = 0, names = ['Mode', 'Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control'])
except:
    print('Error with file.')
    exit()

data = np.array(df['Yaw-Vision'])

# Configure the Kalman Filter
kf = simdkalman.KalmanFilter(
    state_transition = [[1,1],[0,1]],        # A
    process_noise = np.diag([0.05, 0.002]),  # Q
    observation_model = np.array([[1,0]]),   # H
    observation_noise = 20.0)                # R

# Simulate real time

# Prepare kalman filter
state_transition = np.array([[1,1],[0,1]])
process_noise = np.diag([0.05, 0.002])
observation_model = np.array([[1,0]])
observation_noise = np.array([[10.0]])

# Initial values
prevMean = np.zeros((1, 2, 1)) # NOTE: (N, 1) -> (N, 2, 1)
prevCov = np.tile(np.eye(2), (1, 1, 1)) # shape = (N_STREAMS, 2, 2)
observations = np.random.normal(size=(1, 1, 1)) # NOTE: (N, 1) -> (N, 1, 1)

# Storage variables
meanList = []
covarList = []
timeLog = [0]

# Start up
timer = time.time()

# Real time sim
for ii in range(data.size):
    # Kalman filter
    means, covariances = update(
        prevMean,
        prevCov,
        observation_model,
        observation_noise,
        np.array([[[data[ii]]]]))

    prevMean = means
    prevCov  = covariances
    meanList.append(means[0][0][0])
    covarList.append(covariances[0][0][0])

    timeLog.append(time.time()-timer)
    timer = time.time()

print("Average rate: ", statistics.mean(timeLog), "+/-", statistics.stdev(timeLog))

# Movin average calcs -> May be wrong. Double check indices
A = movAvg(data, 10)
B = movAvg(data, 30)

# Plot the dataq
t = range(data.size)
plt.plot(t, data, 'kx', alpha=0.3, label='data')
plt.plot(t, kf_smooth(data), 'k-', label='Kalman Smoothed')
plt.plot(t, kf_filter(data), 'k--', label='Kalman Filtered')
plt.plot(t, meanList, 'k.', label='Real Time Kalman')
# plt.plot(t, covarList, 'k+', label='Real Time Kalman')
# plt.plot(range(A.size), A, 'k-.', label='10 Point Moving Avg')
# plt.plot(range(B.size), B, 'k.', label='30 Point Moving Avg')
plt.legend()
plt.show()
