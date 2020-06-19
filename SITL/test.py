from simdkalman.primitives import update, predict_observation, predict
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statistics
import simdkalman
import time

from filter import KalmanFilter, MovingAverage

##########################
# Import data
##########################
fileName = '2020-06-18__20-52-05.csv' 
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
# data = data[0:500]

##########################
# ONLINE: Simulation
##########################

# Configure the filter
state_transition = np.array([[1,1],[0,1]])  # A
process_noise = np.diag([0.03, 0.001])      # Q: CHANGE THESE
observation_model = np.array([[1,0]])       # H
observation_noise = np.array([[100.0]])     # R: CHANGE THESE

# Initial state
m = np.array([0, 1])
P = np.eye(2)

# Create class instance 
yawKF = KalmanFilter()
flt = MovingAverage(5)

# Logging
kalmanRaw = []
kalmanFilt = []

# Real time sim
for ii in range(data.size):
    temp = yawKF.update(data[ii])
    kalmanRaw.append(temp)
    temp = flt.avg(temp)
    kalmanFilt.append(temp)

##########################
# Moving average calcs
##########################
def movAvg(x, N):
    # Return N-1 terms short (total length)
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    return (cumsum[N:] - cumsum[:-N]) / float(N)

# A = movAvg(meanList, 5)
# B = movAvg(data, 30)

##########################
# OFFLINE: Configure the Kalman Filter
##########################
kf = simdkalman.KalmanFilter(
state_transition = [[1,1],[0,1]],        # A
process_noise = np.diag([0.05, 0.002]),  # Q
observation_model = np.array([[1,0]]),   # H
observation_noise = 20.0)                # R

kf = kf.em(data, n_iter=10)

# Initial state
m = np.array([0, 1])
P = np.eye(2)

##########################
# Plot the data
##########################
t = range(data.size)
plt.plot(t, data, 'k-', alpha=0.2, label='Raw Data')
# plt.plot(t, kf.smooth(data).observations.mean, 'k.', label='Offline: Kalman Smoothed')
# plt.plot(t, kf.compute(data, 0, smoothed=False, filtered=True, initial_value = m, initial_covariance = P).filtered.observations.mean, 'k+', label='Offline: Kalman Filtered')
plt.plot(t, kalmanRaw, 'k--', label='Online: Kalman Filter')
plt.plot(t, kalmanFilt, 'k-', label='Online: Kalman w/ Low Pass Filter')
# plt.plot(range(A.size), A, 'k-', label='10 Point Moving Avg - Kalman')
# plt.plot(range(B.size), B, 'k--', label='30 Point Moving Avg')
plt.xlabel('Index')
plt.ylabel('Yaw [deg]')
plt.legend()
plt.show()
