from simdkalman.primitives import update, predict_observation, predict
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statistics
import simdkalman
import time

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
data = data[0:400]                                                              ### REMOVE THIS LATER

##########################
# ONLINE: Simulation
##########################

# Configure the filter
state_transition = np.array([[1,1],[0,1]])  # A
process_noise = np.diag([0.05, 0.002])      # Q
observation_model = np.array([[1,0]])       # H
observation_noise = np.array([[20.0]])      # R

# Initial state
m = np.array([0, 1])
P = np.eye(2)

# Logging
meanList = []
timeLog = [0]
timer = time.time()

# Real time sim
for ii in range(data.size):
    m1, P1 = update(m, P, observation_model, observation_noise, np.array([data[ii]]))
    a, _ = predict_observation(m1, P1, observation_model, observation_noise)
    m1, P1 = predict(m1, P1, state_transition, process_noise)
    
    m = m1
    P = P1
    meanList.append(a[0][0])

    timeLog.append(time.time()-timer)
    timer = time.time()

print("Average rate: ", statistics.mean(timeLog), "+/-", statistics.stdev(timeLog))

##########################
# Moving average calcs
##########################
def movAvg(x, N):
    # Return N-1 terms short (total length)
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    return (cumsum[N:] - cumsum[:-N]) / float(N)

A = movAvg(data, 10)
B = movAvg(data, 30)

##########################
# OFFLINE: Configure the Kalman Filter
##########################
kf = simdkalman.KalmanFilter(
state_transition = [[1,1],[0,1]],        # A
process_noise = np.diag([0.05, 0.002]),  # Q
observation_model = np.array([[1,0]]),   # H
observation_noise = 20.0)                # R

# Initial state
m = np.array([0, 1])
P = np.eye(2)

##########################
# Plot the data
##########################
t = range(data.size)
plt.plot(t, data, 'kx', alpha=0.3, label='Raw Data')
plt.plot(t, kf.smooth(data).observations.mean, 'k.', label='Offline: Kalman Smoothed')
# plt.plot(t, kf.compute(data, 0, smoothed=False, filtered=True, initial_value = m, initial_covariance = P).filtered.observations.mean, 'k+', label='Offline: Kalman Filtered')
plt.plot(t, meanList, 'k-', label='Online: Kalman Filtered')
# plt.plot(range(A.size), A, 'k+', label='10 Point Moving Avg')
# plt.plot(range(B.size), B, 'k--', label='30 Point Moving Avg')
plt.xlabel('Index')
plt.ylabel('Yaw [deg]')
plt.legend()
plt.show()
