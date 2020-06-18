from simdkalman.primitives import update
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statistics
import simdkalman
import time

# Function defintions
def movAvg(x, N):
    cumsum = np.cumsum(np.insert(x, 0, 0)) 
    return (cumsum[N:] - cumsum[:-N]) / float(N)

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

# ONLINE: Simulation

# Define model (No A or Q matrix required???)
# Look at source code -> Can slap that into the paper
observation_model = np.array([[1,0]])
observation_noise = np.array([[20.0]])

# Initial state
m = np.array([0, 1])
P = np.eye(2)

# Logging
meanList = []
timeLog = [0]
timer = time.time()

# Real time sim
for ii in range(data.size):
    # Kalman filter
    m1, P1 = update(m, P, observation_model, observation_noise, np.array([data[ii]]))

    m = m1
    P1  = P
    meanList.append(m[0][0])

    timeLog.append(time.time()-timer)
    timer = time.time()
    
print("Average rate: ", statistics.mean(timeLog), "+/-", statistics.stdev(timeLog))

# OFFLINE: Configure the Kalman Filter
kf = simdkalman.KalmanFilter(
    state_transition = [[1,1],[0,1]],        # A
    process_noise = np.diag([0.05, 0.002]),  # Q
    observation_model = np.array([[1,0]]),   # H
    observation_noise = 20.0)                # R

# Movin average calcs -> n-1 terms short
A = movAvg(data, 10)
B = movAvg(data, 30)

# Plot the dataq
t = range(data.size)
plt.plot(t, data, 'kx', alpha=0.3, label='Raw Data')
plt.plot(t, kf_smooth(data), 'k.', label='Offline: Kalman Smoothed')
plt.plot(t, kf_filter(data), 'k--', label='Offline: Kalman Filtered')
plt.plot(t, meanList, 'k-', label='Online: Kalman Filtered')
# plt.plot(range(A.size), A, 'k+', label='10 Point Moving Avg')
# plt.plot(range(B.size), B, 'ko', label='30 Point Moving Avg')
plt.xlabel('Index')
plt.ylabel('Yaw [deg]')
plt.legend()
plt.show()
