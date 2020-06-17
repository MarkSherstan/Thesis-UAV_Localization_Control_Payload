# https://github.com/oseiskar/simdkalman/issues/3
import numpy as np
import simdkalman
import pandas as pd
import matplotlib.pyplot as plt

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

# Movin average calcs -> May be wrong. Double check indices
A = movAvg(data, 10)
B = movAvg(data, 30)

# Plot the data
t = range(data.size)
plt.plot(t, data, 'kx', alpha=0.3, label='data')
plt.plot(t, kf_smooth(data), 'k-', label='Kalman smoothed')
plt.plot(t, kf_filter(data), 'k--', label='Kalman filtered')
# plt.plot(range(A.size), A, 'k-.', label='10 Point Moving Avg')
# plt.plot(range(B.size), B, 'k.', label='30 Point Moving Avg')
plt.legend()
plt.show()
