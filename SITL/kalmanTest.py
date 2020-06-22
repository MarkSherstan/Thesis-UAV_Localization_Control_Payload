from simdkalman.primitives import update, predict_observation, predict
from filter import KalmanFilter, MovingAverage, Olympic
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statistics
import time

##########################
# Import data
##########################
fileName = '2020-06-22__02-59-06.csv' 
try:
    df = pd.read_csv(fileName, header = 0, names = ['Mode', 'Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'xGyro', 'yGyro', 'zGyro'])
    df['xGyro'] = df['xGyro'] * (180 / (1000 * np.pi))
    df['yGyro'] = df['yGyro'] * (180 / (1000 * np.pi))
    df['zGyro'] = df['zGyro'] * (180 / (1000 * np.pi))
except:
    print('Error with file.')
    exit()

##########################
# Select data set
##########################
lowRange = 0
highRange = 100

yawData = np.array(df['Yaw-Vision'])
gyroData = np.array(df['zGyro'])
timeData = np.array(df['Time'])

# yawData = yawData[lowRange:highRange]
# gyroData = gyroData[lowRange:highRange]
# timeData = timeData[lowRange:highRange]

data = np.array([np.array([yawData, gyroData]).T])

##########################
# ONLINE: Simulation
##########################
KF = KalmanFilter()
dataLog = []

for ii in range(1, data.shape[1]):
    dt = timeData[ii] - timeData[ii-1]
    dataLog.append(KF.update(dt, data[0][ii]))

##########################
# Plot the data
##########################
plt.plot(timeData, yawData, 'k-', alpha=0.2, label='Raw Yaw Data')
plt.plot(timeData, gyroData, 'b-', alpha=0.2, label='Raw Gyro Data')
plt.plot(timeData[:-1], dataLog, 'k-', label='Kalman Filter')

plt.xlabel('Time [s]')
plt.ylabel('Yaw [deg] or Yaw Rate [deg/s]')
plt.legend()
plt.show()
