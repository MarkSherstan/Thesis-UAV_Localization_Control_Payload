# Add custom packages one directory up 
import sys
sys.path.append('../')
from filter import MovingAverage, KalmanFilter1x, KalmanFilter2x, KalmanFilterNxN

# Import standard packages
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statistics
import time

##########################
# Import data
##########################
try:        
    fileName = '2020-09-11__17-43-45.csv'
    df = pd.read_csv(fileName, header = 0, names = ['Mode', 'Time', 
                            'Freq', 'time2delay', 'actualDelay',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'North-Pos', 'East-Pos', 'Down-Pos', 'Yaw-Ang',
                            'North-Vel', 'East-Vel', 'Down-Vel', 'Yaw-Vel',
                            'North-Acc', 'East-Acc', 'Down-Acc', 
                            'North-Raw', 'East-Raw', 'Down-Raw', 'Yaw-Raw',
                            'North-Dif', 'East-Dif', 'Down-Dif', 'Yaw-Dif',
                            'Landing-State', 'Q-Size'])
except:
    print('Error with file.')
    exit()


# Select data set
####################################################
lowRange = 0
highRange = 10

timeData = np.array(df['Time'])

yawData = np.array(df['Yaw-Raw'])
gyroData = np.array(df['Yaw-Vel'])

northPosData = np.array(df['North-Raw'])
eastPosData  = np.array(df['East-Raw'])
downPosData  = np.array(df['Down-Raw'])

northVelData = np.array(df['North-Vel'])
eastVelData = np.array(df['East-Vel'])
downVelData = np.array(df['Down-Vel'])

# timeData = timeData[lowRange:highRange]

# yawData = yawData[lowRange:highRange]
# gyroData = gyroData[lowRange:highRange]

# northPosData = northPosData[lowRange:highRange]
# eastPosData = eastPosData[lowRange:highRange]
# downPosData = downPosData[lowRange:highRange]

# northVelData = northVelData[lowRange:highRange]
# eastVelData = eastVelData[lowRange:highRange]
# downVelData = downVelData[lowRange:highRange]

####################################################

# ONLINE: Simulation
nKF = KalmanFilter1x(1.0, 10.0)
eKF = KalmanFilter1x(1.0, 10.0)
dKF = KalmanFilter1x(1.0, 10.0)
yKF = KalmanFilter1x(1.0, 10.0)

nKF2 = KalmanFilter2x(3.0, 5.0, 10.0)
eKF2 = KalmanFilter2x(3.0, 5.0, 10.0)
dKF2 = KalmanFilter2x(3.0, 5.0, 10.0)
yKF2 = KalmanFilter2x(3.0, 5.0, 10.0)

KF = KalmanFilterNxN(3.0, 5.0, 10.0)

nAvg = MovingAverage(5)
eAvg = MovingAverage(5)
dAvg = MovingAverage(5)
yAvg = MovingAverage(5)

nlistKF = []; nlistKF2 = []; nlistAvg = []
elistKF = []; elistKF2 = []; elistAvg = []
dlistKF = []; dlistKF2 = []; dlistAvg = []
ylistKF = []; ylistKF2 = []; ylistAvg = []

KFlist = []
timeList = []
masterList = []
tempTime = time.time()


for ii in range(1, timeData.shape[0]):
    dt = timeData[ii] - timeData[ii-1]
    
    nlistKF.append(nKF.update(dt, np.array([northPosData[ii]])))
    elistKF.append(eKF.update(dt, np.array([eastPosData[ii]])))
    dlistKF.append(dKF.update(dt, np.array([downPosData[ii]])))
    ylistKF.append(yKF.update(dt, np.array([yawData[ii]])))

    nlistKF2.append(nKF2.update(dt, np.array([northPosData[ii], northVelData[ii]]).T))
    elistKF2.append(eKF2.update(dt, np.array([eastPosData[ii], eastVelData[ii]]).T))
    dlistKF2.append(dKF2.update(dt, np.array([downPosData[ii], downVelData[ii]]).T))
    ylistKF2.append(yKF2.update(dt, np.array([yawData[ii], gyroData[ii]]).T))

    nlistAvg.append(nAvg.update(northPosData[ii]))
    elistAvg.append(eAvg.update(eastPosData[ii]))
    dlistAvg.append(dAvg.update(downPosData[ii]))
    ylistAvg.append(yAvg.update(yawData[ii]))

    KFlist.append(KF.update(dt, np.array([northPosData[ii], northVelData[ii], 
                                    eastPosData[ii], eastVelData[ii],
                                    downPosData[ii], downVelData[ii],
                                       yawData[ii], gyroData[ii]]).T))
        
# Perform comparison checks
N = []; E = []; D = []; Y = []
for ii in range(len(KFlist)):
    N.append(KFlist[ii][0])
    E.append(KFlist[ii][1])
    D.append(KFlist[ii][2])
    Y.append(KFlist[ii][3])

print('N', sum(np.array(N) - np.array(nlistKF2)))
print('E', sum(np.array(E) - np.array(elistKF2)))
print('D', sum(np.array(D) - np.array(dlistKF2)))
print('Y', sum(np.array(Y) - np.array(ylistKF2)))

##########################
# Plot the data -> Rotation
##########################
plt.plot(timeData, yawData, 'k-', alpha=0.9, label='Raw Yaw Data')
plt.plot(timeData, gyroData, 'k--', alpha=0.5, label='Raw Gyro Data')
plt.plot(timeData[:-1], ylistKF, 'r-', label='Kalman Filter')
plt.plot(timeData[:-1], ylistKF2, 'g-', label='Kalman Filter w/ Gyro')
plt.plot(timeData[:-1], ylistAvg, 'b-', label='Moving Average')

plt.xlabel('Time [s]')
plt.ylabel('Yaw [deg] \nYaw Rate [deg/s]')
plt.legend()
plt.show()

##########################
# Plot the data -> Position
##########################
plt.plot(timeData, northPosData, 'k-',   alpha=0.9, label='Raw N-Pos Data')
plt.plot(timeData, northVelData, 'r--',  alpha=0.3, label='Raw N-Vel Data')
plt.plot(timeData[:-1], nlistKF, 'r-',   label='Kalman Filter')
plt.plot(timeData[:-1], nlistKF2, 'r--', label='Kalman Filter w/ Vel')
plt.plot(timeData[:-1], nlistAvg, 'r.-', label='Moving Average')

plt.plot(timeData, eastPosData, 'k-',    alpha=0.9, label='Raw E-Pos Data')
plt.plot(timeData, eastVelData, 'g--',   alpha=0.3, label='Raw E-Vel Data')
plt.plot(timeData[:-1], elistKF, 'g-',   label='Kalman Filter')
plt.plot(timeData[:-1], elistKF2, 'g--', label='Kalman Filter w/ Vel')
plt.plot(timeData[:-1], elistAvg, 'g.-', label='Moving Average')

plt.plot(timeData, downPosData, 'k-',    alpha=0.9, label='Raw D-Pos Data')
plt.plot(timeData, downVelData, 'b--',   alpha=0.3, label='Raw D-Vel Data')
plt.plot(timeData[:-1], dlistKF, 'b-',   label='Kalman Filter')
plt.plot(timeData[:-1], dlistKF2, 'b--', label='Kalman Filter w/ Vel')
plt.plot(timeData[:-1], dlistAvg, 'b.-', label='Moving Average')

plt.xlabel('Time [s]')
plt.ylabel('Position [cm] \nVelocity [cm/s]')
plt.legend()
plt.show()
