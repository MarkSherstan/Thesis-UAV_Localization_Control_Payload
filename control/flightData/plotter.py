import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse

def General(df, fileName, saveFlag):
    # Find autonomous sections
    df['modeSwitch'] =  df['Mode'] == 'GUIDED_NOGPS'

    idx = []
    for ii in range(df.shape[0]-1):
        if (df['modeSwitch'][ii+1] == df['modeSwitch'][ii]):
            pass
        else:
            idx.append(ii)

    # Master
    fig = plt.figure(figsize=(11, 7), dpi=100)

    # Roll and Pitch
    plt.subplot(2, 2, 1)
    ax0 = plt.gca()

    df.plot(kind='line', x='Time', y='Roll-UAV',  color='#FB8604', alpha=0.5, style='-', ax=ax0)
    df.plot(kind='line', x='Time', y='Pitch-UAV', color='#700CBC', alpha=0.5, style='-', ax=ax0)

    df.plot(kind='line', x='Time', y='Roll-Control',  color='#FB8604', style='-', ax=ax0)
    df.plot(kind='line', x='Time', y='Pitch-Control', color='#700CBC', style='-', ax=ax0)

    ax0.set_title('Roll & Pitch Control', fontsize=14, fontweight='bold')
    ax0.set_xlabel('')
    ax0.set_ylabel('Angle [deg]', fontweight='bold')

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

    # Yaw
    plt.subplot(2, 2, 2)
    ax1 = plt.gca()

    df.plot(kind='line', x='Time', y='Yaw-Ang', color='tab:blue', style='-', ax=ax1)
    df.plot(kind='line', x='Time', y='Yaw-Control', color='tab:blue', style='--', ax=ax1)

    ax1.set_title('Yaw Control', fontsize=14, fontweight='bold')
    ax1.set_xlabel('')
    ax1.set_ylabel('Angle [Deg or Deg/s]', fontweight='bold')

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
    
    # North East
    plt.subplot(2, 2, 3)
    ax3 = plt.gca()

    df.plot(kind='line', x='Time', y='North-Pos', color='#700CBC', style='-',  ax=ax3)
    df.plot(kind='line', x='Time', y='East-Pos',  color='#FB8604', style='-',  ax=ax3)
    df.plot(kind='line', x='Time', y='Down-Pos',  color='#7FBD32', style='-',  ax=ax3)

    df.plot(kind='line', x='Time', y='North-Desired', color='#700CBC', style='--',  ax=ax3)
    df.plot(kind='line', x='Time', y='East-Desired',  color='#FB8604', style='--',  ax=ax3)
    df.plot(kind='line', x='Time', y='Down-Desired',  color='#7FBD32',  style='--', ax=ax3)

    ax3.set_title('NED Position', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time [s]', fontweight='bold')
    ax3.set_ylabel('Position [cm]', fontweight='bold')

    ax3.legend(['N', 'E', 'D'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
    
    # Thrust
    plt.subplot(2, 2, 4)
    ax4 = plt.gca()

    df.plot(kind='line', x='Time', y='Thrust-Control', color='#7FBD32', style='-', ax=ax4)

    ax4.set_title('Thrust Control', fontsize=14, fontweight='bold')
    ax4.set_xlabel('Time [s]', fontweight='bold')
    ax4.set_ylabel('Normalized Thrust Command', fontweight='bold')
    ax4.get_legend().remove()

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
    
    # Save the figure    
    if saveFlag is True:
        fig.savefig(str(fileName).replace('.csv','')+'-GENERAL.png', dpi=fig.dpi)

def FreqSleep(df, fileName, saveFlag): 
    # Create a figure
    fig = plt.figure(figsize=(12, 6), dpi=100)

    # Create subplot: Frequency
    plt.subplot(1, 2, 1)
    ax1 = plt.gca()

    # Plot data
    df.plot(kind='line', x='Time', y='Freq',  color='k',  style='.',  ax=ax1)

    # Format figure
    title = 'Sampling Frequency\n' + '{:<4.3f} +/- {:<0.3f} '.format(df['Freq'].mean(), df['Freq'].std())
    ax1.set_title(title, fontsize=14, fontweight='bold')
    ax1.set_xlabel('Time [s]', fontweight='bold')
    ax1.set_ylabel('Frequency [Hz]', fontweight='bold')
    ax1.get_legend().remove()


    # Create subplot: Sleep request
    plt.subplot(1, 2, 2)
    ax0 = plt.gca()

    # Plot data
    df['time2delay'] = df['time2delay'] * 1000
    df['actualDelay'] = df['actualDelay'] * 1000

    df.plot(kind='line', x='time2delay', y='actualDelay',  color='r',  style='+',  ax=ax0)

    # Figure formatting
    ax0.set_title('Sleep Request', fontsize=14, fontweight='bold')
    ax0.set_xlabel('Requested Sleep Time [ms]', fontweight='bold')
    ax0.set_ylabel('Actual Sleep Time [ms]', fontweight='bold')
    ax0.get_legend().remove()

    # Add grid
    plt.grid()

    # Save the figure 
    if saveFlag is True:
        fig.savefig(str(fileName).replace('.csv','')+'-FREQ_SLEEP.png', dpi=fig.dpi)

def DiffState(df, fileName, saveFlag):
    # Master
    fig = plt.figure(figsize=(12, 6), dpi=100)

    # Create subplot: NEDY Difference 
    plt.subplot(1, 2, 1)
    ax = plt.gca()

    # Plot data
    df.plot(kind='line', x='Time', y='North-Dif', color='tab:purple', style='-', ax=ax)
    df.plot(kind='line', x='Time', y='East-Dif',  color='tab:orange', style='-', ax=ax)
    df.plot(kind='line', x='Time', y='Down-Dif',  color='tab:green',  style='-', ax=ax)
    df.plot(kind='line', x='Time', y='Yaw-Dif',   color='tab:blue',   style='-', ax=ax)

    # Format figure
    ax.set_title('Difference Comparison', fontsize=14, fontweight='bold')
    ax.set_xlabel('Time [s]', fontweight='bold')
    ax.set_ylabel('Difference [Cm]', fontweight='bold')

    # Comparison 
    print('Difference Stats: ')
    print('   North: ' + '{:<4.2f} +/- {:<0.2f} '.format(df['North-Dif'].mean(), df['North-Dif'].std()))
    print('   East: ' + '{:<4.2f} +/- {:<0.2f} '.format(df['East-Dif'].mean(), df['East-Dif'].std()))
    print('   Down: ' + '{:<4.2f} +/- {:<0.2f} '.format(df['Down-Dif'].mean(), df['Down-Dif'].std()))
    print('   Yaw: ' + '{:<4.2f} +/- {:<0.2f} '.format(df['Yaw-Dif'].mean(), df['Yaw-Dif'].std()))


    # Create subplot: Land and Queue
    plt.subplot(1, 2, 2)
    ax = plt.gca()

    # Plot data
    df['Landing-State'] = (df['Landing-State'] == True).astype(int)
    df.plot(kind='line', x='Time', y='Landing-State', color='k', style='-', ax=ax)
    df.plot(kind='line', x='Time', y='Q-Size', alpha=0.3, color='r', style='-', ax=ax)

    # Format Figure
    ax.set_title('State and Queue Check', fontsize=14, fontweight='bold')
    ax.set_xlabel('Time [s]', fontweight='bold')
    ax.set_ylabel('State [ ]', fontweight='bold')

    # Save the figure 
    if saveFlag is True:
        fig.savefig(str(fileName).replace('.csv','')+'-FREQ_SLEEP.png', dpi=fig.dpi)


    # Scatter Plot: Raw Pos Data
    fig = plt.figure(figsize=(12, 6), dpi=100)
    ax = plt.gca()
    
    # Plot data
    df.plot(kind='scatter', x='North-One', y='North-Two', color='tab:purple')
    df.plot(kind='scatter', x='East-One',  y='East-Two',  color='tab:orange')
    df.plot(kind='scatter', x='Down-One',  y='Down-Two',  color='tab:green')
    df.plot(kind='scatter', x='Yaw-One',   y='Yaw-Two',   color='tab:blue')
    
    ax.set_title('Difference Scatter', fontsize=14, fontweight='bold')
    ax.set_xlabel('Camera One [Cm | Degree]', fontweight='bold')
    ax.set_ylabel('Camera Two [Cm | Degree',  fontweight='bold')
    
    ax.legend(['N', 'E', 'D', 'Y'])
    
    if saveFlag is True:
        fig.savefig(str(fileName).replace('.csv','')+'-FREQ_SLEEP-II.png', dpi=fig.dpi)

def KalmanTune(df):
    # Add custom packages one directory up 
    import sys
    sys.path.append('../')
    from filter import MovingAverage, KalmanFilter1x, KalmanFilter2x, KalmanFilterNxN
    
    # Select data set
    ####################################################
    lowRange = 0
    highRange = 10

    timeData = np.array(df['Kalman-Time'])
    deltaT   = np.array(df['Kalman-Dt'])

    yawData = np.array(df['Yaw-Avg'])
    gyroData = np.array(df['Yaw-Vel'])

    northPosData = np.array(df['North-Avg'])
    eastPosData  = np.array(df['East-Avg'])
    downPosData  = np.array(df['Down-Avg'])

    northVelData = np.array(df['North-Vel'])
    eastVelData = np.array(df['East-Vel'])
    downVelData = np.array(df['Down-Vel'])

    # timeData = timeData[lowRange:highRange]
    # deltaT   = deltaT[lowRange:highRange]
    
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

    for ii in range(1, timeData.shape[0]):
        dt = deltaT[ii]
        
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
    fig = plt.figure(figsize=(11, 7), dpi=100)

    plt.plot(timeData, yawData,       'k-',  alpha=0.9, label='Raw Yaw Data')
    plt.plot(timeData, gyroData,      'k--', alpha=0.5, label='Raw Gyro Data')
    plt.plot(timeData[:-1], ylistKF,  'r-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], Y,        'g-',  label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], ylistAvg, 'b-',  label='Moving Average')

    plt.xlabel('Time [s]')
    plt.ylabel('Yaw [deg] \nYaw Rate [deg/s]')
    plt.legend()

    ##########################
    # Plot the data -> Position
    ##########################
    fig = plt.figure(figsize=(11, 7), dpi=100)

    plt.plot(timeData, northPosData,  'k-',  alpha=0.9, label='Raw N-Pos Data')
    plt.plot(timeData, northVelData,  'r--', alpha=0.3, label='Raw N-Vel Data')
    plt.plot(timeData[:-1], nlistKF,  'r-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], N,        'r--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], nlistAvg, 'r.-', label='Moving Average')

    plt.plot(timeData, eastPosData,   'k-',  alpha=0.9, label='Raw E-Pos Data')
    plt.plot(timeData, eastVelData,   'g--', alpha=0.3, label='Raw E-Vel Data')
    plt.plot(timeData[:-1], elistKF,  'g-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], E,        'g--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], elistAvg, 'g.-', label='Moving Average')

    plt.plot(timeData, downPosData,   'k-',  alpha=0.9, label='Raw D-Pos Data')
    plt.plot(timeData, downVelData,   'b--', alpha=0.3, label='Raw D-Vel Data')
    plt.plot(timeData[:-1], dlistKF,  'b-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], D,        'b--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], dlistAvg, 'b.-', label='Moving Average')

    plt.xlabel('Time [s]')
    plt.ylabel('Position [cm] \nVelocity [cm/s]')
    plt.legend()

    # Time Plot
    fig = plt.figure(figsize=(10, 5), dpi=100)
    ax = plt.gca()
    
    # Plot data
    df.plot(kind='line', x='Kalman-Time', y='Kalman-Dt', color='k')

    ax.set_title('Kalman Time Stabilization', fontsize=14, fontweight='bold')
    ax.set_xlabel('Time [s]', fontweight='bold')
    ax.set_ylabel('dt [s]',  fontweight='bold')
    
    
########################
# Argparse
# Example use: python plotter.py --input "flight1.csv"
########################
parser = argparse.ArgumentParser()
parser.add_argument("--input", help = "input date time")
args = parser.parse_args()
fileName = args.input
mainFile = fileName + '--MAIN.csv'
controlFile = fileName + '--CONTROL.csv'


# Prepare CSV
try:
    df = pd.read_csv(mainFile, header=0, names=['Mode', 'Time', 
                            'Freq', 'time2delay', 'actualDelay',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'North-Pos', 'East-Pos', 'Down-Pos', 'Yaw-Ang',
                            'North-Vel', 'East-Vel', 'Down-Vel', 'Yaw-Vel',
                            'North-Acc', 'East-Acc', 'Down-Acc', 
                            'North-Avg', 'East-Avg', 'Down-Avg', 'Yaw-Avg',
                            'North-Dif', 'East-Dif', 'Down-Dif', 'Yaw-Dif',
                            'North-One', 'East-One', 'Down-One', 'Yaw-One',
                            'North-Two', 'East-Two', 'Down-Two', 'Yaw-Two',
                            'Landing-State', 'Q-Size', 'Kalman-Time', 'Kalman-Dt'])
    
    df2 = pd.read_csv(controlFile, header=0, names=['E-kp', 'E-ki', 'E-kd', 'E-I-Tot', 'E-P', 'E-I', 'E-D', 'E-PID',
                                                    'N-kp', 'N-ki', 'N-kd', 'N-I-Tot', 'N-P', 'N-I', 'N-D', 'N-PID',
                                                    'Y-kp', 'Y-ki', 'Y-kd', 'Y-I-Tot', 'Y-P', 'Y-I', 'Y-D', 'Y-PID',
                                                    'D-kp', 'D-ki', 'D-kd', 'D-I-Tot', 'D-P', 'D-I', 'D-D', 'D-PID',
                                                    'errorN', 'desiredN', 'actualN',
                                                    'errorE', 'desiredE', 'actualE',
                                                    'errorD', 'desiredD', 'actualD',
                                                    'errorY', 'desiredY', 'actualY',
                                                    'roll', 'pitch', 'yaw-rate', 'thrust', 'dt', 'Time', 'Mode'])                   
except:
    print('Error with file.')
    exit()


# Cut time
# start = 25  #df['Time'].iloc[0]
# end   = 120 #df['Time'].iloc[-1]
# df = df[(df['Time'] >= start) & (df['Time'] <= end)]

# Plotting options
General(df.copy(), fileName, saveFlag=False)
FreqSleep(df.copy(), fileName, saveFlag=False)
DiffState(df.copy(), fileName, saveFlag=False)
# KalmanTune(df.copy())
plt.show()
