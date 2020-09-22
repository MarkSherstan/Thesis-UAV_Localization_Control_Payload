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

    df.plot(kind='line', x='Time', y='Roll-UAV',  color='tab:orange', alpha=0.5, style='--', ax=ax0)
    df.plot(kind='line', x='Time', y='Pitch-UAV', color='tab:purple', alpha=0.5, style='--', ax=ax0)

    df.plot(kind='line', x='Time', y='Roll-Control',  color='tab:orange', style='-', ax=ax0)
    df.plot(kind='line', x='Time', y='Pitch-Control', color='tab:purple', style='-', ax=ax0)

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

    df.plot(kind='line', x='Time', y='North-Pos', color='tab:purple', style='-',  ax=ax3)
    df.plot(kind='line', x='Time', y='East-Pos',  color='tab:orange', style='-',  ax=ax3)
    df.plot(kind='line', x='Time', y='Down-Pos',  color='tab:green', style='-',  ax=ax3)

    df.plot(kind='line', x='Time', y='North-Desired', color='tab:purple', style='--',  ax=ax3)
    df.plot(kind='line', x='Time', y='East-Desired',  color='tab:orange', style='--',  ax=ax3)
    df.plot(kind='line', x='Time', y='Down-Desired',  color='tab:green',  style='--', ax=ax3)

    ax3.set_title('NED Position', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time [s]', fontweight='bold')
    ax3.set_ylabel('Position [cm]', fontweight='bold')

    ax3.legend(['N', 'E', 'D'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
    
    # Thrust
    plt.subplot(2, 2, 4)
    ax4 = plt.gca()

    df.plot(kind='line', x='Time', y='Thrust-Control', color='tab:green', style='-', ax=ax4)

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
    print('NED Difference Stats: ')
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
    df.plot(kind='scatter', x='North-One', y='North-Two', color='tab:purple', ax=ax)
    df.plot(kind='scatter', x='East-One',  y='East-Two',  color='tab:orange', ax=ax)
    df.plot(kind='scatter', x='Down-One',  y='Down-Two',  color='tab:green',  ax=ax)
    df.plot(kind='scatter', x='Yaw-One',   y='Yaw-Two',   color='tab:blue',   ax=ax)
    
    ax.set_title('Difference Scatter', fontsize=14, fontweight='bold')
    ax.set_xlabel('Camera One [Cm | Degree]', fontweight='bold')
    ax.set_ylabel('Camera Two [Cm | Degree]',  fontweight='bold')
    
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

    nSumDif = sum(np.array(N) - np.array(nlistKF2))
    eSumDif = sum(np.array(E) - np.array(elistKF2))
    dSumDif = sum(np.array(D) - np.array(dlistKF2))
    ySumDif = sum(np.array(Y) - np.array(ylistKF2))
    sumDifTot = nSumDif + eSumDif + dSumDif + ySumDif
    
    if (sumDifTot != 0):
        print('N', nSumDif)
        print('E', eSumDif)
        print('D', dSumDif)
        print('Y', ySumDif)

    ##########################
    # Plot the data -> Rotation
    ##########################
    fig = plt.figure(figsize=(11, 7), dpi=100)

    plt.plot(timeData, yawData,       'k-',  alpha=0.9, label='Raw Yaw Data')
    plt.plot(timeData, gyroData,      color='tab:blue', linestyle='--', alpha=0.3, label='Raw Gyro Data')
    plt.plot(timeData[:-1], ylistKF,  color='tab:blue', linestyle='-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], Y,        color='tab:blue', linestyle='--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], ylistAvg, color='tab:blue', linestyle='-.', label='Moving Average')

    plt.title('Yaw Kalman Filter', fontsize=14, fontweight='bold')
    plt.xlabel('Time [s]', fontweight='bold')
    plt.ylabel('Yaw [deg]\nYaw Rate [deg/s]', fontweight='bold')
    plt.legend()

    ##########################
    # Plot the data -> Position
    ##########################
    fig = plt.figure(figsize=(11, 7), dpi=100)

    plt.plot(timeData, northPosData,  'k-',  alpha=0.9, label='Raw N-Pos Data')
    plt.plot(timeData, northVelData,  color='tab:purple', linestyle='--', alpha=0.3, label='Raw N-Vel Data')
    plt.plot(timeData[:-1], nlistKF,  color='tab:purple', linestyle='-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], N,        color='tab:purple', linestyle='--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], nlistAvg, color='tab:purple', linestyle='-.', label='Moving Average')

    plt.plot(timeData, eastPosData,   'k-',  alpha=0.9, label='Raw E-Pos Data')
    plt.plot(timeData, eastVelData,   color='tab:orange', linestyle='--', alpha=0.3, label='Raw E-Vel Data')
    plt.plot(timeData[:-1], elistKF,  color='tab:orange', linestyle='-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], E,        color='tab:orange', linestyle='--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], elistAvg, color='tab:orange', linestyle='-.', label='Moving Average')

    plt.plot(timeData, downPosData,   'k-',  alpha=0.9, label='Raw D-Pos Data')
    plt.plot(timeData, downVelData,   color='tab:green', linestyle='--', alpha=0.3, label='Raw D-Vel Data')
    plt.plot(timeData[:-1], dlistKF,  color='tab:green', linestyle='-',  label='Kalman Filter Pos')
    plt.plot(timeData[:-1], D,        color='tab:green', linestyle='--', label='Kalman Filter Pos and Vel')
    plt.plot(timeData[:-1], dlistAvg, color='tab:green', linestyle='-.', label='Moving Average')

    plt.title('NED Kalman Filter', fontsize=14, fontweight='bold')
    plt.xlabel('Time [s]', fontweight='bold')
    plt.ylabel('Position [cm]\nVelocity [cm/s]', fontweight='bold')
    plt.legend()

    # Time Plot
    fig = plt.figure(figsize=(8, 4), dpi=100)
    ax = plt.gca()
    
    # Plot data
    df.plot(kind='line', x='Kalman-Time', y='Kalman-Dt', style='.', color='k', ax=ax)

    # Format
    title = 'T265 Time Delta\n' + '{:<4.3f} +/- {:<0.3f} '.format(df['Kalman-Dt'].mean(), df['Kalman-Dt'].std())
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Time [s]', fontweight='bold')
    ax.set_ylabel('dt [s]',  fontweight='bold')
    ax.get_legend().remove()
    
def gainTune(df, fileName, saveFlag):
    # Find Autonomous Sections
    df['modeSwitch'] =  df['Mode'] == 'GUIDED_NOGPS'

    idx = []
    for ii in range(df.shape[0]-1):
        if (df['modeSwitch'][ii+1] == df['modeSwitch'][ii]):
            pass
        else:
            idx.append(ii)

    # Master
    fig = plt.figure(figsize=(13, 7), dpi=100)

    ################################################################################################

    # North - Pos 
    plt.subplot(2, 4, 1)
    ax1 = plt.gca()

    df.plot(kind='line', x='Time', y='actualN',  color='tab:purple', style='-',  ax=ax1)
    df.plot(kind='line', x='Time', y='desiredN', color='tab:purple', style='--', ax=ax1)
    df.plot(kind='line', x='Time', y='errorN',   color='tab:purple', alpha=0.25, style='-', ax=ax1)

    tempTitle = 'Kp: ' + str(df['N-kp'][0]) + ' Ki: ' + str(df['N-ki'][0]) + ' Kd: ' + str(df['N-kd'][0])
    ax1.set_title('North\n' + tempTitle, fontsize=10, fontweight='bold')
    ax1.set_ylabel('Position [cm]', fontweight='bold')
    ax1.set_xlabel('')
    ax1.legend(['Actual', 'Desired', 'Error'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

    # North - Control 
    plt.subplot(2, 4, 5)
    ax2 = plt.gca()

    df.plot(kind='line', x='Time', y='N-P',   color='k', style='-',  ax=ax2)
    df.plot(kind='line', x='Time', y='N-I',   color='k', style='--', ax=ax2)
    df.plot(kind='line', x='Time', y='N-D',   color='k', style=':',  ax=ax2)
    df.plot(kind='line', x='Time', y='N-PID', color='k', alpha=0.5, style='-', ax=ax2)

    ax2.set_xlabel('Time [s]', fontweight='bold')
    ax2.set_ylabel('Angle [deg]', fontweight='bold')
    ax2.set_ylim([-5,5])

    ax2.legend(['P', 'I', 'D', 'PID'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
        
        
    ################################################################################################


    # East - Pos 
    plt.subplot(2, 4, 2)
    ax1 = plt.gca()

    df.plot(kind='line', x='Time', y='actualE',  color='tab:orange', style='-',  ax=ax1)
    df.plot(kind='line', x='Time', y='desiredE', color='tab:orange', style='--', ax=ax1)
    df.plot(kind='line', x='Time', y='errorE',   color='tab:orange', alpha=0.25, style='-', ax=ax1)

    tempTitle = 'Kp: ' + str(df['E-kp'][0]) + ' Ki: ' + str(df['E-ki'][0]) + ' Kd: ' + str(df['E-kd'][0])
    ax1.set_title('East\n' + tempTitle, fontsize=10, fontweight='bold')
    ax1.set_xlabel('')
    ax1.legend(['Actual', 'Desired', 'Error'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

    # East - Control 
    plt.subplot(2, 4, 6)
    ax2 = plt.gca()

    df.plot(kind='line', x='Time', y='E-P',   color='k', style='-',  ax=ax2)
    df.plot(kind='line', x='Time', y='E-I',   color='k', style='--', ax=ax2)
    df.plot(kind='line', x='Time', y='E-D',   color='k', style=':',  ax=ax2)
    df.plot(kind='line', x='Time', y='E-PID', color='k', alpha=0.5, style='-', ax=ax2)

    ax2.set_xlabel('Time [s]', fontweight='bold')
    ax2.set_ylim([-5,5])

    ax2.legend(['P', 'I', 'D', 'PID'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)


    ################################################################################################


    # Down - Pos 
    plt.subplot(2, 4, 3)
    ax1 = plt.gca()

    df.plot(kind='line', x='Time', y='actualD',  color='tab:green', style='-',  ax=ax1)
    df.plot(kind='line', x='Time', y='desiredD', color='tab:green', style='--', ax=ax1)
    df.plot(kind='line', x='Time', y='errorD',   color='tab:green', alpha=0.25, style='-', ax=ax1)

    tempTitle = 'Kp: ' + str(df['D-kp'][0]) + ' Ki: ' + str(df['D-ki'][0]) + ' Kd: ' + str(df['D-kd'][0])
    ax1.set_title('Down\n' + tempTitle, fontsize=10, fontweight='bold')
    ax1.set_xlabel('')
    ax1.legend(['Actual', 'Desired', 'Error'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

    # Down - Control 
    plt.subplot(2, 4, 7)
    ax2 = plt.gca()

    df.plot(kind='line', x='Time', y='D-P',   color='k', style='-',  ax=ax2)
    df.plot(kind='line', x='Time', y='D-I',   color='k', style='--', ax=ax2)
    df.plot(kind='line', x='Time', y='D-D',   color='k', style=':',  ax=ax2)
    df.plot(kind='line', x='Time', y='D-PID', color='k', alpha=0.5, style='-', ax=ax2)

    ax2.set_xlabel('Time [s]', fontweight='bold')

    ax2.legend(['P', 'I', 'D', 'PID'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)


    ################################################################################################


    # Yaw - Ang 
    plt.subplot(2, 4, 4)
    ax1 = plt.gca()

    df.plot(kind='line', x='Time', y='actualY',  color='tab:blue', style='-',  ax=ax1)
    df.plot(kind='line', x='Time', y='desiredY', color='tab:blue', style='--', ax=ax1)
    df.plot(kind='line', x='Time', y='errorY',   color='tab:blue', alpha=0.25, style='-', ax=ax1)

    tempTitle = 'Kp: ' + str(df['Y-kp'][0]) + ' Ki: ' + str(df['Y-ki'][0]) + ' Kd: ' + str(df['Y-kd'][0])
    ax1.set_title('Yaw\n' + tempTitle, fontsize=10, fontweight='bold')
    ax1.set_xlabel('')
    ax1.legend(['Actual', 'Desired', 'Error'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

    # Yaw - Control 
    plt.subplot(2, 4, 8)
    ax2 = plt.gca()

    df.plot(kind='line', x='Time', y='Y-P',   color='k', style='-',  ax=ax2)
    df.plot(kind='line', x='Time', y='Y-I',   color='k', style='--', ax=ax2)
    df.plot(kind='line', x='Time', y='Y-D',   color='k', style=':',  ax=ax2)
    df.plot(kind='line', x='Time', y='Y-PID', color='k', alpha=0.5, style='-', ax=ax2)

    ax2.set_xlabel('Time [s]', fontweight='bold')
    ax2.set_ylim([-10,10])
    ax2.legend(['P', 'I', 'D', 'PID'])

    for ii in range(0,len(idx),2):
        plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

    ################################################################################################

    # Save the figure    
    if saveFlag is True:
        fig.savefig(str(fileName)+'-GAIN-TUNE.png', dpi=fig.dpi)

    ################################################################################################

    # Time Plot
    fig = plt.figure(figsize=(8, 4), dpi=100)
    ax = plt.gca()
    
    # Plot data
    df.plot(kind='line', x='Time', y='dt', style='.', color='k', ax=ax)

    # Format
    title = 'Controller Time Delta\n' + '{:<4.3f} +/- {:<0.3f} '.format(df['dt'].mean(), df['dt'].std())
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('Time [s]', fontweight='bold')
    ax.set_ylabel('dt [s]',  fontweight='bold')
    ax.get_legend().remove()
 
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
# General(df.copy(), fileName, saveFlag=False)
# FreqSleep(df.copy(), fileName, saveFlag=False)
# DiffState(df.copy(), fileName, saveFlag=False)
# KalmanTune(df.copy())
gainTune(df2.copy(), fileName, saveFlag=False)
plt.show()
