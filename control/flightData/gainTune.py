import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse

########################
# Argparse
# Example use: python plotter.py --input "flight1.csv"
########################
parser = argparse.ArgumentParser()
parser.add_argument("--input", help = "input filename")
args = parser.parse_args()
fileName = args.input

########################
# Prepare CSV
########################
try:
    df = pd.read_csv(fileName, header = 0, names = ['D-kp', 'D-ki', 'D-kd', 'D-I-Tot', 'D-P', 'D-I', 'D-D', 'D-PID',
                                          'E-kp', 'E-ki', 'E-kd', 'E-I-Tot', 'E-P', 'E-I', 'E-D', 'E-PID',
                                          'N-kp', 'N-ki', 'N-kd', 'N-I-Tot', 'N-P', 'N-I', 'N-D', 'N-PID',
                                          'Y-kp', 'Y-ki', 'Y-kd', 'Y-I-Tot', 'Y-P', 'Y-I', 'Y-D', 'Y-PID',
                                          'errorN', 'desiredN', 'actualN',
                                          'errorE', 'desiredE', 'actualE',
                                          'errorD', 'desiredD', 'actualD',
                                          'errorY', 'desiredY', 'actualY',
                                          'roll', 'pitch', 'yaw-rate', 'thrust', 'dt', 'Time', 'Mode'])
except:
    print('Error with file.')
    exit()

########################
# Find Autonomous Sections
########################
df['modeSwitch'] =  df['Mode'] == 'GUIDED_NOGPS'

idx = [0]
for ii in range(df.shape[0]-1):
    if (df['modeSwitch'][ii+1] == df['modeSwitch'][ii]):
        pass
    else:
        idx.append(ii)

########################
# Master
########################
fig = plt.figure()

########################
# North - Pos 
########################
plt.subplot(2, 3, 1)
ax1 = plt.gca()

df.plot(kind='line', x='Time', y='actualN',  color='#700CBC', style='-',  ax=ax1)
df.plot(kind='line', x='Time', y='desiredN', color='#700CBC', style='--', ax=ax1)
df.plot(kind='line', x='Time', y='errorN',   color='#700CBC', style='.-', ax=ax1)

ax1.set_title('North', fontsize=14, fontweight='bold')
ax1.set_ylabel('Position [cm]', fontweight='bold')

ax1.legend(['Actual', 'Desired', 'Error'])

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

########################
# North - Control 
########################
plt.subplot(2, 3, 4)
ax2 = plt.gca()

df.plot(kind='line', x='Time', y='N-P',   color='#700CBC', style='-',  ax=ax2)
df.plot(kind='line', x='Time', y='N-I',   color='#700CBC', style='--', ax=ax2)
df.plot(kind='line', x='Time', y='N-D',   color='#700CBC', style='.-', ax=ax2)
df.plot(kind='line', x='Time', y='N-PID', color='#700CBC', style=':',  ax=ax2)

ax2.set_xlabel('Time [s]', fontweight='bold')
ax2.set_ylabel('Angle [deg]', fontweight='bold')

ax2.legend(['P', 'I', 'D', 'PID'])

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
 
######
plt.subplot(2, 3, 3)
plt.subplot(2, 3, 2)
plt.subplot(2, 3, 5)
plt.subplot(2, 3, 6)

########################
# Show the plots
########################
plt.show()
fig.savefig(str(fileName).replace('.csv','')+'.png', dpi=fig.dpi)
