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

################################################################################################

# North - Pos 
plt.subplot(2, 3, 1)
ax1 = plt.gca()

df.plot(kind='line', x='Time', y='actualN',  color='tab:purple', style='-',  ax=ax1)
df.plot(kind='line', x='Time', y='desiredN', color='tab:purple', style='--', ax=ax1)
df.plot(kind='line', x='Time', y='errorN',   color='tab:purple', alpha=0.25, style='-', ax=ax1)

ax1.set_title('North', fontsize=14, fontweight='bold')
ax1.set_ylabel('Position [cm]', fontweight='bold')
ax1.set_xlabel('')
ax1.legend(['Actual', 'Desired', 'Error'])

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)


# North - Control 
plt.subplot(2, 3, 4)
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
plt.subplot(2, 3, 2)
ax1 = plt.gca()

df.plot(kind='line', x='Time', y='actualE',  color='tab:orange', style='-',  ax=ax1)
df.plot(kind='line', x='Time', y='desiredE', color='tab:orange', style='--', ax=ax1)
df.plot(kind='line', x='Time', y='errorE',   color='tab:orange', alpha=0.25, style='-', ax=ax1)

ax1.set_title('East', fontsize=14, fontweight='bold')
ax1.set_xlabel('')
ax1.legend(['Actual', 'Desired', 'Error'])

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)


# East - Control 
plt.subplot(2, 3, 5)
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
plt.subplot(2, 3, 3)
ax1 = plt.gca()

df.plot(kind='line', x='Time', y='actualD',  color='tab:green', style='-',  ax=ax1)
df.plot(kind='line', x='Time', y='desiredD', color='tab:green', style='--', ax=ax1)
df.plot(kind='line', x='Time', y='errorD',   color='tab:green', alpha=0.25, style='-', ax=ax1)

ax1.set_title('Down', fontsize=14, fontweight='bold')
ax1.set_xlabel('')
ax1.legend(['Actual', 'Desired', 'Error'])

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

# Control 
plt.subplot(2, 3, 6)
ax2 = plt.gca()

df.plot(kind='line', x='Time', y='D-P',   color='k', style='-',  ax=ax2)
df.plot(kind='line', x='Time', y='D-I',   color='k', style='--', ax=ax2)
df.plot(kind='line', x='Time', y='D-D',   color='k', style=':',  ax=ax2)
df.plot(kind='line', x='Time', y='D-PID', color='k', alpha=0.5, style='-', ax=ax2)

ax2.set_xlabel('Time [s]', fontweight='bold')

ax2.legend(['P', 'I', 'D', 'PID'])

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)

########################
# Show the plots
########################
plt.show()
fig.savefig(str(fileName).replace('.csv','')+'.png', dpi=fig.dpi)
