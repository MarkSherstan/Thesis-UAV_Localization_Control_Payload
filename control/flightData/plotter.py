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
    df = pd.read_csv(fileName, header = 0, names = ['Mode', 'Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control'])
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
# Roll and Pitch
########################
plt.subplot(2, 2, 1)
ax0 = plt.gca()

df.plot(kind='line', x='Time', y='Roll-Control',  color='#FB8604', style='-', ax=ax0)
df.plot(kind='line', x='Time', y='Pitch-Control', color='#700CBC', style='-', ax=ax0)

df.plot(kind='line', x='Time', y='Roll-UAV',  color='#FB8604', style='--',  ax=ax0)
df.plot(kind='line', x='Time', y='Pitch-UAV', color='#700CBC', style='--',  ax=ax0)

ax0.set_title('Roll & Pitch Control', fontsize=14, fontweight='bold')
ax0.set_xlabel('Time [s]', fontweight='bold')
ax0.set_ylabel('Angle [deg]', fontweight='bold')

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
    # print(idx[ii], idx[ii+1])
    # print(df['Time'][idx[ii]], df['Time'][idx[ii+1]])

########################
# Yaw
########################
plt.subplot(2, 2, 2)
ax1 = plt.gca()

df.plot(kind='line', x='Time', y='Yaw-Vision', color='tab:blue', style='-', ax=ax1)
df.plot(kind='line', x='Time', y='Yaw-Control', color='tab:blue', style='--', ax=ax1)

ax1.set_title('Yaw Control', fontsize=14, fontweight='bold')
ax1.set_xlabel('Time [s]', fontweight='bold')
ax1.set_ylabel('Angle [Deg or Deg/s]', fontweight='bold')

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
 
########################
# North East
########################
plt.subplot(2, 2, 3)
ax3 = plt.gca()

df.plot(kind='line', x='Time', y='North-Vision', color='#700CBC', style='-',  ax=ax3)
df.plot(kind='line', x='Time', y='East-Vision',  color='#FB8604', style='-',  ax=ax3)
df.plot(kind='line', x='Time', y='Down-Vision',  color='#7FBD32', style='-',  ax=ax3)

df.plot(kind='line', x='Time', y='North-Desired', color='#700CBC', style='--',  ax=ax3)
df.plot(kind='line', x='Time', y='East-Desired',  color='#FB8604', style='--',  ax=ax3)
df.plot(kind='line', x='Time', y='Down-Desired',  color='#7FBD32',  style='--',  ax=ax3)

ax3.set_title('NED Position', fontsize=14, fontweight='bold')
ax3.set_xlabel('Time [s]', fontweight='bold')
ax3.set_ylabel('Position [cm]', fontweight='bold')

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
 
########################
# Thrust
########################
plt.subplot(2, 2, 4)
ax4 = plt.gca()

df.plot(kind='line', x='Time', y='Thrust-Control', color='#7FBD32', style='-', ax=ax4)

ax4.set_title('Thrust Control', fontsize=14, fontweight='bold')
ax4.set_xlabel('Time [s]', fontweight='bold')
ax4.set_ylabel('Normalized Thrust Command', fontweight='bold')
ax4.get_legend().remove()

for ii in range(0,len(idx),2):
    plt.axvspan(df['Time'][idx[ii]], df['Time'][idx[ii+1]], color='gray', alpha=0.2)
 
########################
# Show the plots
########################
plt.show()
fig.savefig(str(fileName).replace('.csv','')+'.png', dpi=fig.dpi)
