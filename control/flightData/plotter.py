import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse

########################
# Argparse
# Example use: python plotter.py --input "data.csv"
########################
parser = argparse.ArgumentParser()
parser.add_argument("--input", help = "input filename")
args = parser.parse_args()
fileName = args.input

########################
# Functions
########################
def highlight(indices, ax):
    for ii in range(len(indices)):
        ax.axvspan(indices[ii]-0.5, indices[ii]+0.5, facecolor='grey', edgecolor='none', alpha=0.3)

########################
# Prepare CSV
########################
try:
    df = pd.read_csv(fileName, header = 0, names = ['Mode', 'Time',
								'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
								'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
								'North-Desired', 'East-Desired', 'Down-Desired',
								'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control'])

    df['normTime'] = df['Time'] - df['Time'][0]
except:
    print('Error with file.')
    exit()

########################
# Roll and Pitch
########################
plt.figure()
ax0 = plt.gca()

df.plot(kind='line', x='normTime', y='Roll-Control',  color='#FB8604', style='-', ax=ax0)
df.plot(kind='line', x='normTime', y='Pitch-Control', color='#700CBC', style='-', ax=ax0)

df.plot(kind='line', x='normTime', y='Roll-UAV',  color='#FB8604', style='--',  ax=ax0)
df.plot(kind='line', x='normTime', y='Pitch-UAV', color='#700CBC', style='--',  ax=ax0)

ax0.set_title('Roll & Pitch Control', fontsize=14, fontweight='bold')
ax0.set_xlabel('Time [s]', fontweight='bold')
ax0.set_ylabel('Angle [deg]', fontweight='bold')

highlight(df[df['Mode'] == 'GUIDED_NOGPS'].index, ax0)

########################
# Yaw
########################
plt.figure()
ax1 = plt.gca()
ax2 = ax1.twinx()

df.plot(kind='line', x='normTime', y='Yaw-Vision', color='#347B98', style='-', ax=ax1)
df.plot(kind='line', x='normTime', y='Yaw-Control', color='#F0BD04', style='--', ax=ax2)

ax1.set_title('Yaw Control', fontsize=14, fontweight='bold')
ax1.set_xlabel('Time [s]', fontweight='bold')

ax1.set_ylabel('Yaw [Deg]', color='#347B98', fontweight='bold')
ax2.set_ylabel('Yaw Rate [Deg/s]', color='#F0BD04', fontweight='bold')

ax1.get_legend().remove()
ax2.get_legend().remove()

highlight(df[df['Mode'] == 'GUIDED_NOGPS'].index, ax1)

########################
# North East
########################
plt.figure()
ax3 = plt.gca()

df.plot(kind='line', x='normTime', y='North-Vision', color='#700CBC', style='-',  ax=ax3)
df.plot(kind='line', x='normTime', y='East-Vision',  color='#FB8604', style='-',  ax=ax3)
df.plot(kind='line', x='normTime', y='Down-Vision',  color='#7FBD32', style='-',  ax=ax3)

df.plot(kind='line', x='normTime', y='North-Desired', color='#700CBC', style='--',  ax=ax3)
df.plot(kind='line', x='normTime', y='East-Desired',  color='#FB8604', style='--',  ax=ax3)
df.plot(kind='line', x='normTime', y='Down-Desired',  color='#7FBD32',  style='--',  ax=ax3)

ax3.set_title('NED Position', fontsize=14, fontweight='bold')
ax3.set_xlabel('Time [s]', fontweight='bold')
ax3.set_ylabel('Position [cm]', fontweight='bold')

highlight(df[df['Mode'] == 'GUIDED_NOGPS'].index, ax3)

########################
# Thrust
########################
plt.figure()
ax4 = plt.gca()

df.plot(kind='line', x='normTime', y='Thrust-Control', color='#7FBD32', style='-', ax=ax4)

ax4.set_title('Thrust Control', fontsize=14, fontweight='bold')
ax4.set_xlabel('Time [s]', fontweight='bold')
ax4.set_ylabel('Normalized Thrust Command', fontweight='bold')
ax4.get_legend().remove()

highlight(df[df['Mode'] == 'GUIDED_NOGPS'].index, ax4)

########################
# Show the plots
########################
plt.show()
