
# Import packages
import pandas as pd
import matplotlib.pyplot as plt

# fileName = 'firstFlight.csv'
fileName = 'secoundFlight.csv'
# fileName = 'test.csv'

# Read the csv into a panda data frame
data = pd.read_csv(fileName, header=0, names=['Mode', 'Time', 'RC Roll', 'Roll Control', 'Roll Actual',
				'RC Pitch', 'Pitch Control', 'Pitch Actual', 'Thrust', 'Down Pos'])

# Plot
# ax1 = plt.subplot(211)
# plt.title('Altitude Command Test')
# plt.plot(data['Time'], data['Down Pos'], '-k')
# plt.setp(ax1.get_xticklabels(), fontsize=6)
# plt.ylabel('Altitude [m]')

ax1 = plt.subplot(211)
plt.title('Roll and Pitch Control')
plt.plot(data['Time'], data['Roll Control'], data['Time'], data['Roll Actual'])
plt.setp(ax1.get_xticklabels(), fontsize=6)
plt.ylabel('Angle [deg]')

STABILIZE = data.index[data['Mode'] == 'STABILIZE'].tolist()
GUIDED = data.index[data['Mode'] == 'GUIDED_NOGPS'].tolist()
plt.axvspan(data['Time'].iloc[GUIDED[0]], data['Time'].iloc[GUIDED[-1]], facecolor='g', alpha=0.3)

# ax2 = plt.subplot(212, sharex=ax1)
# plt.plot(data['Time'], data['Thrust'], '-k')
# plt.setp(ax2.get_xticklabels(), visible=False)
# plt.ylabel('Thrust Input [0 1]')
# plt.xlabel('time (s)')

ax2 = plt.subplot(212, sharex=ax1)
plt.plot(data['Time'], data['Pitch Control'], data['Time'], data['Pitch Actual'])
plt.setp(ax2.get_xticklabels(), visible=False)
plt.ylabel('Angle [deg]')
plt.xlabel('Time (s)')


plt.axvspan(data['Time'].iloc[GUIDED[0]], data['Time'].iloc[GUIDED[-1]], facecolor='g', alpha=0.3)

plt.show()
