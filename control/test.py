import matplotlib.pyplot as plt
from filter import TimeSync
import statistics
import random
import time

# Logging variables
freqList = []
timeList = []
reqSleepList = []
actSleepList = []

# Class
sync = TimeSync(1.0/30.0)
sync.startTimer()

# Timers start
startTime = time.time()
loopTimer = time.time()
sync.startTimer()

# Loop for some time
while (time.time() < startTime+10):
    start = time.time()
    sleepy = sync.stabilize()
    end = time.time()

    reqSleepList.append(sleepy * 1000.0)
    actSleepList.append((end - start) * 1000.0)
   
    # Frequency rate
    temp = time.time()
    freqLocal = (1.0 / (temp - loopTimer))
    timeList.append(time.time()-startTime)
    freqList.append(freqLocal)
    loopTimer = temp

    # Some delay
    # time.sleep(random.random()/40)

# Create a figure
fig = plt.figure()



# Create subplot: Frequency
plt.subplot(1, 2, 1)
ax1 = plt.gca()

# Plot data
ax1.plot(timeList, freqList, '-k')

# Format figure
title = 'Sampling Frequency\n' + str(round(statistics.mean(freqList),2)) + '+/-' + str(round(statistics.stdev(freqList), 2))
ax1.set_title(title, fontsize=14, fontweight='bold')
ax1.set_xlabel('Time [s]', fontweight='bold')
ax1.set_ylabel('Frequency [Hz]', fontweight='bold')



# Create subplot: Sleep request
plt.subplot(1, 2, 2)
ax0 = plt.gca()

# Plot data
ax0.plot(reqSleepList, actSleepList, 'r+')

# Figure formatting
ax0.set_title('Sleep Request', fontsize=14, fontweight='bold')
ax0.set_xlabel('Requested Sleep Time [ms]', fontweight='bold')
ax0.set_ylabel('Actual Sleep Time [ms]', fontweight='bold')

# Show and save
plt.grid()
# plt.show()
fig.savefig('test.png', dpi=fig.dpi)
