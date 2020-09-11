import matplotlib.pyplot as plt
from filter import TimeSync
import statistics
import random
import time

# Class
sync = TimeSync(1.0/30.0)
sync.startTimer()

# Logging variables
freqList = []

# Timers start
startTime = time.time()
loopTimer = time.time()
sync.startTimer()

# Loop for some time
while (time.time() < startTime+10):
    # Stabilize rate
    sync.stabilize()

    # Frequency rate
    freqLocal = (1 / (time.time() - loopTimer))
    freqList.append(freqLocal)
    print(freqLocal)
    loopTimer = time.time()

    # Some delay
    time.sleep(random.random()/40)

# Results
print('Average loop rate: ', round(statistics.mean(freqList),2), '+/-', round(statistics.stdev(freqList), 2))

# Plot the results
plt.plot(freqList)
plt.show()
