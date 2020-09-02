from filter import TimeSync
import statistics
import random
import time

# Class
loop = TimeSync(1/30)

# Logging variables
freqList = []

# Timers start
startTime = time.time()
loopTimer = time.time()
loop.startTimer()

# Loop for some time
while (time.time() < startTime+10):
    # Stabilize rate
    loop.stabilize()

    # Frequency rate
    freqLocal = (1 / (time.time() - loopTimer))
    freqList.append(freqLocal)
    print(freqLocal)
    loopTimer = time.time()

    # Some delay
    time.sleep(random.random()/30)

# Results
print('Average loop rate: ', round(statistics.mean(freqList),2), '+/-', round(statistics.stdev(freqList), 2))





# from jtop import jtop


# if __name__ == "__main__":

#     print("Simple jtop reader")

#     with jtop() as jetson:
#         # jetson.ok() will provide the proper update frequency
#         while jetson.ok():
#             # Read tegra stats
#             print(jetson.stats)