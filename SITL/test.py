# https://github.com/oseiskar/simdkalman/issues/3
import numpy as np
import simdkalman
import pandas as pd
import matplotlib.pyplot as plt

np.random.seed(0)
data = np.random.normal(size=150)

kf = simdkalman.KalmanFilter(
    state_transition = [[1,1],[0,1]],        # A
    process_noise = np.diag([0.05, 0.002]),  # Q
    observation_model = np.array([[1,0]]),   # H
    observation_noise = 20.0)                # R

kf_results = kf.smooth(data)

t = range(data.size)

def kf_smooth(y):
    return kf.smooth(y).observations.mean

def kf_filter(y):
    return kf.compute(y, 0, filtered=True).filtered.observations.mean

plt.plot(t, data, 'kx', alpha=0.2, label='data')
plt.plot(t[:100], data[:-50], 'bx', alpha=0.4, label='first 100')
plt.plot(t, kf_smooth(data), 'k-', label='smoothed full')
plt.plot(t, kf_filter(data), 'k--', label='filtered full')
plt.plot(t[:100], kf_smooth(data[:100]), 'b-', label='smoothed first 100')
plt.plot(t[:100], kf_filter(data[:100]), 'b--', label='filtered first 100')
plt.legend()
plt.show()
