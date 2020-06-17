import simdkalman
import numpy as np
import numpy.random as random
import matplotlib.pyplot as plt

# Configure the filter
kf = simdkalman.KalmanFilter(
    state_transition = np.array([[1,1],[0,1]]),     # A
    process_noise = np.diag([0.1, 0.01]),           # Q
    observation_model = np.array([[1,0]]),          # H
    observation_noise = 1.0)                        # R

# simulate 100 random walk time series
# introduce 10% of NaNs denoting missing values
rand = lambda: random.normal(size=(1, 200))
data = np.cumsum(np.cumsum(rand()*0.02, axis=1) + rand(), axis=1) + rand()*3
data[random.uniform(size=data.shape) < 0.1] = np.nan

# fit noise parameters to data with the EM algorithm (optional)
kf = kf.em(data, n_iter=10)

# smooth and explain existing data
smoothed = kf.smooth(data)

# predict new data
pred = kf.predict(data, 15)


# Plot the data
ax1 = plt.gca()

plt.title("time series")
x = np.arange(0, data.shape[1])

ax1.plot(x, data[0], 'b.', label="data")

smoothed_obs = smoothed.observations.mean[0]
obs_stdev = np.sqrt(smoothed.observations.cov[0])
ax1.plot(x, smoothed_obs, 'r-', label="smoothed")
ax1.plot(x, smoothed_obs - obs_stdev, 'k--', label="67% confidence")
ax1.plot(x, smoothed_obs + obs_stdev, 'k--')

x_pred = np.arange(data.shape[1], data.shape[1]+pred.observations.mean.shape[1])
y_pred = pred.observations.mean[0]
pred_stdev = np.sqrt(pred.observations.cov[0])
ax1.plot(x_pred, y_pred, 'b-', label="predicted")
ax1.plot(x_pred, y_pred + pred_stdev, 'k--')
ax1.plot(x_pred, y_pred - pred_stdev, 'k--')
ax1.legend()

plt.show()
