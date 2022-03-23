#!/usr/bin/env python3

from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt

normal = np.random.normal  # type:ignore
random = np.random.random  # type:ignore
asarray = np.asarray  # type:ignore

start = 0
end = 100
timepoints = 1000
datapoints = 20
t = np.linspace(start, end, timepoints)
X = np.linspace(start, end, datapoints)
Y = np.linspace(start, end, datapoints)
print(X)

n_sensors = 5
sensor_std_dev = np.array([random() * 1 for _ in range(0, n_sensors)])
sensor_variance = sensor_std_dev ** 2

sensor_x_value = []
sensor_y_value = []
for n in range(0, n_sensors):
    sensor_x_value.append(
        normal(X, sensor_std_dev[n]),
    )

    sensor_y_value.append(
        normal(Y, sensor_std_dev[0]),
    )

# fmt:off
# time step
dt = t[2] - t[1]

# transition_matrix
F = [[1, 0, dt, 0 ],
     [0, 1, 0,  dt],
     [0, 0, 1,  0 ],
     [0, 0, 0,  1 ]]

# observation_matrix
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]

# transition_covariance
Q = [[1e-4, 0,    0,    0   ],
     [0,    1e-4, 0,    0   ],
     [0,    0,    1e-4, 0   ],
     [0,    0,    0,    1e-4]]

# observation_covariance
R = []
for n in range(0, n_sensors):
        R.append([[sensor_variance[n], 0                 ],
                  [0,                  sensor_variance[n]]])
# initial_state_mean
X0 = [0, 0, 0, 0]

# initial_state_covariance - assumed a bigger uncertainty in initial velocity
P0 = [[0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]]

# fmt:on

n_timesteps = len(t)
n_dim_state = 4
filtered_state_means = np.zeros((n_timesteps, n_dim_state))
filtered_state_covariances = np.zeros((n_timesteps, n_dim_state, n_dim_state))

# Kalman-Filter initialization
kf = KalmanFilter(
    transition_matrices=F,
    observation_matrices=H,
    transition_covariance=Q,
    observation_covariance=R[1],
    initial_state_mean=X0,
    initial_state_covariance=P0,
)


# iterative estimation for each new measurement
for t in range(datapoints):
    for n in range(0, n_sensors):
        if t == 0:
            filtered_state_means[t] = X0
            filtered_state_covariances[t] = P0
        else:

            obs = [sensor_x_value[n][t], sensor_y_value[n][t]]
            obs_cov = asarray(R[n])

            filtered_state_means[t], filtered_state_covariances[t] = kf.filter_update(
                filtered_state_means[t - 1],
                filtered_state_covariances[t - 1],
                observation=obs,
                observation_covariance=obs_cov,
            )

# plot everything that just happened
# real
plt.plot(X, Y, "k-", label="Real Trajectory")
# sensors
for n in range(0, n_sensors):
    plt.plot(
        sensor_x_value[n],
        sensor_y_value[n],
        "r.",
        label="Sensor " + str(n),
    )
plt.plot(
    filtered_state_means[:, 0],
    filtered_state_means[:, 1],
    "go",
    label="Filtered Trajectory",
)
plt.grid()
plt.legend(loc="upper left")
plt.show()
