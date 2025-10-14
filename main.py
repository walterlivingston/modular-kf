import numpy as np
import matplotlib.pyplot as plt

from modular_kf.filters import KalmanFilter
from modular_kf.models import PendulumSystemModel, PendulumFullMeasModel
from modular_kf.core.utils import WithCovariance

# pendulum parameters
g = -9.81  # gravity (m/s^2)
l = 1.0  # length of pendulum (m)
b = 0.1  # damping coefficient (kg*m^2/s)
m = 1.0  # mass of the pendulum

# time setup
t_final = 10
num_steps = 1000
dt = 10 / 1000
t_eval = np.linspace(0, t_final, num_steps)
N = len(t_eval)

# simulation
x0 = np.array([0.1, 0.0])
truth = np.zeros((2, N))
truth[:, 0] = x0
for k in range(1, N):
    truth[:, k] = np.array(
        [
            truth[0, k - 1] + truth[1, k - 1] * dt,
            truth[1, k - 1]
            + ((m * g * l * np.sin(truth[0, k - 1])) - b * truth[1, k - 1]) * dt,
        ]
    )

# filter setup
state_sigmas = np.array([0.1, 0.1])
meas_sigma = np.deg2rad(np.array([1, 1]))

sys_model = PendulumSystemModel(x0, state_sigmas, m, l, b)
full_meas_model = PendulumFullMeasModel(x0, meas_sigma)
filter = KalmanFilter(sys_model, full_meas_model)

# run filter
state_list: list[WithCovariance] = []
y_ = np.zeros((2, N))
for k in range(0, N):
    filter.predict(dt)
    y_[:, k] = truth[:, k] + np.random.normal(0, meas_sigma, size=truth[:, k].shape)
    filter.update(filter.meas_model.H @ y_[:, k])
    state_list.append(filter.state)

estimates = np.array([s.value for s in state_list]).T

error = truth - estimates

# results
fig, axd = plt.subplot_mosaic([["A"], ["B"], ["C"]], figsize=(10, 6))

axd["A"].plot(t_eval, truth[0, :], label="theta (rad)")
axd["A"].plot(t_eval, y_[0, :], label="theta_meas (rad)")
axd["A"].plot(t_eval, estimates[0, :], label="theta_est (rad)")
axd["A"].set_xlabel("Time (s)")
axd["A"].set_ylabel("Angle (rad)")
axd["A"].set_title("Pendulum Angle vs. Time")
axd["A"].legend()
axd["A"].grid(True)

axd["B"].plot(t_eval, truth[1, :], label="dtheta (rad/s)")
axd["B"].plot(t_eval, y_[1, :], label="dtheta_meas (rad/s)")
axd["B"].plot(t_eval, estimates[1, :], label="dtheta_est (rad/s)")
axd["B"].set_xlabel("Time (s)")
axd["B"].set_ylabel("Angular Rate (rad/s)")
axd["B"].set_title("Pendulum Angular Rate vs. Time")
axd["B"].legend()
axd["B"].grid(True)

axd["C"].plot(t_eval, error[0, :], label="Angle Error (rad)")
axd["C"].plot(t_eval, error[1, :], label="Angular Rate Error (rad/s)")
axd["C"].set_xlabel("Time (s)")
axd["C"].set_ylabel("Error")
axd["C"].set_title("Error vs. Time")
axd["C"].grid(True)

plt.tight_layout()
plt.show()
