import numpy as np
import matplotlib.pyplot as plt

from modular_kf.filters import KalmanFilter, ExtendedKalmanFilter
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
dt = t_final / num_steps
t_eval = np.linspace(0, t_final, num_steps)
N = len(t_eval)

# simulation
x0 = np.array([1, 0.0])
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
state_sigmas = np.array([0.1, 0.01])
meas_sigma = np.deg2rad(np.array([1, 1]))

sys_model = PendulumSystemModel(x0, state_sigmas, m, l, b)
meas_model = PendulumFullMeasModel(x0, meas_sigma)

linear_filter = KalmanFilter(sys_model, meas_model)
extended_filter = ExtendedKalmanFilter(sys_model, meas_model)

# run filter
linear_state_list: list[WithCovariance] = []
extended_state_list: list[WithCovariance] = []
y_ = np.zeros((2, N))
for k in range(0, N):
    y_[:, k] = truth[:, k] + np.random.normal(0, meas_sigma, size=truth[:, k].shape)

    linear_filter.predict(dt)
    extended_filter.predict(dt)

    linear_filter.update(linear_filter.meas_model.H @ y_[:, k])
    extended_filter.update(extended_filter.meas_model.H @ y_[:, k])

    linear_state_list.append(linear_filter.state)
    extended_state_list.append(extended_filter.state)

linear_estimates = np.array([s.value for s in linear_state_list]).T
extended_estimates = np.array([s.value for s in extended_state_list]).T

linear_error = truth - linear_estimates
avg_linear_error = np.mean(linear_error, axis=1)

extended_error = truth - extended_estimates
avg_extended_error = np.mean(extended_error, axis=1)

# results
fig, axd = plt.subplot_mosaic([["A", "C"], ["B", "D"]], figsize=(10, 6))

axd["A"].plot(t_eval, truth[0, :], label="Truth")
axd["A"].plot(t_eval, linear_estimates[0, :], label="Linear")
axd["A"].plot(t_eval, extended_estimates[0, :], label="Extended")
axd["A"].set_xlabel("Time (s)")
axd["A"].set_ylabel("Angle (rad)")
axd["A"].set_title("Pendulum Angle vs. Time")
axd["A"].legend()
axd["A"].grid(True)

axd["B"].plot(t_eval, truth[1, :], label="Truth")
axd["B"].plot(t_eval, linear_estimates[1, :], label="Linear")
axd["B"].plot(t_eval, extended_estimates[1, :], label="Extended")
axd["B"].set_xlabel("Time (s)")
axd["B"].set_ylabel("Angular Rate (rad/s)")
axd["B"].set_title("Pendulum Angular Rate vs. Time")
axd["B"].legend()
axd["B"].grid(True)

axd["C"].plot(t_eval, linear_error[0, :], label="Linear")
axd["C"].plot(t_eval, extended_error[0, :], label="Extended")
axd["C"].set_xlabel("Time (s)")
axd["C"].set_ylabel("Angle Error (rad)")
axd["C"].set_title("Pendulum Angle Error vs. Time")
axd["C"].legend()
axd["C"].grid(True)

axd["D"].plot(t_eval, linear_error[1, :], label="Linear")
axd["D"].plot(t_eval, extended_error[1, :], label="Extended")
axd["D"].set_xlabel("Time (s)")
axd["D"].set_ylabel("Angular Rate Error (rad/s)")
axd["D"].set_title("Pendulum Angular Rate Error vs. Time")
axd["D"].legend()
axd["D"].grid(True)

plt.tight_layout()
plt.show()
