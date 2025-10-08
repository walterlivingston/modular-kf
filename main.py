from modular_kf.filters import KalmanFilter
from modular_kf.models.system import PendulumSystemModel
from modular_kf.models.measurement import *
from modular_kf.core.utils import WithCovariance

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import expm

# Pendulum parameters
g = 9.81  # gravity (m/s^2)
l = 1.0  # length of pendulum (m)
b = 0.1  # damping coefficient (kg*m^2/s)
m = 1.0  # mass of the pendulum

# State-space matrices for linearized pendulum
# State vector x = [theta, theta_dot]
A = np.array([[0, 1], [-g / l, -b]])
B = np.array([[0], [1]])  # optional input (torque)
C = np.eye(2)  # we want to observe both theta and theta_dot
D = np.array([[0], [0]])

# Simulation time
tf = 10
num_steps = 1000
dt = 10 / 1000
t_span = (0, tf)  # 10 seconds
t_eval = np.linspace(t_span[0], t_span[1], num_steps)
N = len(t_eval)

# Initial condition: small angle (radians) and zero velocity
x0 = np.array([0.1, 0.0])
Phi = expm(A * dt)

# Define the state-space ODE
y = np.zeros((2, N))
y[:, 0] = x0.T
for k in range(1, N):
    y[:, k] = Phi @ y[:, k - 1]


# Solve the ODE
# sol = solve_ivp(pendulum_ode, t_span, x0, t_eval=t_eval)

meas_sigma = np.deg2rad(np.array([1, 1]))

sys_model = PendulumSystemModel(x0, np.array([1, 1]), m, l, b)
full_meas_model = PendulumFullMeasModel(x0, meas_sigma)
dtheta_meas_model = PendulumThetaDotMeasModel(x0, meas_sigma[1])
filter = KalmanFilter(sys_model, dtheta_meas_model)

state_list: list[WithCovariance] = []
y_ = np.zeros((2, N))
for k in range(0, N):
    filter.predict(10 / 1000)
    y_[:, k] = y[:, k] + np.random.normal(0, meas_sigma, size=y[:, k].shape)
    # filter.update(y_[:, k])
    filter.update(y_[1, k])

    state_list.append(filter.state)

estimates = np.zeros((2, N))
for k in range(0, N):
    estimates[:, k] = state_list[k].value

# Plot results
plt.figure(figsize=(10, 5))
plt.plot(t_eval, y[0], label="theta (rad)")
plt.plot(t_eval, y_[0, :], label="theta_meas (rad)")
plt.plot(t_eval, estimates[0, :], label="theta estimate (rad)")
plt.xlabel("Time (s)")
plt.ylabel("States")
# plt.title("Linearized Pendulum Simulation (State-Space)")
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 5))
plt.plot(t_eval, y[1], label="dtheta (rad/s)")
plt.plot(t_eval, y_[1, :], label="dtheta_meas (rad/s)")
plt.plot(t_eval, estimates[1, :], label="dtheta (rad/s)")
plt.xlabel("Time (s)")
plt.ylabel("States")
# plt.title("Linearized Pendulum Simulation (State-Space)")
plt.legend()
plt.show()
