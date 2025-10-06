from modular_kf.filters import KalmanFilter
from modular_kf.models.system import PendulumSystemModel
from modular_kf.models.measurement import *

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

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
t_span = (0, 10)  # 10 seconds
t_eval = np.linspace(t_span[0], t_span[1], 1000)

# Initial condition: small angle (radians) and zero velocity
x0 = [0.1, 0.0]


# Define the state-space ODE
def pendulum_ode(t, x):
    u = 0  # no control input
    dxdt = A @ x + B.flatten() * u
    return dxdt


# Solve the ODE
sol = solve_ivp(pendulum_ode, t_span, x0, t_eval=t_eval)

sys_model = PendulumSystemModel(np.array(x0), np.array([1, 1]), m, l, b)

theta_meas_model = PendulumThetaMeasModel(np.array(x0), np.array([1, 1]))
dtheta_meas_model = PendulumThetaDotMeasModel(np.array(x0), np.array([1, 1]))
full_meas_model = PendulumFullMeasModel(np.array(x0), np.array([1, 1]))

filter = KalmanFilter(sys_model, full_meas_model)

filter.predict(0.1)

# Plot results
# plt.figure(figsize=(10,5))
# plt.plot(sol.t, sol.y[0], label='theta (rad)')
# plt.plot(sol.t, sol.y[1], label='theta_dot (rad/s)')
# plt.xlabel('Time (s)')
# plt.ylabel('States')
# plt.title('Linearized Pendulum Simulation (State-Space)')
# plt.legend()
# plt.grid(True)
# plt.show()
