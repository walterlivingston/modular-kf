import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import quaternions as q
from modular_kf import KalmanFilter
from modular_kf.helpers import Aux
from examples.MEKF6 import mekfStateBlock, mekfAccMeasBlock, mekfMagMeasBlock
from examples.basicCovBlock import BasicCovBlock

filename = "data/AHRS/300_398_Fifth_Ave-2025-01-01_22-01-14/"

gyr_df = pd.read_csv(filename + "Gyroscope.csv")
acc_df = pd.read_csv(filename + "Accelerometer.csv")
grav_df = pd.read_csv(filename + "Gravity.csv")
mag_df = pd.read_csv(filename + "Magnetometer.csv")
gps_df = pd.read_csv(filename + "Location.csv")

gyr = np.squeeze(np.array([[gyr_df["x"]],[gyr_df["y"]],[gyr_df["z"]]]))
grav = (np.array([[grav_df["x"]],[grav_df["y"]],[grav_df["z"]]]))
acc = np.squeeze(np.array([[acc_df["x"]],[acc_df["y"]],[acc_df["z"]]]) + grav)
mag = np.squeeze(np.array([[mag_df["x"]],[mag_df["y"]],[mag_df["z"]]]))

truth_heading = gps_df["bearing"]

timeIMU = gyr_df["seconds_elapsed"]
timeIMU = timeIMU - timeIMU[0]
N = len(timeIMU)

timeGPS = gps_df["seconds_elapsed"]
timeGPS = timeGPS - timeGPS[0]

sensorToBody = np.array([[np.cos(np.pi/2), 0, np.sin(np.pi/2)],
                         [0,             1, 0],
                         [-np.sin(np.pi/2), 0, np.cos(np.pi/2)]])

acc_sigma = 0.019
gyr_sigma = 0.0048
mag_sigma = 1e-6*2.7

noise_g     = 1*gyr_sigma*np.ones((3,1))
noise_a     = 1*acc_sigma*np.ones((3,1))
noise_m     = 1*mag_sigma*np.ones((3,1))
noise_bias  = 0.1*gyr_sigma*np.ones((3,1))

stateSigmas = np.squeeze(np.vstack((noise_g, noise_bias)))
measSigmas = np.squeeze(np.vstack((noise_a, noise_m)))

eul0 = np.array([0, 0, truth_heading[1]]).T
q0 = q.eul2q(eul0)
q0 = q0.reshape((4,1))

aux = Aux()
aux.y = np.zeros((1,6))

sBlock = mekfStateBlock.mekfStateBlock(6, stateSigmas)
sBlock.processAuxData(aux)
accMBlock = mekfAccMeasBlock.mekfAccMeasBlock(measSigmas[:3])
magMBlock = mekfMagMeasBlock.mekfMagMeasBlock(measSigmas[3:])
covBlock = BasicCovBlock()

filter = KalmanFilter(sBlock, accMBlock, covBlock, X_i = np.vstack((q0, np.zeros((3,1)))), mode = 'error')

win_size = 1
win_overlap = win_size - 1
win_step = win_size - win_overlap

for k in range(win_size, N - win_step-1, win_step):
    win = list(range(k - win_size, k))
    winp = [i + win_step for i in win]
    dt = timeIMU[win[-1]] - timeIMU[win[0] + win_overlap]

    acc_k = sensorToBody @ np.mean(acc[:, win], axis=1)
    gyr_k = sensorToBody @ np.mean(gyr[:, win], axis=1)
    gyr_kp = sensorToBody @ np.mean(gyr[:, winp], axis=1)
    mag_k = sensorToBody @ np.mean(mag[:, win], axis=1)

    y_acc = acc_k
    aux.y = np.concatenate((gyr_k, gyr_kp))

    filter.state_block.processAuxData(aux);
    filter.process(dt);
    filter.update(y_acc, accMBlock);

    # acc_k = sensorToBody*mean(acc(:,win),2);
    # gyr_k = sensorToBody*mean(gyr(:,win),2);
    # gyr_kp = sensorToBody*mean(gyr(:,winp),2);
    # mag_k = sensorToBody*mean(mag(:,win),2);

plt.plot(timeGPS, truth_heading, marker='o', linestyle='none')
plt.title('Time Series Plot')
plt.xlabel('Time (s)')
plt.ylabel('Time (s)')
plt.grid(True)
plt.show()