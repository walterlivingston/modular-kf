import pandas as pd
import numpy as np
import quaternions as q
from examples.MEKF6 import mekfStateBlock, mekfAccMeasBlock, mekfMagMeasBlock

filename = "data/AHRS/641_657_N_Fifth_St-2025-01-01_22-07-59/"

gyr_df = pd.read_csv(filename + "Gyroscope.csv")
acc_df = pd.read_csv(filename + "Accelerometer.csv")
grav_df = pd.read_csv(filename + "Gravity.csv")
mag_df = pd.read_csv(filename + "Magnetometer.csv")
gps_df = pd.read_csv(filename + "Location.csv")

gyr = np.array([[gyr_df["x"]],[gyr_df["y"]],[gyr_df["z"]]])
grav = np.array([[grav_df["x"]],[grav_df["y"]],[grav_df["z"]]])
acc = np.array([[acc_df["x"]],[acc_df["y"]],[acc_df["z"]]]) + grav
mag = np.array([[mag_df["x"]],[mag_df["y"]],[mag_df["z"]]])

truth_heading = gps_df["bearing"]

time = gyr_df["time"]

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

stateSigmas = np.array([noise_g, noise_bias]).T
measSigmas = np.array([noise_a, noise_m]).T

eul0 = np.array([0, 0, truth_heading[1]]).T
q0 = q.eul2q(eul0)

sBlock = mekfStateBlock.mekfStateBlock(6, stateSigmas)
# sBlock.processAuxData(aux)
accMBlock = mekfAccMeasBlock.mekfAccMeasBlock(measSigmas[:3])
magMBlock = mekfMagMeasBlock.mekfMagMeasBlock(measSigmas[3:])
# covBlock = basicCovBlock()