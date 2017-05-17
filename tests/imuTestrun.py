import smbus
from datetime import datetime
from helpfunctions import *
from imu import *

import numpy as np
import scipy

# Pin definitions
# intPin  = 12  # These can be changed, 2 and 3 are the Arduinos ext int pins
# myLed   = 13 # Set up pin 13 led for toggling

#int16_t accelCount[3]  # Stores the 16-bit signed accelerometer sensor output
#int16_t gyroCount[3]   # Stores the 16-bit signed gyro sensor output
#int16_t magCount[3]    # Stores the 16-bit signed magnetometer sensor output
#magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}  # Factory mag calibration and mag bias
#gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}      # Bias corrections for gyro and accelerometer


bus = smbus.SMBus(1)
print("BUS: " + str(bus))

#IMU_setup(bus)
IMU_reset(bus)

A = np.array([[0.999670953587901, 0.003079116766345, 0.008528774791551],[-0.035803072896127,0.995984902109447,-0.009451396312668],[0.002566316379190,-0.031784161946969,0.992275350434413]])
b = np.array([[0.041225832670749],[0.159751748029843],[0.001179938074234]])

a = np.array([[0.],[0.],[0.]],)
g = np.array([[0.],[0.],[0.]])

for k in range(1000):
     
    data = IMU_read(bus)
    a[0] = data["ax"]
    a[1] = data["ay"]
    a[2] = data["az"]
    g[0] = data["gx"]
    g[1] = data["gy"]
    g[2] = data["gz"]
    a_cor = np.dot(A,a) + b
    g_cor = g
    print(g_cor[0])
    print("__________________")
    time.sleep(0.05)



        
