import smbus
from datetime import datetime
from helpfunctions import *
from imu import *

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

IMU_setup(bus)
#IMU_reset(bus)

#lastUpdate = 0
ansage = ["X up", "X down", "Y up", "Y down", "Z up", "Z down"]
ax = [0, 0, 0, 0, 0, 0]
ay = [0, 0, 0, 0, 0, 0]
az = [0, 0, 0, 0, 0, 0]
gx = [0, 0, 0, 0, 0, 0]
gy = [0, 0, 0, 0, 0, 0]
gz = [0, 0, 0, 0, 0, 0]
mx = [0, 0, 0, 0, 0, 0]
my = [0, 0, 0, 0, 0, 0]
mz = [0, 0, 0, 0, 0, 0]
N = 100
gxAv = 0
gyAv = 0
gzAv = 0
for k in range(6):
    input(ansage[k])
    axAv = 0
    ayAv = 0
    azAv = 0
    mxAv = 0
    myAv = 0
    mzAv = 0
    for j in range(N):  
        data = IMU_read(bus)
        axAv += data["ax"]
        ayAv += data["ay"]
        azAv += data["az"]
        gxAv += data["gx"]
        gyAv += data["gy"]
        gzAv += data["gz"]
        mxAv += data["mx"]
        myAv += data["my"]
        mzAv += data["mz"]
        time.sleep(0.05)
    ax[k] = axAv/N
    ay[k] = ayAv/N
    az[k] = azAv/N
    mx[k] = mxAv/N
    my[k] = myAv/N
    mz[k] = mzAv/N
    print("ax: \t", axAv/N)
    print("ay: \t", ayAv/N)
    print("az: \t", azAv/N)
    print("mx: \t", mxAv/N)
    print("my: \t", myAv/N)
    print("mz: \t", mzAv/N)
    print("temp: \t", data["temperature"])
    print("____________________________________")
print("a_mess = [", ax[0], ax[1], ax[2], ax[3], ax[4], ax[5], ";")
print("          ", ay[0], ay[1], ay[2], ay[3], ay[4], ay[5], ";")
print("          ", az[0], az[1], az[2], az[3], az[4], az[5], "]")
print("g_mess = [", gxAv/N/6, gyAv/N/6, gzAv/N/6, "]'")


        
