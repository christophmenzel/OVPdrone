from multiprocessing import Process, Value, Array
from datetime import datetime
from helpfunctions import *
from barometer import barometer_read
from ultrasonic import ultrasonic_read
from battery import *

SENS_H 		= 0
SENS_LAT 	= 1
SENS_LON 	= 2
SENS_ALT 	= 3
SENS_AX 	= 4
SENS_AY 	= 5
SENS_AZ  	= 6
SENS_GX  	= 7
SENS_GY 	= 8
SENS_GZ 	= 9
SENS_MX  	= 10
SENS_MY  	= 11
SENS_MZ  	= 12
SENS_T1  	= 13
SENS_T2  	= 14
SENS_P   	= 15
SENS_BAT 	= 16

STATE_X		= 0
STATE_Y		= 1
STATE_Z		= 2
STATE_U		= 3
STATE_V		= 4
STATE_W		= 5
STATE_PX	= 6
STATE_PY	= 7
STATE_PZ	= 8
STATE_WX	= 9
STATE_WY	= 10
STATE_WZ	= 11

def sensorDataUpdate(sens):
	
	while True:

		# Read current inertial information and temperature
		imu_data = IMU_read()
		if imu_data != -1:
			sens[SENS_AX] = imu_data['ax']
			sens[SENS_AY] = imu_data['ay']
			sens[SENS_AZ] = imu_data['az']
			sens[SENS_GX] = imu_data['gx']
			sens[SENS_GY] = imu_data['gy']
			sens[SENS_GZ] = imu_data['gz']
			sens[SENS_MX] = imu_data['mx']
			sens[SENS_MY] = imu_data['my']
			sens[SENS_MZ] = imu_data['mz']
			sens[T1]	  = imu_data['temperature']

		# Read current height from ultrasonic sensor
		us_data = ultrasonic_read()
		if us_data != -1
			sens[SENS_H] = us_data

		# Read current air pressure and temperature
		(bar_temp,bar_press) = barometer_read()
		if bar_data != -1
			sens[SENS_P] 	= bar_press
			sens[T2] 	= bar_temp

		# Read battery voltage
		bat_data = battery_read()
		if bat_data != -1
		 	sens[SENS_BAT] = bat_data
	

def stateEstimation(sens,state):
	# Kalman state estimation based on sensor data

def missionControl():
    # GSM connection to set current mission
    # 1) IDENTIFY: Perform actions to identify drone model
    # 2) HOVER: keep position at specified height
    # 3) RETURN HOME: get back to intial position and land
    # 4) COORD: fly directly to target coordinates
    # 4) SAFE STATE: blind descent (determine rate during identification)

def rotorControl(state):
	# PID controller that outputs PWM signals to all 4 rotors directly

if __name__ == '__main__':
        
        mqttClient = None

		# sensor data      h, lat, lon, alt, ax, ay, az, gx, gy, gz, mx, my, mz, temp1, temp2, press, battery
        sens = Array('d', [0,   0,   0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,     0,     0,     0,       0])

        # drone state       x, y, z, u, v, w, px, py, pz, wx, wy, wz 
        state = Array('d', [0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0])

        initializeDrone()

        p1 = Process(target=sensorDataUpdate, args=(sens,))
        p1.start()
        p2 = Process(target=stateEstimation, args=(sens,state))
        p2.start()
        p3 = Process(target=missionControl, args=())
        p3.start()
        p4 = Process(target=rotorControl, args=(state,))
        p4.start()

        p1.join()
        p2.join()
        p3.join()
        p4.join()

        try:
			while True:
				sleep(1)
		except KeyboardInterrupt:
			pass
		finally:
			mqttClient.loop_stop()
			mqttClient.disconnect()
