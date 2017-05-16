import math, statistics
import numpy as np
import smbus, time
import geomag 
import imu
import barometer

class Drone(object):

	# define mqtt topics
	TOPIC_STATUS = 'drone/status'

	# gravity boundaires
	ACC_IS_ZERO = 0.05
	ACC_IS_1G = 0.95


	def __init__(self):
		# Open serial bus
		self.bus = smbus.SMBus(1)


	def initialize():
		# Check GSM connection here!
		# TODO: ping tensor-external.ddns.net
		self.gsm_connected = True
		
		# Establish MQTT connection
	    def on_connect(client, userdata, flags, rc):
			print("connected to mqtt broker")
			client.subscribe('control', qos=1) 

		def on_subscribe(client, userdata, mid, granted_qos):
			print("subscribed to CAN/soc: "+str(mid)+" "+str(granted_qos))

		def on_message(client, userdata, msg):
			json_data = msg.payload.decode('utf-8')
			print("message received: "+json_data)

		def on_publish(client, userdata, mid):
		    print("mid: "+str(mid))

		def on_disconnect(client, userdata, rc):
		    print("disconnected. Reconnecting...")
		    client.connect("tensor-external.ddns.net", 1883)

		self.mqttClient = paho.Client()
		self.mqttClient.on_connect = on_connect
		self.mqttClient.on_subscribe = on_subscribe
		self.mqttClient.on_message = on_message
		self.mqttClient.on_disconnect = on_disconnect
		self.mqttClient.connect("tensor-external.ddns.net", 1883)
		self.mqttClient.loop_start()

	    # Determine home base GPS position
	    (self.lat0,self.lon0,self.alt0) = self.GPS_read()
	    self.mqttClient.publish(TOPIC_STATUS,"GPS location set: {} N, {} E, {} A".format(self.lat0,self.lon0,self.alt0), qos = 0)

	    # Determine the local horizontal geomagnetic reference field
	    gm = geomag.GeoMag("WMM.COF")
	    magfield = gm.GeoMag(self.lat0,self.lon0,self.alt0*3280.8399)
	    self.Bh = magfield.bh
	    self.mqttClient.publish(TOPIC_STATUS,"local magnetic field: {} nT".format(self.Bh), qos = 0)

	    # Load Magnetometer Calibration (should be loaded from file!)
		self.mx_noise = 0
		self.mx_bias = 0
		self.mx_factor = 1
		self.my_noise = 0
		self.my_bias = 0
		self.my_factor = 1
		self.mz_noise = 0
		self.mz_bias = 0
		self.mz_factor = 1
	    
	    # Initialize intertial measurement unit
	    self.imu = IMU(bus)
	    self.imu.initialize()
	    self.mqttClient.publish(TOPIC_STATUS,"IMU initialized.", qos = 0)

	    # Load Accelerometer Calibration (should be loaded from file!)
		self.ax_noise = 0
		self.ax_bias = 0
		self.ax_factor = 1
		self.ay_noise = 0
		self.ay_bias = 0
		self.ay_factor = 1
		self.az_noise = 0
		self.az_bias = 0
		self.az_factor = 1

	    # Load Gyroscope Calibration (should be loaded from file!)
		self.gx_noise = 0
		self.gx_bias = 0
		self.gx_factor = 1
		self.gy_noise = 0
		self.gy_bias = 0
		self.gy_factor = 1
		self.gz_noise = 0
		self.gz_bias = 0
		self.gz_factor = 1

	    # Determine h0, p0, T0
	    self.barometer = Barometer(bus)
	    self.h0 = 0
	    (self.T0,self.p0) = self.barometer.read()
	    self.mqttClient.publish(TOPIC_STATUS,"T0 = {} °C, p0 = {} bar, h0 = 0m".format(self.T0,self.p0), qos = 0)


	def calibrateIMU():

		# Determine noise
		ax_noise_vals = []
		ay_noise_vals = []
		az_noise_vals = []
		gx_noise_vals = []
		gy_noise_vals = []
		gz_noise_vals = []
		self.mqttClient.publish(TOPIC_STATUS,"Determine acc/gyro noise...", qos = 0)
		for k in range(200):
			data = self.imu.read()
			ax_noise_vals.append(data["ax"])
			ay_noise_vals.append(data["ay"])
			az_noise_vals.append(data["az"])
			gx_noise_vals.append(data["gx"])
			gy_noise_vals.append(data["gy"])
			gz_noise_vals.append(data["gz"])
			time.sleep(0.1)
		self.ax_noise = statistics.stdev(ax_noise_vals)
		self.ay_noise = statistics.stdev(ay_noise_vals)
		self.az_noise = statistics.stdev(az_noise_vals)
		self.gx_noise = statistics.stdev(gx_noise_vals)
		self.gy_noise = statistics.stdev(gy_noise_vals)
		self.gz_noise = statistics.stdev(gz_noise_vals)

		# Calibrate accelerometer X-axis
		self.mqttClient.publish(TOPIC_STATUS,"Pitch 90° up and hold...", qos = 0)
		while (sqrt(data["ay"]**2+data["az"]**2) < ACC_IS_ZERO && data["ax"] > 0):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"capturing...", qos = 0)
		time.sleep(0.5)
		ax_vals = []
		ay vals = []
		az vals = []
		for k in range(200):
			data = self.imu.read()
			ax_vals.append(data["ax"])
			ay_vals.append(data["ay"])
			az_vals.append(data["az"])
			time.sleep(0.1)
		ax_maxX = mean(ax_vals)
		ay_maxX = mean(ay_vals)
		az_maxX = mean(az_vals)
		self.mqttClient.publish(TOPIC_STATUS,"Pitch 90° down and hold...", qos = 0)
		while (sqrt(data["ay"]**2+data["az"]**2) < ACC_IS_ZERO && data["ax"] < 0):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"capturing...", qos = 0)
		time.sleep(0.5)
		ax_vals = []
		ay vals = []
		az vals = []
		for k in range(200):
			data = self.imu.read()
			ax_vals.append(data["ax"])
			ay_vals.append(data["ay"])
			az_vals.append(data["az"])
			time.sleep(0.1)
		ax_minX = mean(ax_vals)
		ay_minX = mean(ay_vals)
		az_minX = mean(az_vals)

		# Calibrate accelerometer Y-axis
		self.mqttClient.publish(TOPIC_STATUS,"Roll 90° right and hold...", qos = 0)
		while (sqrt(data["ax"]**2+data["az"]**2) < ACC_IS_ZERO && data["ay"] > 0):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"capturing...", qos = 0)
		time.sleep(0.5)
		ax_vals = []
		ay vals = []
		az vals = []
		for k in range(200):
			data = self.imu.read()
			ax_vals.append(data["ax"])
			ay_vals.append(data["ay"])
			az_vals.append(data["az"])
			time.sleep(0.1)
		ax_maxY = mean(ax_vals)
		ay_maxY = mean(ay_vals)
		az_maxY = mean(az_vals)
		self.mqttClient.publish(TOPIC_STATUS,"Roll 90° left and hold...", qos = 0)
		while (sqrt(data["ax"]**2+data["az"]**2) < ACC_IS_ZERO && data["ay"] < 0):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"capturing...", qos = 0)
		time.sleep(0.5)
		ax_vals = []
		ay vals = []
		az vals = []
		for k in range(200):
			data = self.imu.read()
			ax_vals.append(data["ax"])
			ay_vals.append(data["ay"])
			az_vals.append(data["az"])
			time.sleep(0.1)
		ax_minY = mean(ax_vals)
		ay_minY = mean(ay_vals)
		az_minY = mean(az_vals)
		
		# Calibrate accelerometer Z-axis
		self.mqttClient.publish(TOPIC_STATUS,"Set upright and hold...", qos = 0)
		while (sqrt(data["ax"]**2+data["ay"]**2) < ACC_IS_ZERO && data["az"] > 0):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"capturing...", qos = 0)
		time.sleep(0.5)
		ax_vals = []
		ay vals = []
		az vals = []
		for k in range(200):
			data = self.imu.read()
			ax_vals.append(data["ax"])
			ay_vals.append(data["ay"])
			az_vals.append(data["az"])
			time.sleep(0.1)
		ax_maxZ = mean(ax_vals)
		ay_maxZ = mean(ay_vals)
		az_maxZ = mean(az_vals)
		self.mqttClient.publish(TOPIC_STATUS,"Turn upside down and hold...", qos = 0)
		while (sqrt(data["ax"]**2+data["ay"]**2) < ACC_IS_ZERO && data["az"] < 0):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"capturing...", qos = 0)
		time.sleep(0.5)
		ax_vals = []
		ay vals = []
		az vals = []
		for k in range(200):
			data = self.imu.read()
			ax_vals.append(data["ax"])
			ay_vals.append(data["ay"])
			az_vals.append(data["az"])
			time.sleep(0.1)
		ax_minZ = mean(ax_vals)
		ay_minZ = mean(ay_vals)
		az_minZ = mean(az_vals)

		# Solve least mean sqare problem for distortion matrix A and bias vector b
		Reference = [[1, -1, 0, 0, 0, 0],[0, 0, 1, -1, 0, 0],[0, 0, 0, 0, 1, -1]]

		# TODO: Calculate A, b so that A*a+b=Reference


		

	def calibrateMag():

		# Determine noise
		mx_noise_vals = []
		my_noise_vals = []
		mz_noise_vals = []
		self.mqttClient.publish(TOPIC_STATUS,"Determine magnetic noise...", qos = 0)
		for k in range(200):
			data = self.imu.read()
			mx_noise_vals.append(data["mx"])
			my_noise_vals.append(data["my"])
			mz_noise_vals.append(data["mz"])
			time.sleep(0.1)
		self.mx_noise = statistics.stdev(mx_noise_vals)
		self.my_noise = statistics.stdev(my_noise_vals)
		self.mz_noise = statistics.stdev(mz_noise_vals)

		# Calibrate X and Y direction
		self.mqttClient.publish(TOPIC_STATUS,"Waiting for horizontal position...", qos = 0)
		data = self.imu.read()
		while (fabs(data["az"]) < ACC_IS_1G):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"X-Y-Magnetic calibration started. Turn horizontally!", qos = 0)
		# TODO: Drone should hover and turn automatically
		mx_vals = []
		my vals = []
		integrated_angle = 0
		lastTime = microseconds()
		while (math.fabs(integrated_angle) < 2.5*math.pi):
			data = self.imu.read()
			mx_vals.append(data["mx"])
			my_vals.append(data["my"])
			time = microseconds()
			integrated_angle = integrated_angle + data["gz"]*(time - lastTime)/1000000
			lastTime = time
			time.sleep(0.1)
		self.mqttClient.publish(TOPIC_STATUS,"X-Y-Magnetic calibration finished.", qos = 0)
		mx_max = np.max(mx_vals)-self.mx_noise
		mx_min = np.min(mx_vals)+self.mx_noise
		self.mx_bias = (mx_max-mx_min)/2
		self.mx_factor = self.Bh/(mx_max-self.mx_bias)
		my_max = np.max(my_vals)-self.my_noise
		my_min = np.min(my_vals)+self.my_noise
		self.my_bias = (my_max-my_min)/2
		self.my_factor = self.Bh/(my_max-self.my_bias)

		# Calibrate Z direction
		self.mqttClient.publish(TOPIC_STATUS,"Waiting for vertical position...", qos = 0)
		data = self.imu.read()
		while (fabs(data["az"]) > ACC_IS_ZERO):
			time.sleep(0.1)
			data = self.imu.read()
		self.mqttClient.publish(TOPIC_STATUS,"Z-Magnetic calibration started. Turn horizontally!", qos = 0)
		# TODO: How can the drone calibrate this by its own?
		mz_vals = []
		integrated_angle = 0
		lastTime = microseconds()
		while (math.fabs(integrated_angle) < 4*math.pi):
			data = self.imu.read()
			mz_vals.append(data["mz"])
			time = microseconds()
			integrated_angle = integrated_angle + data["gz"]*(time - lastTime)/1000000
			lastTime = time
			time.sleep(0.1)
		self.mqttClient.publish(TOPIC_STATUS,"Z-Magnetic calibration finished.", qos = 0)
		mz_max = np.max(mz_vals)-self.mz_noise
		mz_min = np.min(mz_vals)+self.mz_noise
		self.mz_bias = (mz_max-mz_min)/2
		self.mz_factor = self.Bh/(mz_max-self.mz_bias)

		# TODO: Data should be safed to calibration file!
		
		
	def IdentifyMotors():

		# Turn up all motors subsequently until movement is detected
		

	def LearnHovering():

		# 



