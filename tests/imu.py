# MPU9250 Basic Example Code
# by: Kris Winer
# date: April 1, 2014
# license: Beerware - Use this code however you'd like. If you 
# find it useful you can buy me a beer some time.
# 
# Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
# getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
# allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
# Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
# 
# SDA and SCL should have external pull-up resistors (to 3.3V).
# 10k resistors are on the EMSENSR-9250 breakout board.
# 
# Hardware setup:
# MPU9250 Breakout --------- Arduino
# VDD ---------------------- 3.3V
# VDDI --------------------- 3.3V
# SDA ----------------------- A4
# SCL ----------------------- A5
# GND ---------------------- GND

# See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
# above document the MPU9250 and MPU9150 are virtually identical but the latter has a different register map

##################

import time

#Magnetometer Registers
IMU_AK8963_ADDRESS    = 0x0C
IMU_WHO_AM_I_AK8963   = 0x00 # should return  = 0x48
IMU_INFO              = 0x01
IMU_AK8963_ST1        = 0x02  # data ready status bit 0
IMU_AK8963_XOUT_L	    = 0x03  # data
IMU_AK8963_XOUT_H	    = 0x04
IMU_K8963_YOUT_L	    = 0x05
IMU_AK8963_YOUT_H	    = 0x06
IMU_K8963_ZOUT_L	    = 0x07
IMU_AK8963_ZOUT_H	    = 0x08
IMU_AK8963_ST2        = 0x09  # Data overflow bit 3 and data read error status bit 2
IMU_AK8963_CNTL       = 0x0A  # Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
IMU_AK8963_ASTC       = 0x0C  # Self test control
IMU_AK8963_I2CDIS     = 0x0F  # I2C disable
IMU_AK8963_ASAX       = 0x10  # Fuse ROM x-axis sensitivity adjustment value
IMU_AK8963_ASAY       = 0x11  # Fuse ROM y-axis sensitivity adjustment value
IMU_AK8963_ASAZ       = 0x12  # Fuse ROM z-axis sensitivity adjustment value

IMU_SELF_TEST_X_GYRO  = 0x00                  
IMU_SELF_TEST_Y_GYRO  = 0x01                                                                          
IMU_SELF_TEST_Z_GYRO  = 0x02

IMU_SELF_TEST_X_ACCEL = 0x0D
IMU_SELF_TEST_Y_ACCEL = 0x0E    
IMU_SELF_TEST_Z_ACCEL = 0x0F

IMU_SELF_TEST_A       = 0x10

IMU_XG_OFFSET_H       = 0x13  # User-defined trim values for gyroscope
IMU_XG_OFFSET_L       = 0x14
IMU_YG_OFFSET_H       = 0x15
IMU_YG_OFFSET_L       = 0x16
IMU_ZG_OFFSET_H       = 0x17
IMU_ZG_OFFSET_L       = 0x18
IMU_SMPLRT_DIV        = 0x19
IMU_CONFIG            = 0x1A
IMU_GYRO_CONFIG       = 0x1B
IMU_ACCEL_CONFIG      = 0x1C
IMU_ACCEL_CONFIG2     = 0x1D
IMU_LP_ACCEL_ODR      = 0x1E   
IMU_WOM_THR           = 0x1F   

IMU_MOT_DUR           = 0x20  # Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
IMU_ZMOT_THR          = 0x21  # Zero-motion detection threshold bits [7:0]
IMU_ZRMOT_DUR         = 0x22  # Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

IMU_FIFO_EN           = 0x23
IMU_I2C_MST_CTRL      = 0x24   
IMU_I2C_SLV0_ADDR     = 0x25
IMU_I2C_SLV0_REG      = 0x26
IMU_I2C_SLV0_CTRL     = 0x27
IMU_I2C_SLV1_ADDR     = 0x28
IMU_I2C_SLV1_REG      = 0x29
IMU_I2C_SLV1_CTRL     = 0x2A
IMU_I2C_SLV2_ADDR     = 0x2B
IMU_I2C_SLV2_REG      = 0x2C
IMU_I2C_SLV2_CTRL     = 0x2D
IMU_I2C_SLV3_ADDR     = 0x2E
IMU_I2C_SLV3_REG      = 0x2F
IMU_I2C_SLV3_CTRL     = 0x30
IMU_I2C_SLV4_ADDR     = 0x31
IMU_I2C_SLV4_REG      = 0x32
IMU_I2C_SLV4_DO       = 0x33
IMU_I2C_SLV4_CTRL     = 0x34
IMU_I2C_SLV4_DI       = 0x35
IMU_I2C_MST_STATUS    = 0x36
IMU_INT_PIN_CFG       = 0x37
IMU_INT_ENABLE        = 0x38
IMU_DMP_INT_STATUS    = 0x39  # Check DMP interrupt
IMU_INT_STATUS        = 0x3A
IMU_ACCEL_XOUT_H      = 0x3B
IMU_ACCEL_XOUT_L      = 0x3C
IMU_ACCEL_YOUT_H      = 0x3D
IMU_ACCEL_YOUT_L      = 0x3E
IMU_ACCEL_ZOUT_H      = 0x3F
IMU_ACCEL_ZOUT_L      = 0x40
IMU_TEMP_OUT_H        = 0x41
IMU_TEMP_OUT_L        = 0x42
IMU_GYRO_XOUT_H       = 0x43
IMU_GYRO_XOUT_L       = 0x44
IMU_GYRO_YOUT_H       = 0x45
IMU_GYRO_YOUT_L       = 0x46
IMU_GYRO_ZOUT_H       = 0x47
IMU_GYRO_ZOUT_L       = 0x48
IMU_EXT_SENS_DATA_00  = 0x49
IMU_EXT_SENS_DATA_01  = 0x4A
IMU_EXT_SENS_DATA_02  = 0x4B
IMU_EXT_SENS_DATA_03  = 0x4C
IMU_EXT_SENS_DATA_04  = 0x4D
IMU_EXT_SENS_DATA_05  = 0x4E
IMU_EXT_SENS_DATA_06  = 0x4F
IMU_EXT_SENS_DATA_07  = 0x50
IMU_EXT_SENS_DATA_08  = 0x51
IMU_EXT_SENS_DATA_09  = 0x52
IMU_EXT_SENS_DATA_10  = 0x53
IMU_EXT_SENS_DATA_11  = 0x54
IMU_EXT_SENS_DATA_12  = 0x55
IMU_EXT_SENS_DATA_13  = 0x56
IMU_EXT_SENS_DATA_14  = 0x57
IMU_EXT_SENS_DATA_15  = 0x58
IMU_EXT_SENS_DATA_16  = 0x59
IMU_EXT_SENS_DATA_17  = 0x5A
IMU_EXT_SENS_DATA_18  = 0x5B
IMU_EXT_SENS_DATA_19  = 0x5C
IMU_EXT_SENS_DATA_20  = 0x5D
IMU_EXT_SENS_DATA_21  = 0x5E
IMU_EXT_SENS_DATA_22  = 0x5F
IMU_EXT_SENS_DATA_23  = 0x60
IMU_MOT_DETECT_STATUS = 0x61
IMU_I2C_SLV0_DO       = 0x63
IMU_I2C_SLV1_DO       = 0x64
IMU_I2C_SLV2_DO       = 0x65
IMU_I2C_SLV3_DO       = 0x66
IMU_I2C_MST_DELAY_CTRL= 0x67
IMU_SIGNAL_PATH_RESET = 0x68
IMU_MOT_DETECT_CTRL   = 0x69
IMU_USER_CTRL         = 0x6A  # Bit 7 enable DMP, bit 3 reset DMP
IMU_PWR_MGMT_1        = 0x6B # Device defaults to the SLEEP mode
IMU_PWR_MGMT_2        = 0x6C
IMU_DMP_BANK          = 0x6D  # Activates a specific bank in the DMP
IMU_DMP_RW_PNT        = 0x6E  # Set read/write pointer to a specific start address in specified DMP bank
IMU_DMP_REG           = 0x6F  # Register in DMP from which to read or to which to write
IMU_DMP_REG_1         = 0x70
IMU_DMP_REG_2         = 0x71 
IMU_FIFO_COUNTH       = 0x72
IMU_FIFO_COUNTL       = 0x73
IMU_FIFO_R_W          = 0x74
IMU_WHO_AM_I_MPU9250  = 0x75 # Should return  = 0x71
IMU_XA_OFFSET_H       = 0x77
IMU_XA_OFFSET_L       = 0x78
IMU_YA_OFFSET_H       = 0x7A
IMU_YA_OFFSET_L       = 0x7B
IMU_ZA_OFFSET_H       = 0x7D
IMU_ZA_OFFSET_L       = 0x7E

# Using the MSENSR-9250 breakout board, ADO is set to 0 
# Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1

IMU_MPU9250_ADDRESS   = 0x68   # Device address when ADO = 0
IMU_AK8963_ADDRESS    = 0x0C   #  Address of magnetometer 

IMU_AHRS              = False   # set to false for basic data read

# Set initial input parameters
# IMU_ASCALE
IMU_AFS_2G  = 0
IMU_AFS_4G  = 1
IMU_AFS_8G  = 2
IMU_AFS_16G = 3

#IMU_GSCALE
IMU_GFS_250DPS  = 0
IMU_GFS_500DPS  = 1
IMU_GFS_1000DPS = 2
IMU_GFS_2000DPS = 3

#IMU_MSCALE 
IMU_MFS_14BITS = 0 # 0.6 mG per LSB
IMU_MFS_16BITS = 1 # 0.15 mG per LSB


###################################################################################################

# SETTINGS
# Specify sensor full scale
IMU_ASCALE  = IMU_AFS_4G
IMU_GSCALE  = IMU_GFS_250DPS
IMU_MSCALE  = IMU_MFS_16BITS # Choose either 14-bit or 16-bit magnetometer resolution
IMU_MMODE   = 0x02        # 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read


###################################################################################################

def IMU_setup(bus):

  print(IMU_GSCALE)
  
  # Read the WHO_AM_I register, this is a good test of communication
  c = readByte(bus,IMU_MPU9250_ADDRESS, IMU_WHO_AM_I_MPU9250)  # Read WHO_AM_I register for MPU-9250
  print("MPU9250 - I AM " + str(c) + ", I should be " + str(0x71))
  time.sleep(1)

  if (c == 0x71): 

    print("MPU9250 is online...")
    
    # Start by performing self test and reporting values
    SelfTest = MPU9250SelfTest(bus) 
    print("x-axis self test: acceleration trim within : " + str(SelfTest[0]) + "% of factory value")
    print("y-axis self test: acceleration trim within : " + str(SelfTest[1]) + "% of factory value")
    print("z-axis self test: acceleration trim within : " + str(SelfTest[2]) + "% of factory value")
    print("x-axis self test: gyration trim within : " + str(SelfTest[3]) + "% of factory value")
    print("y-axis self test: gyration trim within : " + str(SelfTest[4]) + "% of factory value")
    print("z-axis self test: gyration trim within : " + str(SelfTest[5]) + "% of factory value")

    # Calibrate gyro and accelerometers, load biases in bias registers
    gyroBias, accelBias = calibrateMPU9250(bus) 
    print("acceleration bias im mg")
    print("x: " + str(1000*accelBias[0]))
    print("y: " + str(1000*accelBias[1]))
    print("z: " + str(1000*accelBias[2]))
    print("gyro bias im °/s")
    print("x: " + str(gyroBias[0])) 
    print("y: " + str(gyroBias[1])) 
    print("z: " + str(gyroBias[2]))    
    time.sleep(1)

    # Initialize device for active mode read of acclerometer, gyroscope, and temperature
    initMPU9250(bus) 
    print("MPU9250 initialized for active data mode....") 

    # Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    d = readByte(bus,IMU_AK8963_ADDRESS, IMU_WHO_AM_I_AK8963)  # Read WHO_AM_I register for AK8963
    print("AK8963 - I AM " + str(d) + ", I should be " + str(0x48))
    time.sleep(1)

    # Get magnetometer calibration from AK8963 ROM
    # Initialize device for active mode read of magnetometer
    magCalibration = initAK8963(bus)
    print("AK8963 initialized for active data mode....") 

    print("Calibration values: ")
    print("X-Axis sensitivity adjustment value " + str(magCalibration[0]))
    print("Y-Axis sensitivity adjustment value " + str(magCalibration[1]))
    print("Z-Axis sensitivity adjustment value " + str(magCalibration[2]))
    time.sleep(1)  
    return {'accelBias': accelBias, 'gyroBias': gyroBias, 'magCalibration': magCalibration}

  else:
    print("Could not connect to MPU9250!")
    return 0

    

##################/


def IMU_read(bus):
  # If intPin goes high, all data registers have new data
  if (readByte(bus,IMU_MPU9250_ADDRESS, IMU_INT_STATUS) & 0x01):  # On interrupt, check if data ready interrupt
    accelCount = readAccelData(bus)  # Read the x/y/z adc values
    aRes = getAres()

    #print(accelCount)
    # Now we'll calculate the accleration value into actual g's
    ax = twos_comp(accelCount[0],16)*aRes # - accelBias[0]  # get actual g value, this depends on scale being set
    ay = twos_comp(accelCount[1],16)*aRes # - accelBias[1]   
    az = twos_comp(accelCount[2],16)*aRes # - accelBias[2]  
   
    gyroCount = readGyroData(bus)  # Read the x/y/z adc values
    gRes = getGres()

    #print(gyroCount)
    # Calculate the gyro value into actual degrees per second
    gx = twos_comp(gyroCount[0],16)*gRes  # get actual gyro value (deg/sec), this depends on scale being set
    gy = twos_comp(gyroCount[1],16)*gRes  
    gz = twos_comp(gyroCount[2],16)*gRes   
  
    magCount = readMagData(bus)  # Read the x/y/z adc values
    mRes = getMres()
    magbias = [0, 0, 0]
    magbias[0] = 0 #+470.0  # User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = 0 #+120.0  # User environmental y-axis correction in milliGauss
    magbias[2] = 0 #+125.0  # User environmental z-axis correction in milliGauss

    #print(magCount)
    magCalibration = [1, 1, 1]
    # Calculate the magnetometer values in milliGauss
    # Include factory calibration per data sheet and user environmental corrections
    mx = twos_comp(magCount[0],16)*mRes*magCalibration[0] - magbias[0]  # get actual magnetometer value, this depends on scale being set
    my = twos_comp(magCount[1],16)*mRes*magCalibration[1] - magbias[1]  
    mz = twos_comp(magCount[2],16)*mRes*magCalibration[2] - magbias[2]   
  
    # Get IMU temperature info
    tempCount = readTempData(bus)  # Read the adc values
    temperature = tempCount / 333.87 + 21.0 # Temperature in degrees Centigrade °C

    return {'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz, 'mx': mx, 'my': my, 'mz': mz, 'temperature': temperature}

  else:
    return 0


def twos_comp(val, bits):
    """compute the 2's compliment of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val
   

#===================================================================================================================
#====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
#===================================================================================================================


def getMres():
 	# Possible magnetometer scales (and their register bit settings) are:
	# 14 bit resolution (0) and 16 bit resolution (1)
  switcher = {
      IMU_MFS_14BITS: 10.0*4912.0/8190.0,   # Proper scale to return milliGauss
      IMU_MFS_16BITS: 10.0*4912.0/32760.0,  # Proper scale to return milliGauss
  }
  return switcher.get(IMU_MSCALE,0)


def getGres():
 	# Possible gyro scales (and their register bit settings) are:
	# 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
  # Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  switcher = {
    IMU_GFS_250DPS: 250.0/32768.0,
    IMU_GFS_500DPS: 500.0/32768.0,
    IMU_GFS_1000DPS: 1000.0/32768.0,
    IMU_GFS_2000DPS: 2000.0/32768.0,
  }
  return switcher.get(IMU_GSCALE,0)


def getAres():
 	# Possible accelerometer scales (and their register bit settings) are:
	# 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
  # Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  switcher = {
    IMU_AFS_2G: 2.0/32768.0,
    IMU_AFS_4G: 4.0/32768.0,
    IMU_AFS_8G: 8.0/32768.0,
    IMU_AFS_16G: 16.0/32768.0,
  }
  return switcher.get(IMU_ASCALE,0)


#################


def readAccelData(bus):
  rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_XOUT_H, 6)  # Read the six raw data registers into data array
  destination = [0, 0, 0]
  destination[0] = (rawData[0] << 8) | rawData[1]   # Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (rawData[2] << 8) | rawData[3]   
  destination[2] = (rawData[4] << 8) | rawData[5]  
  return destination


def readGyroData(bus):
  rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_XOUT_H, 6)  # Read the six raw data registers sequentially into data array
  destination = [0, 0, 0]
  destination[0] = (rawData[0] << 8) | rawData[1]   # Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (rawData[2] << 8) | rawData[3]   
  destination[2] = (rawData[4] << 8) | rawData[5]  
  return destination


def readMagData(bus):
  destination = [0, 0, 0]
  if (readByte(bus,IMU_AK8963_ADDRESS, IMU_AK8963_ST1) & 0x01): # wait for magnetometer data ready bit to be set
    rawData = readBytes(bus,IMU_AK8963_ADDRESS, IMU_AK8963_XOUT_L, 7)  # Read the six raw data and ST2 registers sequentially into data array
    c = rawData[6] # End data read by reading ST2 register
    if((c & 0x08) != True): # Check if magnetic sensor overflow set, if not then report data
      destination[0] = (rawData[1] << 8) | rawData[0]   # Turn the MSB and LSB into a signed 16-bit value
      destination[1] = (rawData[3] << 8) | rawData[2]   # Data stored as little Endian
      destination[2] = (rawData[5] << 8) | rawData[4]  
  return destination

def readTempData(bus):
  rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_TEMP_OUT_H, 2)  # Read the two raw data registers sequentially into data array 
  return (rawData[0] << 8) | rawData[1]   # Turn the MSB and LSB into a 16-bit value


#################/

       
def initAK8963(bus):
  # First extract the factory calibration for each magnetometer axis
  writeByte(bus,IMU_AK8963_ADDRESS, IMU_AK8963_CNTL, 0x00) # Power down magnetometer  
  time.sleep(0.01)
  writeByte(bus,IMU_AK8963_ADDRESS, IMU_AK8963_CNTL, 0x0F) # Enter Fuse ROM access mode
  time.sleep(0.01)
  rawData = readBytes(bus,IMU_AK8963_ADDRESS, IMU_AK8963_ASAX, 3)  # Read the x-, y-, and z-axis calibration values
  destination = [0, 0, 0]
  destination[0] =  (rawData[0] - 128)/256. + 1.   # Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (rawData[1] - 128)/256. + 1.  
  destination[2] =  (rawData[2] - 128)/256. + 1. 
  writeByte(bus,IMU_AK8963_ADDRESS, IMU_AK8963_CNTL, 0x00) # Power down magnetometer  
  time.sleep(0.01)
  # Configure the magnetometer for continuous read and highest resolution
  # set IMU_MSCALE bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  # and enable continuous mode data acquisition IMU_MMODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(bus,IMU_AK8963_ADDRESS, IMU_AK8963_CNTL, IMU_MSCALE << 4 | IMU_MMODE) # Set magnetometer data resolution and sample ODR
  time.sleep(0.01)
  return destination


#################/


def initMPU9250(bus):

  # wake up device
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_PWR_MGMT_1, 0x00) # Clear sleep mode bit (6), enable all sensors 
  time.sleep(0.1) # Wait for all registers to reset 

  # get stable time source
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_PWR_MGMT_1, 0x01)  # Auto select clock source to be PLL gyroscope reference if ready else
  time.sleep(0.2)
  
  # Configure Gyro and Thermometer
  # Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively 
  # minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  # be higher than 1 / 0.0059 = 170 Hz
  # DLPF_CFG = bits 2:0 = 011 this limits the sample rate to 1000 Hz for both
  # With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_CONFIG, 0x03)  

  # Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_SMPLRT_DIV, 0x04)  # Use a 200 Hz rate a rate consistent with the filter update rate determined inset in CONFIG above
 
  # Set gyroscope full scale range
  # Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  c = readByte(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_CONFIG) # get current GYRO_CONFIG register value
  # c = c & ~0xE0 # Clear self-test bits [7:5] 
  c = c & ~0x02 # Clear Fchoice bits [1:0] 
  c = c & ~0x18 # Clear AFS bits [4:3]
  c = c | IMU_GSCALE << 3 # Set full scale range for the gyro
  # c =| 0x00 # Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_CONFIG, c ) # Write new GYRO_CONFIG value to register
  
  # Set accelerometer full-scale range configuration
  c = readByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG) # get current ACCEL_CONFIG register value
  # c = c & ~0xE0 # Clear self-test bits [7:5] 
  c = c & ~0x18  # Clear AFS bits [4:3]
  c = c | IMU_ASCALE << 3 # Set full scale range for the accelerometer 
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG, c) # Write new ACCEL_CONFIG register value

  # Set accelerometer sample rate configuration
  # It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  # accel_fchoice_b bit [3] in this case the bandwidth is 1.13 kHz
  c = readByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG2) # get current ACCEL_CONFIG2 register value
  c = c & ~0x0F # Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03  # Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG2, c) # Write new ACCEL_CONFIG2 register value
  # The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
  # but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  # Configure Interrupts and Bypass Enable
  # Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  # clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  # can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_INT_PIN_CFG, 0x22)    
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_INT_ENABLE, 0x01)  # Enable data ready (bit 0) interrupt
  time.sleep(0.1)


##########################/  


# Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
# of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
def calibrateMPU9250(bus): 
  
  # reset device
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_PWR_MGMT_1, 0x80) # Write a one to bit 7 reset bit toggle reset device
  time.sleep(0.1)
   
  # get stable time source Auto select clock source to be PLL gyroscope reference if ready 
  # else use the internal oscillator, bits 2:0 = 001
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_PWR_MGMT_1, 0x01)  
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_PWR_MGMT_2, 0x00)
  time.sleep(0.2)                                   

  # Configure device for bias calculation
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_INT_ENABLE, 0x00)   # Disable all interrupts
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_FIFO_EN, 0x00)      # Disable FIFO
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_PWR_MGMT_1, 0x00)   # Turn on internal clock source
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_I2C_MST_CTRL, 0x00) # Disable I2C master
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_USER_CTRL, 0x00)    # Disable FIFO and I2C master modes
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_USER_CTRL, 0x0C)    # Reset FIFO and DMP
  time.sleep(0.015)

  # Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_CONFIG, 0x01)      # Set low-pass filter to 188 Hz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_SMPLRT_DIV, 0x00)  # Set sample rate to 1 kHz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_CONFIG, 0x00)  # Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG, 0x00) # Set accelerometer full-scale to 2 g, maximum sensitivity

  gyrosensitivity  = 131   # = 131 LSB/degrees/sec
  accelsensitivity = 16384  # = 16384 LSB/g

    # Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_USER_CTRL, 0x40)   # Enable FIFO  
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_FIFO_EN, 0x78)     # Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  time.sleep(0.04) # accumulate 40 samples in 40 milliseconds = 480 bytes

  # At end of sample accumulation, turn off FIFO sensor read
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_FIFO_EN, 0x00)        # Disable gyro and accelerometer sensors for FIFO
  data = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_FIFO_COUNTH, 2) # read FIFO sample count
  fifo_count = (data[0] << 8) | data[1]
  packet_count = fifo_count//12 # How many sets of full gyro and accelerometer data for averaging

  accel_temp = [0, 0, 0]
  gyro_temp = [0, 0, 0]
  gyro_bias  = [0, 0, 0]
  accel_bias = [0, 0, 0]
  for ii in range(packet_count):
    data = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_FIFO_R_W, 12) # read data for averaging
    accel_temp[0] =  twos_comp(((data[0] << 8) | data[1]  ),16)   # Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] =  twos_comp(((data[2] << 8) | data[3]  ),16) 
    accel_temp[2] =  twos_comp(((data[4] << 8) | data[5]  ),16)     
    gyro_temp[0]  =  twos_comp(((data[6] << 8) | data[7]  ),16) 
    gyro_temp[1]  =  twos_comp(((data[8] << 8) | data[9]  ),16) 
    gyro_temp[2]  =  twos_comp(((data[10] << 8) | data[11]),16) 
    accel_bias[0] += accel_temp[0] # Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += accel_temp[1]
    accel_bias[2] += accel_temp[2]
    gyro_bias[0]  += gyro_temp[0]
    gyro_bias[1]  += gyro_temp[1]
    gyro_bias[2]  += gyro_temp[2]
            
  accel_bias[0] /= packet_count # Normalize sums to get average count biases
  accel_bias[1] /= packet_count
  accel_bias[2] /= packet_count
  gyro_bias[0]  /= packet_count
  gyro_bias[1]  /= packet_count
  gyro_bias[2]  /= packet_count
    
  if (accel_bias[2] > 0): # originally 0L
    accel_bias[2] -= accelsensitivity  # Remove gravity from the z-axis accelerometer bias calculation
  else:
    accel_bias[2] += accelsensitivity
   
  # Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data = [0, 0, 0, 0, 0, 0]
  data[0] = (-int(gyro_bias[0])//4  >> 8) & 0xFF # Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-int(gyro_bias[0])//4)       & 0xFF # Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-int(gyro_bias[1])//4  >> 8) & 0xFF
  data[3] = (-int(gyro_bias[1])//4)       & 0xFF
  data[4] = (-int(gyro_bias[2])//4  >> 8) & 0xFF
  data[5] = (-int(gyro_bias[2])//4)       & 0xFF
  
  # Push gyro biases to hardware registers
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XG_OFFSET_H, data[0])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XG_OFFSET_L, data[1])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YG_OFFSET_H, data[2])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YG_OFFSET_L, data[3])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZG_OFFSET_H, data[4])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZG_OFFSET_L, data[5])
  
  # Output scaled gyro biases for display in the main program
  dest1 = [0, 0, 0]
  dest1[0] = gyro_bias[0]/gyrosensitivity  
  dest1[1] = gyro_bias[1]/gyrosensitivity
  dest1[2] = gyro_bias[2]/gyrosensitivity

  # Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  # factory trim values which must be added to the calculated accelerometer biases on boot up these registers will hold
  # non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  # compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  # the accelerometer biases calculated above must be divided by 8.

  accel_bias_reg = [0, 0, 0]
  data = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_XA_OFFSET_H, 2) # Read factory accelerometer trim values
  accel_bias_reg[0] = twos_comp(((data[0] << 8) | data[1]),16)
  data = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_YA_OFFSET_H, 2)
  accel_bias_reg[1] = twos_comp(((data[0] << 8) | data[1]),16)
  data = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_ZA_OFFSET_H, 2)
  accel_bias_reg[2] = twos_comp(((data[0] << 8) | data[1]),16)
  
  mask = 1 # originally 1uL - Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  mask_bit = [0, 0, 0] # Define array to hold mask bit for each accelerometer bias axis
  
  for ii in range(3):
    if ((accel_bias_reg[ii] & mask)): 
      mask_bit[ii] = 0x01 # If temperature compensation bit is set, record that fact in mask_bit
  
  # Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8) # Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8)
  accel_bias_reg[2] -= (accel_bias[2]/8)

  data = [0, 0, 0, 0, 0, 0]
  data[0] = (int(accel_bias_reg[0]) >> 8) & 0xFF
  data[1] = int(accel_bias_reg[0])      & 0xFF
  data[1] = data[1] | mask_bit[0] # preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (int(accel_bias_reg[1]) >> 8) & 0xFF
  data[3] = int(accel_bias_reg[1])      & 0xFF
  data[3] = data[3] | mask_bit[1] # preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (int(accel_bias_reg[2]) >> 8) & 0xFF
  data[5] = int(accel_bias_reg[2])      & 0xFF
  data[5] = data[5] | mask_bit[2] # preserve temperature compensation bit when writing back to accelerometer bias registers
 
  # Apparently this is not working for the acceleration biases in the MPU-9250
  # Are we handling the temperature correction bit properly?
  # Push accelerometer biases to hardware registers
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XA_OFFSET_H, data[0])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XA_OFFSET_L, data[1])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YA_OFFSET_H, data[2])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YA_OFFSET_L, data[3])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZA_OFFSET_H, data[4])
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZA_OFFSET_L, data[5])

  # Output scaled accelerometer biases for display in the main program
  dest2 = [0, 0, 0]  
  dest2[0] = accel_bias[0]/accelsensitivity 
  dest2[1] = accel_bias[1]/accelsensitivity
  dest2[2] = accel_bias[2]/accelsensitivity

  return (dest1, dest2)



def IMU_reset(bus):
  # Reset acc biases
  #writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XA_OFFSET_H, 0x00)
  #writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XA_OFFSET_L, 0x00)
  #writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YA_OFFSET_H, 0x00)
  #writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YA_OFFSET_L, 0x00)
  #writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZA_OFFSET_H, 0x00)
  #writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZA_OFFSET_L, 0x00)
  # Reset gyro biases
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XG_OFFSET_H, 0x00)
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_XG_OFFSET_L, 0x00)
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YG_OFFSET_H, 0x00)
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_YG_OFFSET_L, 0x00)
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZG_OFFSET_H, 0x00)
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ZG_OFFSET_L, 0x00)
  print("IMU Reseted")

#########################/

   
# Accelerometer and gyroscope self test check calibration wrt factory settings
def MPU9250SelfTest(bus): # Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass

  FS = 0
   
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_SMPLRT_DIV, 0x00)    # Set gyro sample rate to 1 kHz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_CONFIG, 0x02)        # Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_CONFIG, FS<<3)  # Set full scale range for the gyro to 250 dps
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG2, 0x02) # Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG, FS<<3) # Set full scale range for the accelerometer to 2 g

  aAvg = [0, 0, 0]
  gAvg = [0, 0, 0]
  for ii in range(200):  # get average current values of gyro and acclerometer
    rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_XOUT_H, 6)        # Read the six raw data registers into data array
    aAvg[0] += twos_comp(((rawData[0] << 8) | rawData[1]) ,16)  # Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += twos_comp(((rawData[2] << 8) | rawData[3]) ,16)  
    aAvg[2] += twos_comp(((rawData[4] << 8) | rawData[5]) ,16) 
    rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_XOUT_H, 6)       # Read the six raw data registers sequentially into data array
    gAvg[0] += twos_comp(((rawData[0] << 8) | rawData[1]) ,16)  # Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += twos_comp(((rawData[2] << 8) | rawData[3]) ,16)  
    gAvg[2] += twos_comp(((rawData[4] << 8) | rawData[5]) ,16) 
  
  for ii in range(3):  # Get average of 200 values and store as average current readings
    aAvg[ii] /= 200
    gAvg[ii] /= 200
  
  # Configure the accelerometer for self-test
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG, 0xE0) # Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_CONFIG,  0xE0) # Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  time.sleep(0.025)  # Delay a while to let the device stabilize

  aSTAvg = [0, 0, 0]
  gSTAvg = [0, 0, 0]
  for ii in range(200):  # get average self-test values of gyro and acclerometer
    rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_XOUT_H, 6)  # Read the six raw data registers into data array
    aSTAvg[0] += twos_comp(((rawData[0] << 8) | rawData[1]),16)   # Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += twos_comp(((rawData[2] << 8) | rawData[3]),16)   
    aSTAvg[2] += twos_comp(((rawData[4] << 8) | rawData[5]),16)  
    rawData = readBytes(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_XOUT_H, 6)  # Read the six raw data registers sequentially into data array
    gSTAvg[0] += twos_comp(((rawData[0] << 8) | rawData[1]),16)   # Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += twos_comp(((rawData[2] << 8) | rawData[3]),16)   
    gSTAvg[2] += twos_comp(((rawData[4] << 8) | rawData[5]),16)  
  
  for ii in range(3):  # Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200
    gSTAvg[ii] /= 200 
  
  # Configure the gyro and accelerometer for normal operation
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_ACCEL_CONFIG, 0x00)  
  writeByte(bus,IMU_MPU9250_ADDRESS, IMU_GYRO_CONFIG,  0x00)  
  time.sleep(0.025)  # Delay a while to let the device stabilize

  # Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest = [0, 0, 0, 0, 0, 0]
  selfTest[0] = readByte(bus,IMU_MPU9250_ADDRESS, IMU_SELF_TEST_X_ACCEL) # X-axis accel self-test results
  selfTest[1] = readByte(bus,IMU_MPU9250_ADDRESS, IMU_SELF_TEST_Y_ACCEL) # Y-axis accel self-test results
  selfTest[2] = readByte(bus,IMU_MPU9250_ADDRESS, IMU_SELF_TEST_Z_ACCEL) # Z-axis accel self-test results
  selfTest[3] = readByte(bus,IMU_MPU9250_ADDRESS, IMU_SELF_TEST_X_GYRO)  # X-axis gyro self-test results
  selfTest[4] = readByte(bus,IMU_MPU9250_ADDRESS, IMU_SELF_TEST_Y_GYRO)  # Y-axis gyro self-test results
  selfTest[5] = readByte(bus,IMU_MPU9250_ADDRESS, IMU_SELF_TEST_Z_GYRO)  # Z-axis gyro self-test results

  # Retrieve factory self-test value from self-test code reads
  factoryTrim = [0, 0, 0, 0, 0, 0]
  factoryTrim[0] = (2620//1<<FS)*(pow( 1.01 , (selfTest[0] - 1.0) )) # FT[Xa] factory trim calculation
  factoryTrim[1] = (2620//1<<FS)*(pow( 1.01 , (selfTest[1] - 1.0) )) # FT[Ya] factory trim calculation
  factoryTrim[2] = (2620//1<<FS)*(pow( 1.01 , (selfTest[2] - 1.0) )) # FT[Za] factory trim calculation
  factoryTrim[3] = (2620//1<<FS)*(pow( 1.01 , (selfTest[3] - 1.0) )) # FT[Xg] factory trim calculation
  factoryTrim[4] = (2620//1<<FS)*(pow( 1.01 , (selfTest[4] - 1.0) )) # FT[Yg] factory trim calculation
  factoryTrim[5] = (2620//1<<FS)*(pow( 1.01 , (selfTest[5] - 1.0) )) # FT[Zg] factory trim calculation
 
  # Report results as a ratio of (STR - FT)/FT the change from Factory Trim of the Self-Test Response
  # To get percent, must multiply by 100
  destination = [0, 0, 0, 0, 0, 0]
  for i in range(3):
    destination[i]   = 100.0*((aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.   # Report percent differences
    destination[i+3] = 100.0*((gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100. # Report percent differences

  return destination


########################

        
def writeByte(bus,address, subAddress, data):
  bus.write_byte_data(address, subAddress, data)


def readByte(bus,address, subAddress):
  return bus.read_byte_data(address, subAddress)


def readBytes(bus,address, subAddress, count):
  return bus.read_i2c_block_data(address, subAddress, count)


###########################

