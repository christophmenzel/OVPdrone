/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

from enum import Enum

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
AK8963_ADDRESS    = 0x0C
WHO_AM_I_AK8963   = 0x00 // should return  = 0x48
INFO              = 0x01
AK8963_ST1        = 0x02  // data ready status bit 0
AK8963_XOUT_L	    = 0x03  // data
AK8963_XOUT_H	    = 0x04
AK8963_YOUT_L	    = 0x05
AK8963_YOUT_H	    = 0x06
AK8963_ZOUT_L	    = 0x07
AK8963_ZOUT_H	    = 0x08
AK8963_ST2        = 0x09  // Data overflow bit 3 and data read error status bit 2
AK8963_CNTL       = 0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
AK8963_ASTC       = 0x0C  // Self test control
AK8963_I2CDIS     = 0x0F  // I2C disable
AK8963_ASAX       = 0x10  // Fuse ROM x-axis sensitivity adjustment value
AK8963_ASAY       = 0x11  // Fuse ROM y-axis sensitivity adjustment value
AK8963_ASAZ       = 0x12  // Fuse ROM z-axis sensitivity adjustment value

SELF_TEST_X_GYRO  = 0x00                  
SELF_TEST_Y_GYRO  = 0x01                                                                          
SELF_TEST_Z_GYRO  = 0x02

SELF_TEST_X_ACCEL  = 0x0D
SELF_TEST_Y_ACCEL  = 0x0E    
SELF_TEST_Z_ACCEL  = 0x0F

SELF_TEST_A       = 0x10

XG_OFFSET_H       = 0x13  // User-defined trim values for gyroscope
XG_OFFSET_L       = 0x14
YG_OFFSET_H       = 0x15
YG_OFFSET_L       = 0x16
ZG_OFFSET_H       = 0x17
ZG_OFFSET_L       = 0x18
SMPLRT_DIV        = 0x19
CONFIG            = 0x1A
GYRO_CONFIG       = 0x1B
ACCEL_CONFIG      = 0x1C
ACCEL_CONFIG2     = 0x1D
LP_ACCEL_ODR      = 0x1E   
WOM_THR           = 0x1F   

MOT_DUR           = 0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
ZMOT_THR          = 0x21  // Zero-motion detection threshold bits [7:0]
ZRMOT_DUR         = 0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

FIFO_EN           = 0x23
I2C_MST_CTRL      = 0x24   
I2C_SLV0_ADDR     = 0x25
I2C_SLV0_REG      = 0x26
I2C_SLV0_CTRL     = 0x27
I2C_SLV1_ADDR     = 0x28
I2C_SLV1_REG      = 0x29
I2C_SLV1_CTRL     = 0x2A
I2C_SLV2_ADDR     = 0x2B
I2C_SLV2_REG      = 0x2C
I2C_SLV2_CTRL     = 0x2D
I2C_SLV3_ADDR     = 0x2E
I2C_SLV3_REG      = 0x2F
I2C_SLV3_CTRL     = 0x30
I2C_SLV4_ADDR     = 0x31
I2C_SLV4_REG      = 0x32
I2C_SLV4_DO       = 0x33
I2C_SLV4_CTRL     = 0x34
I2C_SLV4_DI       = 0x35
I2C_MST_STATUS    = 0x36
INT_PIN_CFG       = 0x37
INT_ENABLE        = 0x38
DMP_INT_STATUS    = 0x39  // Check DMP interrupt
INT_STATUS        = 0x3A
ACCEL_XOUT_H      = 0x3B
ACCEL_XOUT_L      = 0x3C
ACCEL_YOUT_H      = 0x3D
ACCEL_YOUT_L      = 0x3E
ACCEL_ZOUT_H      = 0x3F
ACCEL_ZOUT_L      = 0x40
TEMP_OUT_H        = 0x41
TEMP_OUT_L        = 0x42
GYRO_XOUT_H       = 0x43
GYRO_XOUT_L       = 0x44
GYRO_YOUT_H       = 0x45
GYRO_YOUT_L       = 0x46
GYRO_ZOUT_H       = 0x47
GYRO_ZOUT_L       = 0x48
EXT_SENS_DATA_00  = 0x49
EXT_SENS_DATA_01  = 0x4A
EXT_SENS_DATA_02  = 0x4B
EXT_SENS_DATA_03  = 0x4C
EXT_SENS_DATA_04  = 0x4D
EXT_SENS_DATA_05  = 0x4E
EXT_SENS_DATA_06  = 0x4F
EXT_SENS_DATA_07  = 0x50
EXT_SENS_DATA_08  = 0x51
EXT_SENS_DATA_09  = 0x52
EXT_SENS_DATA_10  = 0x53
EXT_SENS_DATA_11  = 0x54
EXT_SENS_DATA_12  = 0x55
EXT_SENS_DATA_13  = 0x56
EXT_SENS_DATA_14  = 0x57
EXT_SENS_DATA_15  = 0x58
EXT_SENS_DATA_16  = 0x59
EXT_SENS_DATA_17  = 0x5A
EXT_SENS_DATA_18  = 0x5B
EXT_SENS_DATA_19  = 0x5C
EXT_SENS_DATA_20  = 0x5D
EXT_SENS_DATA_21  = 0x5E
EXT_SENS_DATA_22  = 0x5F
EXT_SENS_DATA_23  = 0x60
MOT_DETECT_STATUS = 0x61
I2C_SLV0_DO       = 0x63
I2C_SLV1_DO       = 0x64
I2C_SLV2_DO       = 0x65
I2C_SLV3_DO       = 0x66
I2C_MST_DELAY_CTRL= 0x67
SIGNAL_PATH_RESET = 0x68
MOT_DETECT_CTRL   = 0x69
USER_CTRL         = 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
PWR_MGMT_1        = 0x6B // Device defaults to the SLEEP mode
PWR_MGMT_2        = 0x6C
DMP_BANK          = 0x6D  // Activates a specific bank in the DMP
DMP_RW_PNT        = 0x6E  // Set read/write pointer to a specific start address in specified DMP bank
DMP_REG           = 0x6F  // Register in DMP from which to read or to which to write
DMP_REG_1         = 0x70
DMP_REG_2         = 0x71 
FIFO_COUNTH       = 0x72
FIFO_COUNTL       = 0x73
FIFO_R_W          = 0x74
WHO_AM_I_MPU9250  = 0x75 // Should return  = 0x71
XA_OFFSET_H       = 0x77
XA_OFFSET_L       = 0x78
YA_OFFSET_H       = 0x7A
YA_OFFSET_L       = 0x7B
ZA_OFFSET_H       = 0x7D
ZA_OFFSET_L       = 0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1

MPU9250_ADDRESS   = 0x68   // Device address when ADO = 0
AK8963_ADDRESS    = 0x0C   //  Address of magnetometer 

AHRS              = true   // set to false for basic data read
SerialDebug       = true   // set to true to get Serial output for debugging

// Set initial input parameters
// Ascale
AFS_2G  = 0
AFS_4G  = 1
AFS_8G  = 2
AFS_16G = 3

//Gscale
GFS_250DPS  = 0
GFS_500DPS  = 1
GFS_1000DPS = 2
GFS_2000DPS = 3

//Mscale 
MFS_14BITS = 0 // 0.6 mG per LSB
MFS_16BITS = 1 // 0.15 mG per LSB

// Specify sensor full scale
Ascale  = AFS_2G
Gscale  = GFS_250DPS
Mscale  = MFS_16BITS // Choose either 14-bit or 16-bit magnetometer resolution
Mmode   = 0x02        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
// float aRes, gRes, mRes      // scale resolutions per LSB for the sensors
  
// Pin definitions
intPin  = 12  // These can be changed, 2 and 3 are the Arduinos ext int pins
myLed   = 13 // Set up pin 13 led for toggling

/*
int16_t accelCount[3]  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3]   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3]    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}      // Bias corrections for gyro and accelerometer
int16_t tempCount      // temperature raw count output
float   temperature    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6]    // holds results of gyro and accelerometer self test
*/

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
GyroMeasError = PI * (40.0f / 180.0f)   // gyroscope measurement error in rads/s (start at 40 deg/s)
GyroMeasDrift = PI * (0.0f  / 180.0f)   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
beta = sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
Kp = 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
Ki = 0.0f

delt_t = 0 // used to control display output rate
count = 0
sumCount = 0 // used to control display output rate
deltat = 0.0f
sum = 0.0f        // integration interval for both filter schemes
lastUpdate = 0
firstUpdate = 0 // used to calculate integration interval
Now = 0        // used to calculate integration interval

// float ax, ay, az, gx, gy, gz, mx, my, mz // variables to hold latest sensor data values 
q[4] = {1.0f, 0.0f, 0.0f, 0.0f}    // vector to hold quaternion
eInt[3] = {0.0f, 0.0f, 0.0f}       // vector to hold integral error for Mahony method


////////////////////////////////////


def setup():

  // Read the WHO_AM_I register, this is a good test of communication
  c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250)  // Read WHO_AM_I register for MPU-9250
  print("MPU9250 - I AM " + c + ", I should be " + 0x71)
  time.sleep(1)

  if (c == 0x71): // WHO_AM_I should always be 0x68 

    print("MPU9250 is online...")
    
    // Start by performing self test and reporting values
    SelfTest = MPU9250SelfTest() 
    print("x-axis self test: acceleration trim within : " + SelfTest[0] + "% of factory value")
    print("y-axis self test: acceleration trim within : " + SelfTest[1] + "% of factory value")
    print("z-axis self test: acceleration trim within : " + SelfTest[2] + "% of factory value")
    print("x-axis self test: gyration trim within : " + SelfTest[3] + "% of factory value")
    print("y-axis self test: gyration trim within : " + SelfTest[4] + "% of factory value")
    print("z-axis self test: gyration trim within : " + SelfTest[5] + "% of factory value")

    // Calibrate gyro and accelerometers, load biases in bias registers
    gyroBias, accelBias = calibrateMPU9250() 
    print("MPU9250 bias")
    print(" x   y   z  ")
    print((int)(1000*accelBias[0])) 
    print((int)(1000*accelBias[1])) 
    print((int)(1000*accelBias[2])) 
    print("mg")
    print(gyroBias[0], 1) 
    print(gyroBias[1], 1) 
    print(gyroBias[2], 1) 
    print("o/s")   
    time.sleep(1)

    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    initMPU9250() 
    print("MPU9250 initialized for active data mode....") 

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963)  // Read WHO_AM_I register for AK8963
    print("AK8963 - I AM " + d + " I should be 0x48")
    time.sleep(1)

    // Get magnetometer calibration from AK8963 ROM
    // Initialize device for active mode read of magnetometer
    initAK8963(magCalibration)
    println("AK8963 initialized for active data mode....") 

    print("Calibration values: ")
    print("X-Axis sensitivity adjustment value ") println(magCalibration[0], 2)
    print("Y-Axis sensitivity adjustment value ") println(magCalibration[1], 2)
    print("Z-Axis sensitivity adjustment value ") println(magCalibration[2], 2)
    time.sleep(1)  

  else:
    print("Could not connect to MPU9250: 0x")
    println(c, HEX)
    while(1)  // Loop forever if communication doesn't happen


/////////////////////////////////////


def loop():
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01):  // On interrupt, check if data ready interrupt
    readAccelData(accelCount)  // Read the x/y/z adc values
    getAres()
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes // - accelBias[0]  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes // - accelBias[1]   
    az = (float)accelCount[2]*aRes // - accelBias[2]  
   
    readGyroData(gyroCount)  // Read the x/y/z adc values
    getGres()
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes  
    gz = (float)gyroCount[2]*gRes   
  
    readMagData(magCount)  // Read the x/y/z adc values
    getMres()
    magbias[0] = +470.0  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.0  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.0  // User environmental x-axis correction in milliGauss
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0]  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1]  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2]   

  
  Now = micros()
  deltat = ((Now - lastUpdate)/1000000.0f) // set integration time by time elapsed since last filter update
  lastUpdate = Now

  sum = sum + deltat // sum for averaging filter update rate
  sumCount = sumCount + 1
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz)
  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz)


  if (!AHRS):
    delt_t = millis() - count
    if(delt_t > 500):

      // Print acceleration values in milligs!
      print("X-acceleration: ") print(1000*ax) print(" mg ")
      print("Y-acceleration: ") print(1000*ay) print(" mg ")
      print("Z-acceleration: ") print(1000*az) println(" mg ")
   
      // Print gyro values in degree/sec
      print("X-gyro rate: ") print(gx, 3) print(" degrees/sec ") 
      print("Y-gyro rate: ") print(gy, 3) print(" degrees/sec ") 
      print("Z-gyro rate: ") print(gz, 3) println(" degrees/sec") 
      
      // Print mag values in degree/sec
      print("X-mag field: ") print(mx) print(" mG ") 
      print("Y-mag field: ") print(my) print(" mG ") 
      print("Z-mag field: ") print(mz) println(" mG") 
   
      tempCount = readTempData()  // Read the adc values
      temperature = ((float) tempCount) / 333.87 + 21.0 // Temperature in degrees Centigrade
      // Print temperature in degrees Centigrade      
      print("Temperature is ")  print(temperature, 1)  println(" degrees C") // Print T values to tenths of s degree C
    
      display.clearDisplay()     
      display.setCursor(0, 0) display.print("MPU9250/AK8963")
      display.setCursor(0, 8) display.print(" x   y   z  ")

      display.setCursor(0,  16) display.print((int)(1000*ax)) 
      display.setCursor(24, 16) display.print((int)(1000*ay)) 
      display.setCursor(48, 16) display.print((int)(1000*az)) 
      display.setCursor(72, 16) display.print("mg")
      
      display.setCursor(0,  24) display.print((int)(gx)) 
      display.setCursor(24, 24) display.print((int)(gy)) 
      display.setCursor(48, 24) display.print((int)(gz)) 
      display.setCursor(66, 24) display.print("o/s")    
          
      display.setCursor(0,  32) display.print((int)(mx)) 
      display.setCursor(24, 32) display.print((int)(my)) 
      display.setCursor(48, 32) display.print((int)(mz)) 
      display.setCursor(72, 32) display.print("mG")   
     
      display.setCursor(0,  40) display.print("Gyro T ") 
      display.setCursor(50,  40) display.print(temperature, 1) display.print(" C")
      display.display()
      
      count = millis()
      digitalWrite(myLed, !digitalRead(myLed))  // toggle led
    
  else:
      
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count
    if (delt_t > 500): // update LCD once per half-second independent of read rate

      print("ax = ") print((int)1000*ax)  
      print(" ay = ") print((int)1000*ay) 
      print(" az = ") print((int)1000*az) println(" mg")
      print("gx = ") print( gx, 2) 
      print(" gy = ") print( gy, 2) 
      print(" gz = ") print( gz, 2) println(" deg/s")
      print("mx = ") print( (int)mx ) 
      print(" my = ") print( (int)my ) 
      print(" mz = ") print( (int)mz ) println(" mG")
      
      print("q0 = ") print(q[0])
      print(" qx = ") print(q[1]) 
      print(" qy = ") print(q[2]) 
      print(" qz = ") println(q[3])                
    
      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth. 
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]))
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
      pitch *= 180.0f / PI
      yaw   *= 180.0f / PI 
      yaw   -= 13.8 // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      roll  *= 180.0f / PI
       
      print("Yaw, Pitch, Roll: ")
      print(yaw, 2)
      print(", ")
      print(pitch, 2)
      print(", ")
      println(roll, 2)
      
      print("rate = ") print((float)sumCount/sum, 2) println(" Hz")
    
      // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
      // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
      // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
      // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
      // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
      // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
      // This filter update rate should be fast enough to maintain accurate platform orientation for 
      // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
      // The 3.3 V 8 MHz Pro Mini is doing pretty well!

      count = millis() 
      sumCount = 0
      sum = 0    


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

def getMres():
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
  switcher = {
      MFS_14BITS: 10.0*4912.0/8190.0,   // Proper scale to return milliGauss
      MFS_16BITS: 10.0*4912.0/32760.0,  // Proper scale to return milliGauss
  }
  return switcher.get(Mscale,0)


def getGres():
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  switcher = {
    GFS_250DPS: 250.0/32768.0,
    GFS_500DPS: 500.0/32768.0,
    GFS_1000DPS: 1000.0/32768.0,
    GFS_2000DPS: 2000.0/32768.0,
  }
  return switcher.get(Gscale,0)


def getAres():
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
  switcher = {
    AFS_2G: 2.0/32768.0
    AFS_4G: 4.0/32768.0
    AFS_8G: 8.0/32768.0
    AFS_16G: 16.0/32768.0
  }
  return switcher.get(Ascale,0)


//////////////////////////////////


def readAccelData():
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0])  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3]   
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5]  
  return destination


def readGyroData():
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0])  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3]   
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5]  
  return destination


def readMagData():
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01): // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0])  // Read the six raw data and ST2 registers sequentially into data array
    c = rawData[6] // End data read by reading ST2 register
      if(!(c & 0x08)): // Check if magnetic sensor overflow set, if not then report data
        destination[0] = ((int16_t)rawData[1] << 8) | rawData[0]   // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[3] << 8) | rawData[2]   // Data stored as little Endian
        destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]  


def readTempData():
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0])  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1]   // Turn the MSB and LSB into a 16-bit value


///////////////////////////////////

       
def initAK8963():
  // First extract the factory calibration for each magnetometer axis
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00) // Power down magnetometer  
  time.sleep(0.01)
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F) // Enter Fuse ROM access mode
  time.sleep(0.01)
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0])  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1. 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00) // Power down magnetometer  
  time.sleep(0.01)
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode) // Set magnetometer data resolution and sample ODR
  time.sleep(0.01)
  return destination


///////////////////////////////////


def initMPU9250():

  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00) // Clear sleep mode bit (6), enable all sensors 
  time.sleep(0.1) // Wait for all registers to reset 

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01)  // Auto select clock source to be PLL gyroscope reference if ready else
  time.sleep(0.2)
  
  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively 
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011 this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03)  

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04)  // Use a 200 Hz rate a rate consistent with the filter update rate determined inset in CONFIG above
 
  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  c = readByte(MPU9250_ADDRESS, GYRO_CONFIG) // get current GYRO_CONFIG register value
  // c = c & ~0xE0 // Clear self-test bits [7:5] 
  c = c & ~0x02 // Clear Fchoice bits [1:0] 
  c = c & ~0x18 // Clear AFS bits [4:3]
  c = c | Gscale << 3 // Set full scale range for the gyro
  // c =| 0x00 // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ) // Write new GYRO_CONFIG value to register
  
  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG) // get current ACCEL_CONFIG register value
  // c = c & ~0xE0 // Clear self-test bits [7:5] 
  c = c & ~0x18  // Clear AFS bits [4:3]
  c = c | Ascale << 3 // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c) // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3] in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2) // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c) // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22)    
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01)  // Enable data ready (bit 0) interrupt
  time.sleep(0.1)


/////////////////////////////////////////////////////  


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
def calibrateMPU9250(float * dest1, float * dest2): 
  
  gyro_bias  = [0, 0, 0]
  accel_bias = [0, 0, 0]

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80) // Write a one to bit 7 reset bit toggle reset device
  time.sleep(0.1)
   
  // get stable time source Auto select clock source to be PLL gyroscope reference if ready 
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01)  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00)
  time.sleep(0.2)                                   

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00)   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00)      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00) // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00)    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C)    // Reset FIFO and DMP
  time.sleep(0.015)

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01)      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00)  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00)  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00) // Set accelerometer full-scale to 2 g, maximum sensitivity

  gyrosensitivity  = 131   // = 131 LSB/degrees/sec
  accelsensitivity = 16384  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40)   // Enable FIFO  
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78)     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  time.sleep(0.04) // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00)        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]) // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1]
  packet_count = fifo_count/12// How many sets of full gyro and accelerometer data for averaging
  
  for ii in range(packet_count):
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0}
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]) // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  )   // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) 
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  )     
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) 
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) 
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) 
    accel_bias[0] += (int32_t) accel_temp[0] // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]
    accel_bias[2] += (int32_t) accel_temp[2]
    gyro_bias[0]  += (int32_t) gyro_temp[0]
    gyro_bias[1]  += (int32_t) gyro_temp[1]
    gyro_bias[2]  += (int32_t) gyro_temp[2]
            
  accel_bias[0] /= (int32_t) packet_count // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count
  accel_bias[2] /= (int32_t) packet_count
  gyro_bias[0]  /= (int32_t) packet_count
  gyro_bias[1]  /= (int32_t) packet_count
  gyro_bias[2]  /= (int32_t) packet_count
    
  if (accel_bias[2] > 0L):
    accel_bias[2] -= (int32_t) accelsensitivity  // Remove gravity from the z-axis accelerometer bias calculation
  else:
    accel_bias[2] += (int32_t) accelsensitivity
   
  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF
  data[3] = (-gyro_bias[1]/4)       & 0xFF
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF
  data[5] = (-gyro_bias[2]/4)       & 0xFF
  
  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0])
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1])
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2])
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3])
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4])
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5])
  
  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  accel_bias_reg = [0, 0, 0] // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]) // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1])
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0])
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1])
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0])
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1])
  
  mask = 1uL // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  mask_bit = [0, 0, 0] // Define array to hold mask bit for each accelerometer bias axis
  
  for ii in range(3):
    if ((accel_bias_reg[ii] & mask)): 
      mask_bit[ii] = 0x01 // If temperature compensation bit is set, record that fact in mask_bit
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8) // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8)
  accel_bias_reg[2] -= (accel_bias[2]/8)
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF
  data[1] = (accel_bias_reg[0])      & 0xFF
  data[1] = data[1] | mask_bit[0] // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF
  data[3] = (accel_bias_reg[1])      & 0xFF
  data[3] = data[3] | mask_bit[1] // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF
  data[5] = (accel_bias_reg[2])      & 0xFF
  data[5] = data[5] | mask_bit[2] // preserve temperature compensation bit when writing back to accelerometer bias registers
 
  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0])
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1])
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2])
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3])
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4])
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5])

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0]/(float)accelsensitivity 
  dest2[1] = (float)accel_bias[1]/(float)accelsensitivity
  dest2[2] = (float)accel_bias[2]/(float)accelsensitivity

  return (dest1, dest2)


///////////////////////////////////////////////////

   
// Accelerometer and gyroscope self test check calibration wrt factory settings
def MPU9250SelfTest(): // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass

  rawData = [0, 0, 0, 0, 0, 0]
  FS = 0
   
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00)    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02)        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3)  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02) // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3) // Set full scale range for the accelerometer to 2 g

  for ii in range(200):  // get average current values of gyro and acclerometer
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0])        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1])   // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3])   
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5])  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0])       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1])   // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3])   
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5])  
  
  for ii in range(3):  // Get average of 200 values and store as average current readings
    aAvg[ii] = aAvg[ii]/200
    gAvg[ii] = gAvg[ii]/200
  
  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0) // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0) // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  time.sleep(0.025)  // Delay a while to let the device stabilize

  for ii in range(200):  // get average self-test values of gyro and acclerometer
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0])  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1])   // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3])   
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5])  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0])  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1])   // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3])   
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5])  
  
  for ii in range(200):  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] = aSTAvg[ii]/200
    gSTAvg[ii] = gSTAvg[ii]/200 
  
  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00)  
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00)  
  time.sleep(0.025)  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL) // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL) // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL) // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO)  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO)  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO)  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )) // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )) // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )) // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )) // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )) // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )) // FT[Zg] factory trim calculation
 
  // Report results as a ratio of (STR - FT)/FT the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for i in range(3):
    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.   // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100. // Report percent differences

  return destination


/////////////////////////////////////////////////
        

// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address)  // Initialize the Tx buffer
	Wire.write(subAddress)           // Put slave register address in Tx buffer
	Wire.write(data)                 // Put data in Tx buffer
	Wire.endTransmission()           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data // `data` will store the register data	 
	Wire.beginTransmission(address)         // Initialize the Tx buffer
	Wire.write(subAddress)	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false)             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1)  // Read one byte from slave register address 
	data = Wire.read()                      // Fill Rx buffer with result
	return data                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address)   // Initialize the Tx buffer
	Wire.write(subAddress)            // Put slave register address in Tx buffer
	Wire.endTransmission(false)       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0
        Wire.requestFrom(address, count)  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read() }         // Put read results in the Rx buffer
}