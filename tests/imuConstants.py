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
# Ascale
IMU_AFS_2G  = 0
IMU_AFS_4G  = 1
IMU_AFS_8G  = 2
IMU_AFS_16G = 3

#Gscale
IMU_GFS_250DPS  = 0
IMU_GFS_500DPS  = 1
IMU_GFS_1000DPS = 2
IMU_GFS_2000DPS = 3

#Mscale 
IMU_MFS_14BITS = 0 # 0.6 mG per LSB
IMU_MFS_16BITS = 1 # 0.15 mG per LSB