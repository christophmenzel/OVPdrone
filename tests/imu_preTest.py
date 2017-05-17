import smbus
from time import sleep

idIMU = 0x68

bus = smbus.SMBus(1)

# Configure gyroscope range
bus.write_byte_data(idIMU,0x1B,0x00)
# Configure accelerometers range
bus.write_byte_data(idIMU,0x1C,0x00)
# Set by pass mode for the magnetometers
bus.write_byte_data(idIMU,0x37,0x02)

# Request first magnetometer single measurement
bus.write_byte_data(idIMU,0x0A,0x01)

# Read block data for gyro and acc
block = bus.read_i2c_block_data(idIMU,0x3B,14)
# Get accerleration
ax = -(block[0]*256+block[1])
ay = -(block[2]*256+block[3])
az =  (block[4]*256+block[5])
# Get gyro
gx = -(block[8]*256+block[9])
gy = -(block[10]*256+block[11])
gz =  (block[12]*256+block[13])

# Get magnometer data
mag = bus.read_i2c_block_data(idIMU,0x03,7)
mx = -(mag[3]*256+mag[2])+200
my = -(mag[1]*256+mag[0])-70
mz =  (mag[5]*256+mag[4])-700

