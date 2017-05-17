import smbus
from time import sleep

bus = 1
ultrasonic = 0x70
start = 0x51
highbyte = 0
lowbyte = 0
distance = 0

i2cBus = smbus.SMBus(bus)

while True:

    i2cBus.write_byte_data(ultrasonic,0x00,0x51)
    sleep(0.1)
    highbyte = i2cBus.read_byte_data(ultrasonic,0x02)
    lowbyte = i2cBus.read_byte_data(ultrasonic,0x03)
    if highbyte == 0:
        distance = lowbyte
    else:
        distance = highbyte*256+lowbyte
    print(distance, " cm")
