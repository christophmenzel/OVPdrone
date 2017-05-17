import smbus
from time import sleep

def ultrasonic_read(bus):
    try:
        bus.write_byte_data(0x70,0x00,0x51)
        sleep(0.1)
        highbyte = i2cBus.read_byte_data(0x70,0x02)
        lowbyte = i2cBus.read_byte_data(0x70,0x03)
        if highbyte == 0:
            distance = lowbyte
        else:
            distance = highbyte*256+lowbyte
        return distance
    except IOError as err:
        return -1
