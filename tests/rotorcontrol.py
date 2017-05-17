from Adafruit_PWM_Servo_Driver import PWM
import time
from math import floor

R1 = 8
R2 = 9
R3 = 10
R4 = 11

def setRotor(pin, p):
    global pwm
    pwm.setPWM(pin,0,floor(1862*p+2232))

pwm = PWM(0x40)
pwm.setPWMFreq(300)

for p in range(100):
    print(p)
    setRotor(R1,p/100)
    time.sleep(0.03)

for p in range(100):
    print(p)
    setRotor(R1,(99-p)/100)
    time.sleep(0.03)
