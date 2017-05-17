# Code goes here.
import RPi.GPIO as GPIO
import time

# Variable for the GPIO pin number
LED_pin = 26

# Tell the Pi we are using the breakout board pin numbering
GPIO.setmode(GPIO.BCM)

# Set up the GPIO pin for output
GPIO.setup(LED_pin, GPIO.OUT)

# Loop to blink our led
p = GPIO.PWM(LED_pin, 300)
p.start(0)
x = 0
d = 0
count = 0
while count < 3: 
	p.ChangeDutyCycle(x)
	time.sleep(0.01)
	if d == 0:
		x = x + 1
	else:
		x = x - 1
	if x == 100:
		d = 1
	if x == 0:
		d = 0
		count = count + 1
p.stop()
GPIO.cleanup()
