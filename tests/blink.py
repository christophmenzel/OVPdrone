# Code goes here.
import RPi.GPIO as GPIO
import time

# Variable for the GPIO pin number
LED_pin_red = 26

# Tell the Pi we are using the breakout board pin numbering
GPIO.setmode(GPIO.BCM)

# Set up the GPIO pin for output
GPIO.setup(LED_pin_red, GPIO.OUT)

# Loop to blink our led
while True:
    GPIO.output(LED_pin_red, GPIO.HIGH)
    print("On")
    time.sleep(1)
    GPIO.output(LED_pin_red, GPIO.LOW)
    print("Off")
    time.sleep(1)
