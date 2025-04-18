import RPi.GPIO as GPIO
from gpiozero import LED
from signal import pause
from time import sleep

led1=LED(13)# Automatically takes it to be bcm
led2=LED(17)

try:
    while True:
        sleep(0.5)
        led1.ON()
        sleep(0.5)
        led1.OFF()
        led2.ON()
        sleep(0.5)
        led2.OFF()
        pause()
except:
    GPIO.cleanup()
