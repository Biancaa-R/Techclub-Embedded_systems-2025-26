import RPi.GPIO as GPIO
#(0.7.1) doesnot support sp1,i2c,pwm yet,one wire func as well
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT,initial=0)

try:
    while True:
        sleep(1)
        GPIO.output(40,1)
        sleep(1)
        GPIO.output(40,0)
except KeyboardInterrupt:
    GPIO.cleanup()

 #This is used to clear the hardware configuration (registers) that was made during the GPIO configuration
