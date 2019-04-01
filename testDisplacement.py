from integration import *
import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
from utils import *

GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)

if __name__ == '__main__':
    try:
        time1 = []
        time1.append(time.time())
        accelData = mpuSensor.get_accel_data()
        time1.append(time.time())
        deltaT = calc_deltat(time1)
        velocity = integration_trapeze(accelData['x'], accelData['y'], accelData['z'], deltaT)
        print "Velocity = ", velocity
        time.sleep(0.5)
    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()