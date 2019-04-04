from integration import *
import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
from utils import *

GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)

if __name__ == '__main__':
    try:
        while True:
            time1 = []
            accelerationY = []
            time1.append(time.time())
            for i in range(1, 4):
                accelData = mpuSensor.get_accel_data()
                accelerationY.append(accelData['y'])
            time1.append(time.time())
            deltaT = calc_deltat(time1)
            velocity_Y = integration_trapeze(1, 4, accelerationY, deltaT)
            print "Velocity = ", velocity_Y
            time.sleep(0.5)
    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()