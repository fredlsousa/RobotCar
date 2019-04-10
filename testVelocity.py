import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
from utils import *

GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)

def integrate(interval, deltaT):
    ac1 = interval[0]
    ac2 = interval[3]
    vel = (ac2+ac1)/2
    vel *= deltaT
    return vel


if __name__ == '__main__':
    try:
        while True:
            time1 = []
            accelerationY = []
            time1.append(time.time())
            for i in range(0, 4):
                accelData = mpuSensor.get_accel_data()
                accelerationY.append(accelData['y'])
            time1.append(time.time())
            deltaT = calc_deltat(time1)
            velocity_Y = integrate(accelerationY, deltaT)
            print ("Velocity\t Acceleration\t Time\n")
            print ("%f\t %f\t %f\n" % ( velocity_Y, accelerationY[3], time.clock_gettime()))
    except KeyboardInterrupt:
        print "Stopping"
        GPIO.cleanup()