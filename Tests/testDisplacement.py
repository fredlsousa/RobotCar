from Tests.integration import *
import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
from Tests.utils import *

GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)

def getVelocity_Y():
    time1 = []
    accelerationY = []
    time1.append(time.time())
    for i in range(0, 4):
        accelData = mpuSensor.get_accel_data()
        accelerationY.append(accelData['y'])
    time1.append(time.time())
    deltaT = calc_deltat(time1)
    velocity_Y = integration_trapeze(0, 3, accelerationY, deltaT)
    return velocity_Y

if __name__ == '__main__':
    try:
        while True:
            time1 = []
            vListY = []
            time1.append(time.time())
            for i in range(0, 4):
                velocity_Y = getVelocity_Y()
                print ("debug %f \n" %(velocity_Y))
                vListY.append(velocity_Y)
            time1.append(time.time())
            deltaT = calc_deltat(time1)
            displacementY = integration_trapeze(0, 3, vListY, deltaT)
            print("Velocity\t Displacement\t Time\n")
            print("%f\t %f\t %f\n" % (vListY[3], displacementY, deltaT))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()