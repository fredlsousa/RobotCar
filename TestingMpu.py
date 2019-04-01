import RPi.GPIO as GPIO
from time import sleep
from mpu6050 import mpu6050
import integration as Integ

GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)

if __name__ == '__main__':
    try:
        while True:
            accelData = mpuSensor.get_accel_data()
            print("X: ", accelData['x'])
            print("Y: ", accelData['y'])
            print("Z: ", accelData['z'])
            sleep(1)
    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()