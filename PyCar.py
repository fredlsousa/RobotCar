import RPi.GPIO as GPIO
from time import sleep
from UltrassonicClass import Ultrassonic
from MotorDClass import MotorDC
from mpu6050 import mpu6050

GPIO.setmode(GPIO.BOARD)


Ultra1C = Ultrassonic(31, 33, 1)   #Ultrasonico Centro   (Trigger Pin, Echo pin, FLag)
Ultra2E = Ultrassonic(35, 37, 2)   #Ultrasonico Esquerda (Trigger Pin, Echo Pin, FLag)
Ultra3D = Ultrassonic(38, 36, 3)   #Ultrasonico Direita  (Trigger Pin, Echo Pin, Flag)


dcMotorLeft = MotorDC(22, 16, 18)   #Motor DC Esquerda (A, B, PWM)
dcMotorRight = MotorDC(11, 40, 15)  #Motor DC Direita  (A, B, PWM)

mpuSensor = mpu6050(0x68)

if __name__ == '__main__':
    try:
        #Ultra1C.showDistance()
        #Ultra2E.showDistance()
        #Ultra3D.showDistance()
        accelData = mpuSensor.get_accel_data()
        print("X: ", accelData['x'])
        print("Y: ", accelData['y'])
        print("Z: ", accelData['z'])
        sleep(1)
    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()
