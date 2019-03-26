import RPi.GPIO as GPIO
from time import sleep
from UltrassonicClass import Ultrassonic
from MotorDClass import MotorDC
from mpu6050 import mpu6050

GPIO.setmode(GPIO.BOARD)

threshold = 6     #Limite superior de distancia de um obstaculo


Ultra1C = Ultrassonic(31, 33, 1)   #Ultrasonico Centro   (Trigger Pin, Echo pin, FLag)
Ultra2E = Ultrassonic(35, 37, 2)   #Ultrasonico Esquerda (Trigger Pin, Echo Pin, FLag)
Ultra3D = Ultrassonic(38, 36, 3)   #Ultrasonico Direita  (Trigger Pin, Echo Pin, Flag)


dcMotorLeft = MotorDC(22, 16, 18)   #Motor DC Esquerda (A, B, PWM)
dcMotorRight = MotorDC(11, 40, 15)  #Motor DC Direita  (A, B, PWM)

mpuSensor = mpu6050(0x68)


def debugUltrasonic():
    Ultra1C.showDistance()
    Ultra2E.showDistance()
    Ultra3D.showDistance()
    sleep(0.5)

try:
    while True:
        while Ultra1C.getDistance() >= threshold:
            dcMotorLeft.moveForward()
            dcMotorRight.moveForward()
            sleep(0.1)
            print "Forward"
            debugUltrasonic()
        while Ultra1C.getDistance() <= threshold:
            dcMotorLeft.moveBackwards()
            dcMotorRight.moveBackwards()
            sleep(0.2)
            print "Backwards"
            debugUltrasonic()
            dcMotorLeft.moveForward()
            dcMotorRight.moveBackwards()
            sleep(0.3)
            print "Left"
            debugUltrasonic()
            if Ultra1C.getDistance() <= threshold:
                dcMotorLeft.moveBackwards()
                dcMotorRight.moveForward()
                sleep(0.3)
                print "Right"
                debugUltrasonic()


except KeyboardInterrupt:
    print ("Stopping")
    GPIO.cleanup()
