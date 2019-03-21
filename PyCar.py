import RPi.GPIO as GPIO
from time import sleep
from UltrassonicClass import Ultrassonic
from MotorDClass import MotorDC
from mpu6050 import mpu6050

GPIO.setmode(GPIO.BOARD)

inferiorThreshold = 4.5     #Limite inferior de distancia de um obstaculo
superiorThreshold = 6.5     #Limite superior de distancia de um obstaculo


Ultra1C = Ultrassonic(31, 33, 1)   #Ultrasonico Centro   (Trigger Pin, Echo pin, FLag)
Ultra2E = Ultrassonic(35, 37, 2)   #Ultrasonico Esquerda (Trigger Pin, Echo Pin, FLag)
Ultra3D = Ultrassonic(38, 36, 3)   #Ultrasonico Direita  (Trigger Pin, Echo Pin, Flag)


dcMotorLeft = MotorDC(22, 16, 18)   #Motor DC Esquerda (A, B, PWM)
dcMotorRight = MotorDC(11, 40, 15)  #Motor DC Direita  (A, B, PWM)

mpuSensor = mpu6050(0x68)


if __name__ == '__main__':
    try:
        while True:
            while (Ultra1C.getDistance() and Ultra2E.getDistance() and Ultra3D.getDistance()) >= superiorThreshold:     #GO FORWARD IF THE DISNTANCES READ FROM THE ULTRASSONIC SENSORS ARE GREATER THAN 6.5CM
                dcMotorLeft.moveForward()
                dcMotorRight.moveForward()
            if (Ultra1C.getDistance() or Ultra2E.getDistance() or Ultra3D.getDistance()) <= inferiorThreshold:
                while (Ultra1C.getDistance() or Ultra2E.getDistance() or Ultra3D.getDistance()) <= inferiorThreshold:
                    if Ultra1C.getDistance() <= inferiorThreshold:      #GO BACKWARDS if the distance of the CENTER ultrassonic is inferior or equal to 4.5cm
                        dcMotorLeft.moveBackwards()
                        dcMotorRight.moveBackwards()
                    if Ultra3D.getDistance() <= inferiorThreshold:      #GO LEFT if the distance of the RIGHT ultrassonic is inferior or equal to 4.5cm
                        dcMotorLeft.moveBackwards()
                        dcMotorRight.moveForward()
                    if Ultra2E.getDistance() <= inferiorThreshold:      #GO RIGHT if the distance of the LEFT ultrassonic is inferior or equal to 4.5cm
                        dcMotorLeft.moveForward()
                        dcMotorRight.moveBackwards()

    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()
