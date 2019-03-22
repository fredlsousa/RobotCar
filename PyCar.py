import RPi.GPIO as GPIO
from time import sleep
from UltrassonicClass import Ultrassonic
from MotorDClass import MotorDC
from mpu6050 import mpu6050

GPIO.setmode(GPIO.BOARD)

inferiorThreshold = 5.5     #Limite inferior de distancia de um obstaculo
superiorThreshold = 8.5     #Limite superior de distancia de um obstaculo


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
    sleep(0.1)

if __name__ == '__main__':
    try:
        while True:
            while (Ultra1C.getDistance() >= superiorThreshold and Ultra2E.getDistance() >= superiorThreshold and Ultra3D.getDistance()) >= superiorThreshold:     #GO FORWARD IF THE DISNTANCES READ FROM THE ULTRASSONIC SENSORS ARE GREATER THAN 6.5CM
                dcMotorLeft.moveForward()
                dcMotorRight.moveForward()
                sleep(0.5)
                print "Forward"
                debugUltrasonic()
            if (Ultra1C.getDistance() <= inferiorThreshold or Ultra2E.getDistance() <= inferiorThreshold or Ultra3D.getDistance()) <= inferiorThreshold:
                while (Ultra1C.getDistance() <= inferiorThreshold or Ultra2E.getDistance() <= inferiorThreshold or Ultra3D.getDistance()) <= inferiorThreshold:
                    if Ultra1C.getDistance() <= inferiorThreshold or Ultra3D.getDistance() <= inferiorThreshold:      #GO LEFT if the distance of the RIGHT ultrassonic is inferior or equal to 4.5cm
                        while Ultra1C.getDistance() <= inferiorThreshold:
                            dcMotorLeft.moveBackwards()
                            dcMotorRight.moveBackwards()
                            sleep(0.1)
                            print "Backwards, then Left"
                            debugUltrasonic()
                        dcMotorLeft.moveBackwards()
                        dcMotorRight.moveForward()
                        sleep(0.1)
                        print "Left"
                        debugUltrasonic()
                    if Ultra1C.getDistance() <= inferiorThreshold or Ultra2E.getDistance() <= inferiorThreshold:      #GO RIGHT if the distance of the LEFT ultrassonic is inferior or equal to 4.5cm
                        while Ultra1C.getDistance() <= inferiorThreshold:
                            dcMotorLeft.moveBackwards()
                            dcMotorRight.moveBackwards()
                            sleep(0.1)
                            print "Backwards, then Right"
                            debugUltrasonic()
                        dcMotorLeft.moveForward()
                        dcMotorRight.moveBackwards()
                        sleep(0.1)
                        print "Right"
                        debugUltrasonic()

    except KeyboardInterrupt:
        print ("Stopping")
        GPIO.cleanup()
