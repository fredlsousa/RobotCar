from socket import *
from MotorDClass import *
import time


dcMotorLeft = MotorDC(22, 16, 18, 100)   #Motor DC Esquerda (A, B, PWM, pwmPower)
dcMotorRight = MotorDC(11, 40, 15, 100)  #Motor DC Direita  (A, B, PWM, PwmPower)

s = socket(AF_INET, SOCK_DGRAM)

valueIndicator = 0      #Indicator finger value
valueMiddle = 0         #Middle finger value
valueRing = 0           #Ring finger value

host = "192.168.0.159"    #Server to receive from data-glove
port = 5151             #Port to receive from data-glove

s.bind((host, port))    #binding the host and port to socket


try:
    while True:
        msg, client = s.recvfrom(1024)              #my interest is the sixth, seventh and eigth value from msg (indicator, middle and ring fincgers)
        msgF = msg[2::]
        receivedList = msgF.split(":")               #List to receive values from msg string
        print("%d\t %d\t %d\t" %(int(receivedList[5]), int(receivedList[6]), int(receivedList[7])))

        if (int(receivedList[5]) <= 20) and (int(receivedList[6]) <= 20) and (int(receivedList[7]) > 20):
            print("backwards")
            dcMotorLeft.moveBackwards()
            dcMotorRight.moveBackwards()

        elif (int(receivedList[5]) > 20) and (int(receivedList[6]) > 20) and (int(receivedList[7]) > 20):
            print("forward")
            dcMotorLeft.moveForward()
            dcMotorRight.moveForward()

        elif (int(receivedList[5]) <= 20) and (int(receivedList[6]) > 20) and (int(receivedList[7]) > 20):
            print("left")
            dcMotorLeft.moveForward()
            dcMotorRight.moveBackwards()

        elif (int(receivedList[5]) > 20) and (int(receivedList[6]) <= 20) and (int(receivedList[7]) <= 20):
            print("right")
            dcMotorLeft.moveBackwards()
            dcMotorRight.moveForward()

        else:
            print("stopped")
            dcMotorLeft.stop()
            dcMotorRight.stop()
            time.sleep(0.001)


except KeyboardInterrupt:
    print("Stopped")
    GPIO.cleanup()
    s.close()
