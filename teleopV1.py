from socket import *
from MotorDClass import *
import time
from rotaryControl import calculateDisplacement
from rotaryControl import controlHandler
import threading


dcMotorLeft = MotorDC(22, 16, 18, 100)   #Motor DC Esquerda (A, B, PWM, pwmPower)
dcMotorRight = MotorDC(11, 40, 15, 100)  #Motor DC Direita  (A, B, PWM, PwmPower)

s = socket(AF_INET, SOCK_DGRAM)

host = "192.168.0.159"    #Server to receive from data-glove
port = 5151             #Port to receive from data-glove

s.bind((host, port))    #binding the host and port to socket

th1 = threading.Thread(target = calculateDisplacement, name = 'thread1', args = ())
th1.start()             #th1.join() blocks the python interpreter to continue in the main program, and olny gets back when the thread is completed


try:
    while True:
        msg, client = s.recvfrom(1024)              #my interest is the sixth, seventh and eigth value from msg (indicator, middle and ring fincgers)
        msgF = msg[2::]
        receivedList = msgF.split(":")               #List to receive values from msg string
        speedList = controlHandler()
        if (int(receivedList[5]) <= 20) and (int(receivedList[6]) <= 20) and (int(receivedList[7]) > 20):  #goes backwards
            #print("backwards")
            dcMotorLeft.moveBackwards()
            dcMotorRight.moveBackwards()

        elif (int(receivedList[5]) > 20) and (int(receivedList[6]) > 20) and (int(receivedList[7]) > 20):  #goes forward
            #print("forward")
            dcMotorLeft.moveForward()
            dcMotorRight.moveForward()

        elif (int(receivedList[5]) <= 20) and (int(receivedList[6]) > 20) and (int(receivedList[7]) > 20):  #goes left
            #print("left")
            dcMotorLeft.moveForward()
            dcMotorRight.moveBackwards()

        elif (int(receivedList[5]) > 20) and (int(receivedList[6]) <= 20) and (int(receivedList[7]) <= 20):  #goes right
            #print("right")
            dcMotorLeft.moveBackwards()
            dcMotorRight.moveForward()

        else:                                                                                                 #stops
            #print("stopped")
            dcMotorLeft.stop()
            dcMotorRight.stop()
            time.sleep(0.001)


except KeyboardInterrupt:
    print("Stopped")
    GPIO.cleanup()
    s.close()
