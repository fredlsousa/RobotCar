from socket import *
from MotorDClass import *
import time
import threading
from threading import Lock
from RPi import GPIO
import numpy as np
import UltrassonicClass as us


dcMotorLeft = MotorDC(22, 16, 18, 100)   #Motor DC Esquerda (A, B, PWM, pwmPower)
dcMotorRight = MotorDC(11, 40, 15, 100)  #Motor DC Direita  (A, B, PWM, PwmPower)

ultraSensor = us.Ultrassonic(29, 32, 1)  #Ultrasonic sensor

s = socket(AF_INET, SOCK_DGRAM)
#s.settimeout(0.5)


file = open("log.txt", "a+")

host = "192.168.0.159"    #Server to receive from data-glove
port = 5151             #Port to receive from data-glove

s.bind((host, port))    #binding the host and port to socket


mutex = Lock()          #mutex to controll access to
mutex2 = Lock()

LS = 0
RS = 0
RD = 0
LD = 0
distance = 0

def sensorDistance():
    global distance
    while True:
        d = ultraSensor.getDistance()
        mutex2.acquire()
        distance = d
        mutex2.release()
        sleep(0.5)


def calc_velocity(dist_cm, deltaT):
    return (dist_cm / deltaT)


def calculateDisplacement():
    clkR = 33
    dtR = 31
    clkL = 37
    dtL = 35

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(clkR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dtR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(clkL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dtL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    counterR = 0
    clkLastStateR = GPIO.input(clkR)
    counterL = 0
    clkLastStateL = GPIO.input(clkL)
    RightSpin = 0
    LeftSpin = 0
    pi = 3.14159265
    radius = 6.65 / 2
    ppi = 20 / (2 * (pi * radius))

    dispListR = np.zeros([100])
    dispListL = np.zeros([100])
    timeListR = np.zeros([100])
    timeListL = np.zeros([100])
    speedRightList = np.zeros([50])
    speedLeftList = np.zeros([50])
    m_speedRightList = np.zeros([50])
    m_speedLeftList = np.zeros([50])
    dispRNow = 0
    dispLNow = 0
    sampling_time = 0.001
    global RS
    global RD
    global LS
    global LD

    elapsed_time = 0
    ltime = time.perf_counter()
    print("GO")
    while True:
        atime = time.perf_counter()
        deltaT = atime - ltime
        elapsed_time += deltaT
        clkStateR = GPIO.input(clkR)
        clkStateL = GPIO.input(clkL)
        dtStateR = GPIO.input(dtR)
        dtStateL = GPIO.input(dtL)
        if clkStateR != clkLastStateR:
            if dtStateR != clkStateR:
                counterR -= 1
            else:
                counterR += 1
        dispRNow = (counterR / 2) / ppi
        dispListR = np.append(dispListR,dispRNow)
        timeListR = np.append(timeListR,elapsed_time)
        deltaR = (dispListR[-1] - dispListR[-2])
        velR = calc_velocity(deltaR, sampling_time)
        velR = velR/10
        speedRightList = np.append(speedRightList,velR)
        sub = speedRightList[-50:-1]
        mR = np.mean(sub, dtype=np.float32)
        m_speedRightList = np.append(m_speedRightList,mR)
        clkLastStateR = clkStateR
        if clkStateL != clkLastStateL:
            if dtStateL != clkStateL:
                counterL += 1
            else:
                counterL -= 1
        dispLNow = (counterL / 2) / ppi
        dispListL = np.append(dispListL, dispLNow)
        timeListL = np.append(timeListL, elapsed_time)
        deltaL = (dispListL[-1] - dispListL[-2])
        velL = calc_velocity(deltaL, sampling_time)
        velL = velL/10
        speedLeftList = np.append(speedLeftList,velL)
        sub = speedLeftList[-50:-1]
        mL = np.mean(sub, dtype=np.float32)
        m_speedLeftList = np.append(m_speedLeftList,mL)
        clkLastStateL = clkStateL
        ltime = atime
        sleep(sampling_time)
        if mR < 0:
            mR = mR * -1
        if mL < 0:
            mL = mL * -1

        mutex.acquire()
        RD = dispRNow
        LD = dispLNow
        RS = mR
        LS = mL
        mutex.release()
        sleep(0.005)



th1 = threading.Thread(target = calculateDisplacement, name = 'thread1', args = ())
ultraThread = threading.Thread(target = sensorDistance, name = 'thread2', args = ())
th1.start()
ultraThread.start()


data = None

if __name__ == '__main__':
    try:
        msg = str()
        DCR = 0             #DutyCycle to be set to each wheel (right motor)
        DCL = 0             #DutyCycle to be set to each wheel (left motor)
        SPL = 8            #setPoint to speed of the left motor, 15 cm/s - the speed limit of the left wheel control
        SPR = 8            #setPoint to speed of the right motor, 15 cm/s - the speed limit of the rigt wheel control
        SPDist = 15         #setPoint of distance from a obstacle

        Kp = 0.1

        while True:

            try:
                msg, client = s.recvfrom(64)              #Vlues of interest are sixth, seventh and eigth value from msg (indicator, middle and ring fincgers)
                #msg = str(msg)
                print(client)
                msg = msg.decode()
            except Exception as e:
                print (e)
                pass

            if msg == "ff":
                mutex.acquire()
                msg = str(LS) + ":" + str(RS) + ":" + str(LD) + ":" + str(RD)
                mutex.release()
                msg = msg + ":" + str(distance)
                msg = bytes(msg, encoding='utf8')
                s.sendto(msg, client)

            #if msg == "glove":
            #    mutex2.acquire()
            #    msg = str(distance)
            #    mutex2.release()
            #    msg = bytes(msg, encoding='utf8')
            #    s.sendto(msg, client)

            else:

                msgF = msg[2::]
                receivedList = msgF.split(":")               #List to receive values from msg string

                mutex.acquire()
                lsensor = LS                                 #access to critic region (the thread data)
                rsensor = RS
                mutex.release()

                dist = distance

                if (int(receivedList[5]) <= 20) and (int(receivedList[6]) <= 20) and (int(receivedList[7]) > 20):  #goes backwards
                    Err = (SPL-lsensor)         #calculating error from system
                    ErrL = (abs(Err))
                    Err = (SPR - rsensor)
                    ErrR = (abs(Err))

                    if (lsensor < SPL and DCL <= 98):       #define the increments of dutycycle by kp*error
                        DCL += Kp*ErrL
                    if (rsensor < SPR and DCR <= 98):
                        DCR += Kp*ErrR
                    if (lsensor > SPL and DCL > 0):
                        DCL -= Kp*ErrL
                    if (rsensor > SPR and DCR > 0):
                        DCR -= Kp*ErrR

                    if(DCL > 99):
                        DCL = 99
                    elif DCL < 0:
                        DCL = 0
                    if (DCR > 99):
                        DCR = 99
                    elif DCR < 0:
                        DCR = 0

                    dcMotorLeft.moveBackwards(DCL)
                    dcMotorRight.moveBackwards(DCR)

                elif (int(receivedList[5]) > 20) and (int(receivedList[6]) > 20) and (int(receivedList[7]) > 20):  #goes forward
                    Err = (SPL - lsensor)
                    ErrL = (abs(Err))
                    Err = (SPR - rsensor)
                    ErrR = (abs(Err))

                    if (lsensor < SPL and DCL <= 98):
                        DCL += Kp * ErrL
                    if (rsensor < SPR and DCR <= 98):
                        DCR += Kp * ErrR
                    if (lsensor > SPL and DCL > 0):
                        DCL -= Kp * ErrL
                    if (rsensor > SPR and DCR > 0):
                        DCR -= Kp * ErrR

                    if (DCL > 99):
                        DCL = 99
                    elif DCL < 0:
                        DCL = 0
                    if (DCR > 99):
                        DCR = 99
                    elif DCR < 0:
                        DCR = 0

                    dcMotorLeft.moveForward(DCL)
                    dcMotorRight.moveForward(DCR)


                elif (int(receivedList[5]) <= 20) and (int(receivedList[6]) > 20) and (int(receivedList[7]) > 20):  #goes left
                    Err = (SPL - lsensor)
                    ErrL = (abs(Err))
                    Err = (SPR - rsensor)
                    ErrR = (abs(Err))

                    if (lsensor < SPL and DCL <= 98):
                        DCL += Kp * ErrL
                    if (rsensor < SPR and DCR <= 98):
                        DCR += Kp * ErrR
                    if (lsensor > SPL and DCL > 0):
                        DCL -= Kp * ErrL
                    if (rsensor > SPR and DCR > 0):
                        DCR -= Kp * ErrR

                    if (DCL > 99):
                        DCL = 99
                    elif DCL < 0:
                        DCL = 0
                    if (DCR > 99):
                        DCR = 99
                    elif DCR < 0:
                        DCR = 0

                    dcMotorLeft.moveForward(DCL)
                    dcMotorRight.moveBackwards(DCR)

                elif (int(receivedList[5]) > 20) and (int(receivedList[6]) <= 20) and (int(receivedList[7]) <= 20):  #goes right
                    Err = (SPL - lsensor)
                    ErrL = (abs(Err))
                    Err = (SPR - rsensor)
                    ErrR = (abs(Err))

                    if (lsensor < SPL and DCL <= 98):
                        DCL += Kp * ErrL
                    if (rsensor < SPR and DCR <= 98):
                        DCR += Kp * ErrR
                    if (lsensor > SPL and DCL > 0):
                        DCL -= Kp * ErrL
                    if (rsensor > SPR and DCR > 0):
                        DCR -= Kp * ErrR

                    if (DCL > 99):
                        DCL = 99
                    elif DCL < 0:
                        DCL = 0
                    if (DCR > 99):
                        DCR = 99
                    elif DCR < 0:
                        DCR = 0

                    dcMotorLeft.moveBackwards(DCL)
                    dcMotorRight.moveForward(DCR)


                else:               #stops
                    DCL = 0
                    DCR = 0
                    dcMotorLeft.stop()
                    dcMotorRight.stop()


                print (lsensor, rsensor, DCL, DCR, dist)
                sleep(0.01)

    except KeyboardInterrupt:
        print("Stopped")
        GPIO.cleanup()
        file.close()
        s.close()

