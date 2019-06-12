from RPi import GPIO
from time import sleep
import time
import matplotlib.pyplot as plt
import numpy as np

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
radius = 6.65/2               #3.15 other tires - 6.3 diameter
ppi = 20 / (2 * (pi * radius))




def graphicView(listX, listY, i):
    fig, ax = plt.subplots()
    ax.plot(listX, listY)
    ax.set(xlabel='time(s)', ylabel='Displacement(cm))', title='Variation ' + i + ' through time')
    ax.grid()
    fig.savefig("variation" + i + ".jpg")

def calc_velocity(dist_cm, deltaT):
    return (dist_cm / deltaT)



if __name__ == '__main__':
    dispListR = np.zeros([100])
    dispListL = np.zeros([100])
    timeListR = np.zeros([100])
    timeListL = np.zeros([100])
    speedRightList = np.zeros([100])
    speedLeftList = np.zeros([100])
    m_speedRightList = np.zeros([100])
    m_speedLeftList = np.zeros([100])
    dispRNow = 0
    dispLNow = 0
    sampling_time = 0.001

    try:
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
            speedRightList = np.append(speedRightList,velR)

            sub = speedRightList[-100:-1]
            m = np.mean(sub, dtype=np.float32)
            m_speedRightList = np.append(m_speedRightList,m)

            print("DisplacementR\t Speed\t\t Time\n")
            print("%f cm\t %f cm/s\t %f s\n" % (dispRNow, velR, timeListR[-1]))
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
            speedLeftList = np.append(speedLeftList,velL)

            sub = speedLeftList[-100:-1]
            m = np.mean(sub, dtype=np.float32)
            m_speedLeftList = np.append(m_speedLeftList,m)

            print("DisplacementL\t Speed\t\t Time\n")
            print("%f cm\t %f cm/s\t %f s\n" % (dispLNow, velL, timeListL[-1]))


            clkLastStateL = clkStateL
            ltime = atime
            sleep(sampling_time)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print()
        print((m_speedRightList))
        print((m_speedLeftList))
        graphicView(timeListL, dispListL, "Left")
        graphicView(timeListR, dispListR, "Right")
        graphicView(timeListR, speedRightList, "Right Speed")
        graphicView(timeListL, speedLeftList, "Left Speed")
        graphicView(timeListR, m_speedRightList, "MM Right Speed")
        graphicView(timeListL, m_speedLeftList, "MM Left Speed")
        print ("Stopped")