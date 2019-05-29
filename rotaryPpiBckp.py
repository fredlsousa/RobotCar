from RPi import GPIO
from time import sleep
import time
import matplotlib.pyplot as plt

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



if __name__ == '__main__':
    dispListR = []
    dispListL = []
    timeListR = []
    timeListL = []
    dispL = 0
    dispR = 0
    try:
        elapsed_time = 0
        ltime = time.perf_counter()
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
                    dispR = (counterR/2)/ppi
                    dispListR.append(dispR)
                    timeListR.append(elapsed_time)
                    #timeListR.insert(len(timeListR), elapsed_time)
                    print ("Right Displacement = " + str(dispR) + " cm " + str(elapsed_time))

                else:
                    counterR += 1
                    dispR = (counterR/2)/ppi
                    dispListR.append(dispR)
                    timeListR.append(elapsed_time)
                    #timeListR.insert(len(timeListR), elapsed_time)
                    print ("Right Displacement = " + str(dispR) + " cm " + str(elapsed_time))


            clkLastStateR = clkStateR
            if clkStateL != clkLastStateL:
                if dtStateL != clkStateL:
                    counterL += 1
                    dispL = (counterL/2)/ppi
                    dispListL.append(dispL)
                    timeListL.append(elapsed_time)
                    #timeListL.insert(len(timeListR), elapsed_time)
                    print ("Left Displacement = " + str(dispL) + " cm " + str(elapsed_time))


                else:
                    counterL -= 1
                    dispL = (counterL/2)/ppi
                    dispListL.append(dispL)
                    timeListL.append(elapsed_time)
                    #timeListR.insert(len(timeListR), elapsed_time)
                    print ("Left Displacement = " + str(dispL) + " cm " + str(elapsed_time))


            clkLastStateL = clkStateL
            ltime = atime
            sleep(0.01)
    except KeyboardInterrupt:
        GPIO.cleanup()
        graphicView(timeListL, dispListL, "Left")
        graphicView(timeListR, dispListR, "Right")
        print ("Stopped")