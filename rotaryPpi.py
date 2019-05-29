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
    ax.set(xlabel='time(s)', ylabel='Moving Average(m/s2)', title='Variation through time')
    ax.grid()
    fig.savefig("variation" + str(i) + ".jpg")


if __name__ == '__main__':
    dispListR = []
    dispListL = []
    timeList = []
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
                    disp = (counterR/2)/ppi
                    dispListR.append(disp)
                    timeList.append(elapsed_time)
                    print ("Right Displacement = " + str(disp) + " cm")

                else:
                    counterR += 1
                    disp = (counterR/2)/ppi
                    dispListR.append(disp)
                    timeList.append(elapsed_time)
                    print ("Right Displacement = " + str(disp) + " cm")


            clkLastStateR = clkStateR
            if clkStateL != clkLastStateL:
                if dtStateL != clkStateL:
                    counterL += 1
                    disp = (counterL/2)/ppi
                    dispListL.append(disp)
                    timeList.append(elapsed_time)
                    print ("Left Displacement = " + str(disp) + " cm")


                else:
                    counterL -= 1
                    disp = (counterL/2)/ppi
                    dispListL.append(disp)
                    timeList.append(elapsed_time)
                    print ("Left Displacement = " + str(disp) + " cm")


            clkLastStateL = clkStateL
            ltime = atime
            sleep(0.01)
    except KeyboardInterrupt:
        GPIO.cleanup()
        graphicView(timeList, dispListL, 1)
        graphicView(timeList, dispListR, 1)
        print ("Stopped")