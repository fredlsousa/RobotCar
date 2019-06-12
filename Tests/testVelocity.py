import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
import matplotlib.pyplot as plt


GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)
mpuSensor.set_accel_range(0x18)
mpuSensor.set_gyro_range(0x18)


def integrate(ac1, ac2, deltaT):
    vel = (ac2+ac1)/2
    vel *= deltaT
    return vel


def graphicView(listX, listY, i):
    fig, ax = plt.subplots()
    ax.plot(listX, listY)
    ax.set(xlabel='time(s)', ylabel='Moving Average(m/s2)', title='Variation through time')
    ax.grid()
    fig.savefig("variation" + str(i) + ".jpg")


def setOffset(n):               #tambem calcula uma media movel
    mv = 0
    for i in range(1, n):
        data = mpuSensor.get_accel_data()
        element = data['y']
        mv = mv+element
    mv = mv/n
    return mv

def setGyroMoving(n):
    gMv = 0
    for i in range(1, n):
        dataG = mpuSensor.get_gyro_data()
        elementG = dataG['x']
        gMv = gMv + elementG
    gMv = gMv / n
    return gMv


if __name__ == '__main__':
    timeList = []
    mvView = []
    gMvView = []
    accelView = []
    speedView = []
    try:
        elapsed_time = 0
        ltime = time.perf_counter()
        laccelerationY = 0
        velocity_Y = 0

        while True:
            data = mpuSensor.get_accel_data()
            element = data['y']
            atime = time.perf_counter()
            deltaT = atime-ltime
            elapsed_time += deltaT


            n = 40
            offset = setOffset(n)
            mv = setOffset(n)
            gyroX = setGyroMoving(n)

            #if gyroX != -4.9:
            #    offset = setOffset(nOffset)

            accelerationY = mv - offset
            velocity_Y += integrate(accelerationY, laccelerationY, deltaT)
            laccelerationY = accelerationY


            timeList.append(elapsed_time)
            mvView.append(mv)
            gMvView.append(gyroX)
            accelView.append(accelerationY*100)
            speedView.append(velocity_Y*100)

            ltime = atime

            print ("Velocity\t Acceleration\t NTimes\t Gyro\t Time\n")
            print ("%f cm/s\t %f cm/s2\t %f m/s2\t %f\t %f\n" % (
                velocity_Y * 100, accelerationY * 100, mv, gyroX, elapsed_time))
            time.sleep(0.1)



    except KeyboardInterrupt:
        print ("Stopping")
        graphicView(timeList, mvView, 2)
        graphicView(timeList, gMvView, 3)
        graphicView(timeList, accelView, 4)
        graphicView(timeList, speedView, 5)
        GPIO.cleanup()