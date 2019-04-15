import RPi.GPIO as GPIO
import time
from mpu6050 import mpu6050
import matplotlib.pyplot as plt



GPIO.setmode(GPIO.BOARD)

mpuSensor = mpu6050(0x68)
mpuSensor.set_accel_range(0x18)


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



if __name__ == '__main__':
    #movingAverageView = []
    timeList = []
    mvView = []
    try:
        elapsed_time = 0
        averageList = [0, 0, 0, 0, 0, 0, 0, 0]
        movingAverage = 0
        ltime = time.perf_counter()
        laccelerationY = 0
        velocity_Y = 0

        while True:
            mv = 0
            averageList.pop()
            data = mpuSensor.get_accel_data()
            element = data['y']
            averageList.insert(0, element)
            atime = time.perf_counter()
            deltaT = atime-ltime
            elapsed_time += deltaT


            n = 100
            for i in range(1, n):
                data = mpuSensor.get_accel_data()
                element = data['y']
                mv = mv+element
            mv = mv/n


            #for i in range(0, len(averageList)):
            #    movingAverage += averageList[i]
            #movingAverage /= len(averageList)

            #accelerationY = movingAverage - 1.06                               #movingAverage len = 5 - >1.092
            accelerationY = mv - 0.845
            velocity_Y += integrate(accelerationY, laccelerationY, deltaT)
            laccelerationY = accelerationY

            #movingAverageView.append(movingAverage)
            timeList.append(elapsed_time)
            mvView.append(mv)

            ltime = atime
            print("Size: %d\n" % (len(averageList)))
            print ("Velocity\t Acceleration\t Moving Average\t NTimes\t Time\n")
            print ("%f cm/s\t %f cm/s2\t %f m/s2\t  %f\t %f\n" % (velocity_Y*100, (accelerationY)*100, movingAverage, mv, elapsed_time))
            time.sleep(0.1)



    except KeyboardInterrupt:
        print ("Stopping")
        #graphicView(timeList, movingAverageView, 1)
        graphicView(timeList, mvView, 2)
        GPIO.cleanup()