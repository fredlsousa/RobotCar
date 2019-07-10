import RPi.GPIO as GPIO
import time

class Ultrassonic():
    def __init__(self, Trigger, Echo, index):
        # Trigger and Echo are int values from the pin used in the raspberry
        # Index is the sensor index for a case of more than one sensor being used
        self.Trigger = Trigger
        self.Echo = Echo
        self.index = index
        self.distanceValue = 0
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BOARD)
        # set GPIO direction (IN / OUT)
        GPIO.setup(self.Trigger, GPIO.OUT)
        GPIO.setup(self.Echo, GPIO.IN)

    def setDistance(self, dist):
        self.distanceValue = dist

    def distance(self):
        # set Trigger to HIGH
        GPIO.output(self.Trigger, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.Trigger, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(self.Echo) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(self.Echo) == 1:
            StopTime = time.time()

        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        self.setDistance(distance)

    def getDistance(self):
        self.distance()
        return self.distanceValue

    def showDistance(self):
        print ("#%s - Distance in cm: %.1f " % (self.index, self.getDistance()))