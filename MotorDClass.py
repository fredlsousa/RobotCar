import RPi.GPIO as GPIO
from time import sleep

class MotorDC():
    def __init__(self, pinA, pinB, pinPWM):
        self.pinA = pinA
        self.pinB = pinB
        self.pinPWM = pinPWM
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pinA, GPIO.OUT)
        GPIO.setup(self.pinB, GPIO.OUT)
        GPIO.setup(self.pinPWM, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pinPWM, 30)
        self.pwm.start(0)

    def moveForward(self):
        GPIO.output(self.pinA, True)
        GPIO.output(self.pinB, False)
        self.pwm.ChangeDutyCycle(30)
        GPIO.output(self.pinPWM, True)
        sleep(0.030)

    def moveBackwards(self):
        GPIO.output(self.pinA, False)
        GPIO.output(self.pinB, True)
        self.pwm.ChangeDutyCycle(30)
        GPIO.output(self.pinPWM, True)
        sleep(0.030)

