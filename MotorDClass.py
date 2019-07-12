import RPi.GPIO as GPIO
from time import sleep


class MotorDC():
    def __init__(self, pinA, pinB, pinPWM, frequency):
        self.pinA = pinA
        self.pinB = pinB
        self.pinPWM = pinPWM
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pinA, GPIO.OUT)
        GPIO.setup(self.pinB, GPIO.OUT)
        GPIO.setup(self.pinPWM, GPIO.OUT)
        self.frequency = frequency
        self.pwm = GPIO.PWM(self.pinPWM, self.frequency)
        self.pwm.start(0)

    def moveForward(self, dutyCycle):
        GPIO.output(self.pinA, True)
        GPIO.output(self.pinB, False)
        self.pwm.ChangeDutyCycle(dutyCycle)
        GPIO.output(self.pinPWM, True)

    def moveBackwards(self, dutyCycle):
        GPIO.output(self.pinA, False)
        GPIO.output(self.pinB, True)
        self.pwm.ChangeDutyCycle(dutyCycle)
        GPIO.output(self.pinPWM, True)

    def stop(self):
        GPIO.output(self.pinPWM, False)
        self.pwm.ChangeDutyCycle(0)

    def changeDuty(self, dutyCycle):
        self.pwm.ChangeDutyCycle(dutyCycle)