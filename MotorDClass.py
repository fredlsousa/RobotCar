import RPi.GPIO as GPIO
from time import sleep

class MotorDC():
    def __init__(self, pinA, pinB, pinPWM, pwmPower):
        self.pinA = pinA
        self.pinB = pinB
        self.pinPWM = pinPWM
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pinA, GPIO.OUT)
        GPIO.setup(self.pinB, GPIO.OUT)
        GPIO.setup(self.pinPWM, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pinPWM, 30)
        self.pwmPower = pwmPower

    def moveForward(self):
        GPIO.output(self.pinA, True)
        GPIO.output(self.pinB, False)
        self.pwm.start(self.pwmPower)
        GPIO.output(self.pinPWM, True)
        sleep(0.030)

    def moveBackwards(self):
        GPIO.output(self.pinA, False)
        GPIO.output(self.pinB, True)
        self.pwm.start(self.pwmPower)
        GPIO.output(self.pinPWM, True)
        sleep(0.030)

    def stop(self):
        GPIO.output(self.pinPWM, False)
        self.pwm.stop()
