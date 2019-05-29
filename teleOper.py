import RPi.GPIO as GPIO
import pygame
from time import sleep
from MotorDClass import MotorDC

GPIO.setmode(GPIO.BOARD)


motorLeft = MotorDC(22, 16, 18)
motorRight = MotorDC(11, 40, 15)


pygame.init()
screen = pygame.display.set_mode((640, 320))
finished = False
isKeyPressed = False
clock = pygame.time.Clock()
while not finished:
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            finished = True
            GPIO.cleanup()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s:
                isKeyPressed = True
                motorLeft.moveBackwards()
                motorRight.moveBackwards()
                sleep(0.030)

            if event.key == pygame.K_w:
                motorLeft.moveForward()
                motorRight.moveForward()
                sleep(0.030)

            if event.key == pygame.K_a:
                motorLeft.moveBackwards()
                motorRight.moveForward()
                sleep(0.030)

            if event.key == pygame.K_d:
                motorLeft.moveForward()
                motorRight.moveBackwards()
                sleep(0.030)

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w or event.key == pygame.K_s or event.key == pygame.K_a or event.key == pygame.K_d:
                isKeyPressed = False
                # Perform action (here) when 'w' is unpressed
                motorLeft.stop()
                motorRight.stop()

        pygame.display.flip()
        clock.tick(60)
pygame.quit()



