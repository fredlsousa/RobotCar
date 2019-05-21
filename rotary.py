from RPi import GPIO
from time import sleep

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
radius = 3.15
circ = 2 * (pi * radius) / 30

try:
    while True:
        clkStateR = GPIO.input(clkR)
        clkStateL = GPIO.input(clkL)
        dtStateR = GPIO.input(dtR)
        dtStateL = GPIO.input(dtL)
        if clkStateR != clkLastStateR:
            if dtStateR != clkStateR:
                counterR -= 1
                # if counterR == -20:
                #    RightSpin -=1
                #    counterR = 0
                #    print "Right: ", RightSpin
            else:
                counterR += 1
                disp = circ * counterR
                print "Right Displacement = " + str(disp) + " cm"
                # if counterR == 20:
                #    RightSpin += 1
                #    counterR = 0
                #    print "Right: ", RightSpin

        clkLastStateR = clkStateR
        if clkStateL != clkLastStateL:
            if dtStateL != clkStateL:
                counterL += 1
                disp = circ * counterR
                print "Left Displacement = " + str(disp) + " cm"
                # if counterL == 20:
                #    LeftSpin += 1
                #    counterL = 0
                #    print "Left: ", LeftSpin

            else:
                counterL -= 1
                # if counterL == -20:
                #    LeftSpin -= 1
                #    counterL = 0
                #   print "Left: ", LeftSpin

        clkLastStateL = clkStateL
        sleep(0.01)
except KeyboardInterrupt:
    GPIO.cleanup()
    print "Stopped"