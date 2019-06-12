import time
from Tests.utils import calc_deltat

#  To use the function calc_deltat:
#  It expects a list to calculate the nth discrete difference beetween the values within the list
#  this method is executed by the diff function that belongs to the numpy library.
#  You'll need to be familiar with the timestamp method.
#  On a very much brief approach, timestamp is a value, given by time.time() that gives you the
#  seconds since January 1st, 1970, it is how time is measured.
#  So when you have a timestamp, you'll need another value to get the deltaT or variation of time
#  between these two time values.
#  In this example time1 is a list wich is overwritten all the time, to calculate the deltaT between
#  1 second, wich will always be a very similar value.
#  A simple and dumb example to show on how to calculate deltaT, unsing calc_deltaT


try:
    while True:
        time1 = []                      #declaring the list
        time1.append(time.time())       #.append means push on that list -> pushing a timestamp value
        time.sleep(1)                   #sleeps one second to actually have a interval
        time1.append(time.time())       #Pushes other value to the list (other timestamp)
        deltaT = calc_deltat(time1)     #calculates deltaT from the list time1
        print "Delta T = ", deltaT      #prints the deltaT value

except KeyboardInterrupt:
    print ("Sttoping...")
