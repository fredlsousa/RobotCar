from socket import *

s = socket(AF_INET, SOCK_DGRAM)

valueIndicator = 0      #Indicator finger value
valueMiddle = 0         #Middle finger value
valueRing = 0           #Ring finger value

host = "192.168.0.159"    #Server to receive from data-glove
port = 5151             #Port to receive from data-glove

s.bind((host, port))    #binding the host and port to socket


try:
    while True:
        msg, client = s.recvfrom(1024)              #my interest is the sixth, seventh and eigth value from msg (indicator, middle and ring fincgers)
        print("%s" %msg)
        receivedList = msg.split(":")               #List to receive values from msg string
        if int(receivedList[5]) <= 20:
            print("open indicator")
        else:
            print("closed indicator")
        if int(receivedList[6]) <= 20:
            print("open middle")
        else:
            print("closed middle")
        if int(receivedList[7]) <= 20:
            print("open ring")
        else:
            print("closed ring")

except KeyboardInterrupt:
    print("Stopped")
    s.close()
