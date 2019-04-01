from datetime import datetime
from utils import calc_deltat

try:
    while True:
        time1 = datetime.now().time()
        deltaT = calc_deltat(time1)
        print("Delta T = ", deltaT)

except KeyboardInterrupt:
    print ("Sttoping...")
