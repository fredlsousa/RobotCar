from time import sleep
from mpu6050 import mpu6050


mpuSensor = mpu6050(0x68)

if __name__ == '__main__':
    try:
        while True:
            gyroData = mpuSensor.get_gyro_data()
            print("X: ", gyroData['x'])
            print("Y: ", gyroData['y'])
            print("Z: ", gyroData['z'])
            sleep(1)
    except KeyboardInterrupt:
        print ("Stopping")
