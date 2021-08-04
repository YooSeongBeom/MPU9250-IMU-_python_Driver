import smbus
import math
import time
from MPU9250 import MPU9250_custom as mpu9250

#addresses
#read out of axis of accelerometer
#addresses
imu=mpu9250.MPU9250(0x68)
#read out of axis of accelerometer
try:
    imu.imu_reset()
    #imu.imu_calibration() 
    while True:
        imu.imu_run()
        time.sleep(0.1)

except KeyboardInterrupt:
    pass