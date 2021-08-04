import smbus
import math
import time
import numpy as np

class MPU9250:
    
    
    def __init__(self, mpu):
        self.MPU=mpu
        self.bus=smbus.SMBus(1)
        self.acc_x=0
        self.acc_y=0
        self.acc_z=0
        self.acc_x_bias=0
        self.acc_y_bias=0
        self.acc_z_bias=0
        self.gyro_x=0
        self.gyro_y=0
        self.gyro_z=0
        self.mag_x=0
        self.mag_y=0
        self.mag_z=0
        self.temp_C=0
        self.roll_angle=0
        self.pitch_angle=0
        #scale
        self.scaling_gyro =131
        self.scaling_acc =16384
        
        #self.scaling_mag = 0.6 # for 14 bit
        self.scaling_mag = 0.15 #0.15 for 16bit
        self.scaling_temp =321
        self.offset_temp = 800
    #   self.MPU=mpu

    def imu_run(self):
        self.calc_accelerometer()
        self.calc_angle()
        self.calc_gyro()
        self.calc_mag()
        self.calc_temp()
        #self.calc_gyro()
        #self.cale_angle()

        self.imu_info()

    def imu_reset(self):
        pmgt_1 = 0x6b
        pmgt_2 = 0x6c
        self.bus.write_byte_data(self.MPU,pmgt_1,0x80)
        #self.bus.write_byte_data(self.MPU,pmgt_2,0x80)
    
    def write_word(self,reg,data):
        data_h= (data & 0xFF00) >> 8
        data_l= (data & 0x00FF)
        self.bus.write_byte_data(self.MPU,reg,data_h)
        self.bus.write_byte_data(self.MPU,reg+1,data_l)
        return

    def imu_calibration(self):
        
        # bias read out_accel
        acc_x_bias= self.read_word(0x77) >> 1
        acc_y_bias= self.read_word(0x7A) >> 1
        acc_z_bias= self.read_word(0x7D) >> 1
        
        acc_x=self.sign_word(0x3b)/(-16)
        acc_y=-((16384-self.sign_word(0x3d))/16)
        acc_z=self.sign_word(0x3f)/16

        print(acc_x_bias,acc_x)
        print(acc_y_bias,acc_y)
        print(acc_z_bias,acc_z)

        acc_x_new_bias = (acc_x_bias - int(acc_x)) << 1
        acc_y_new_bias = (acc_y_bias - int(acc_y)) << 1
        acc_z_new_bias = (acc_z_bias - int(acc_z)) << 1

        print(acc_x_new_bias)
        print(acc_y_new_bias)
        print(acc_z_new_bias)
        self.write_word(0x77,acc_x_new_bias)
        self.write_word(0x7A,acc_y_new_bias)
        self.write_word(0x7D,acc_z_new_bias)

        
    # Accelerometer
    def calc_accelerometer(self):

        self.acc_x=float(self.sign_word(0x3b)) / self.scaling_acc
        self.acc_y=float(self.sign_word(0x3d)) / self.scaling_acc
        self.acc_z=float(self.sign_word(0x3f)) / self.scaling_acc
        

    def calc_gyro(self):
        self.gyro_x=float(self.sign_word(0x43)) / self.scaling_gyro
        self.gyro_y=float(self.sign_word(0x45)) / self.scaling_gyro
        self.gyro_z=float(self.sign_word(0x47)) / self.scaling_gyro
    
    def calc_mag(self):
        self.mag_x=float(self.sign_word_invert(0x03)) * self.scaling_mag
        self.mag_y=float(self.sign_word_invert(0x05)) * self.scaling_mag
        self.mag_z=float(self.sign_word_invert(0x07)) * self.scaling_mag

    def calc_temp(self):
        temp=float(self.sign_word(0x41)) / self.scaling_temp

        self.temp_C = (temp - (self.offset_temp/ self.scaling_temp)) + 21
    def calc_angle(self):
        roll_rad=math.atan2(-self.acc_x,math.sqrt((self.acc_z * self.acc_z)+(self.acc_y*self.acc_y)))
        pitch_rad=math.atan2(self.acc_z,np.sign(self.acc_y)*math.sqrt((0.01*self.acc_x * self.acc_x)+(self.acc_y*self.acc_y)))

        self.roll_angle=roll_rad * 100/math.pi
        self.pitch_angle=pitch_rad * 100/math.pi

    def imu_info(self):
        print('Accelero_x:  '+str(self.acc_x))
        print('Accelero_y:  '+str(self.acc_y))
        print('Accelero_z:  '+str(self.acc_z))
        print('\n')
        print('Roll_Angle:  '+str(self.roll_angle))
        print('Pitch_Angle: '+str(self.pitch_angle))
        print('\n')
        print('Gyro x:      '+str(self.gyro_x))
        print('Gyro y:      '+str(self.gyro_y))
        print('Gyro z:      '+str(self.gyro_z))
        print('\n')
        print('Mag  x:      '+str(self.gyro_x))
        print('Mag  y:      '+str(self.gyro_y))
        print('Mag  z:      '+str(self.gyro_z))
        print('\n')
        print('Tempertature '+str(self.temp_C)+' C')
        print('\n')
    def read_word(self,reg):
        high=self.bus.read_byte_data(self.MPU,reg)
        low=self.bus.read_byte_data(self.MPU,reg+1)
        word = (high << 8) +low
        return word

    def read_word_invert(self,reg):
        high=self.bus.read_byte_data(self.MPU,reg+1)
        low=self.bus.read_byte_data(self.MPU,reg)
        word = (high << 8) +low
        return word

    def sign_word(self,reg):
        word=self.read_word(reg)
        if (word >= 0x8000):
            return -((0xFFFF- word)+1)
        else:
            return word

    def sign_word_invert(self,reg):
        word=self.read_word_invert(reg)
        if (word >= 0x8000):
            return -((0xFFFF- word)+1)
        else:
            return word
    

