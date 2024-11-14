import math
import time
import serial
import sys
import struct
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from bmp280 import BMP280
from smbus2 import SMBus
import numpy as np

SEA_LEVEL_PRESSURE = 1013.25


class Sensors:
    def __init__(self):
        self.alpha = 0.1
        self.fil_roll = 0
        self.fil_pitch = 0
        self.fil_altitude = 0

        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            mfs=AK8963_BIT_16,
            mode=AK8963_MODE_C100HZ
        )
        self.mpu.configure()
        self.mpu.writeMaster(CONFIG, 0x03)
        self.mpu.writeMaster(ACCEL_CONFIG_2, 0x03)

        self.bus = SMBus(1)
        self.bmp280 = BMP280(i2c_dev=self.bus, i2c_addr=0x76)
        self.i = 0

    def sensing(self):    
        accel_data = self.mpu.readAccelerometerMaster()
        gyro_data = self.mpu.readGyroscopeMaster()
        mag_data = self.mpu.readMagnetometerMaster()

        accelX, accelY, accelZ = accel_data
        gyroX, gyroY, gyroZ = gyro_data
        magX, magY, magZ = mag_data

        pitch = math.atan2(accelY, (math.sqrt((accelX**2) + (accelZ**2))))
        roll = math.atan2(-accelX, (math.sqrt(accelY**2 + accelZ**2)))

        Yh = (magY * math.cos(roll)) - (magZ * math.sin(roll))
        Xh = (magX * math.cos(pitch)) + (magY * math.sin(roll) * math.sin(pitch)) + (magZ * math.cos(roll) * math.sin(pitch))
        yaw = math.atan2(Yh, Xh)

        roll = roll * 57.3
        pitch = pitch * 57.3
        yaw = yaw * 57.3

        pressure = self.bmp280.get_pressure()
        altitude = 44330 * (1 - (pressure / SEA_LEVEL_PRESSURE) ** 0.1903)

        self.fil_roll = self.alpha * roll + (1 - self.alpha) * self.fil_roll
        self.fil_pitch = self.alpha * pitch + (1 - self.alpha) * self.fil_pitch
        self.fil_altitude = self.alpha * altitude + (1 - self.alpha) * self.fil_altitude

        return self.fil_roll, self.fil_pitch, self.fil_altitude



class GradientDescent:
    def __init__(self, a, learning_rate, cost_function, a_min, a_max):
        self.a = a
        self.learning_rate = learning_rate
        self.cost_function = cost_function
        self.a_min = a_min
        self.a_max = a_max
        self.G = np.zeros([len(a), len(a)])
        self.points = []
        self.result = []
    
    def grad(self, a):
        h = 0.0000001
        a_h = a + (np.eye(len(a)) * h)
        cost_function_at_a = self.cost_function(a)
        grad = []
        for i in range(0, len(a)):
            grad.append((self.cost_function(a_h[i]) - cost_function_at_a) / h)
        grad = np.array(grad)
        return grad
    
    def update_a(self, learning_rate, grad):
        if len(grad) == 1:
            grad = grad[0]
        self.a -= (learning_rate * grad)
        if (self.a_min is not None) or (self.a_max is not None):
            self.a = np.clip(self.a, self.a_min, self.a_max)
    
    def update_G(self, grad):
        self.G += np.outer(grad,grad.T)
    
    def execute(self, iterations):
        for i in range(0, iterations):
            self.points.append(list(self.a))
            self.result.append(self.cost_function(self.a))
            grad = self.grad(self.a)
            self.update_a(self.learning_rate, grad)
    
    def execute_adagrad(self, iterations):
        for i in range(0, iterations):
            self.points.append(list(self.a))
            self.result.append(self.cost_function(self.a))
            grad = self.grad(self.a)
            self.update_G(grad)
            learning_rate = self.learning_rate * np.diag(self.G)**(-0.5)
            self.update_a(learning_rate, grad)

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prevErr_1 = 0.0
        self.intErr_1 = 0.0
        self.dt = 1/30

    def compute(self, target_pitch, current_pitch, dt):
        err = current_pitch - target_pitch

        defErr = err - self.prevErr_1
        self.intErr += err * self.dt

        output = (self.kp * err) + (self.ki * self.intErr) + (self.kd * defErr)
        self.prevErr_1 = err

        return output

def check(nsteps, dt, current_pitch, target_pitch, pid_controller):
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    sensors = Sensors()
    pwm_s = np.zeros(nsteps)
    pitch_s = np.zeros(nsteps)
    time = np.zeros(nsteps)
    pitch_s[0] = current_pitch

    for i in range(nsteps - 1):
        pwm = pid_controller.compute(target_pitch, current_pitch, dt)
        pwm = np.clip(pwm, 30, 160)
        pwm_s[i] = pwm

        pwm_array = [pwm, 30, 30, 30]
        message = b'\xFF'
        message += struct.pack('4B', *pwm_array)
        ser.reset_output_buffer()
        ser.write(message)

        roll, pitch, altitude = sensors.sensing()
        pitch = round(pitch, 2)

        pitch_s[i+1] = pitch
        time[i+1] = time[i] + dt
    ser.close()
    return pwm_s, pitch_s, time


def motor_cost_function(a, i):
    Kp = a[0]
    Ki = a[1]
    Kd = a[2]

    dt = 0.1
    total_time = 60
    nsteps = int(total_time / dt)
    if not i:
        sensors = Sensors()
        roll, pitch, altitude = sensors.sensing()
        initial_pitch = round(pitch, 2)
        i == True
    # else:
    #     continue
    
    target_pitch = 2.90

    pid_controller = PIDController(Kp, Ki, Kd)

    pwm_1, pitch_s, time = check(nsteps, dt, current_pitch, target_pitch, pid_controller)

    cost = np.trapezoid(np.absolute(current_pitch - target_pitch), time) # Have to check how changing trarget from big to small changes things
    return cost

i = False
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.reset_output_buffer()
a = np.array([2.5, 0.5, 0.0])
gradient_descent = GradientDescent(a, 0.1, motor_cost_function, a_min=[2.0,0.0,-1.0], a_max=[3.0, 1.5, 1.0]) # Recheck  max values
gradient_descent.execute_adagrad(500)
print(a)
ser.close()

# Normal pitch: 2.90
# Motor 1: +ve pitch