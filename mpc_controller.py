import asyncio
import math
import serial
import struct
import numpy as np
from cvxpy import Variable, Minimize, Problem, quad_form
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from bmp280 import BMP280
from smbus2 import SMBus

SEA_LEVEL_PRESSURE = 1013.25

class Sensors:
    def __init__(self):
        self.alpha = 0.1
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.last_altitude = 0.0

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
        self.last_yaw = 0.0

    async def sensing(self):
        await asyncio.sleep(0)
        
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
        
        roll = 0.1 * roll + 0.9 * self.last_roll
        pitch = 0.1 * pitch + 0.9 * self.last_pitch
        yaw = 0.1 * yaw + 0.9 * self.last_yaw
        
        self.last_roll = roll
        self.last_pitch = pitch
        self.last_yaw = yaw
        
        pressure = self.bmp280.get_pressure()
        altitude = 44330 * (1 - (pressure / SEA_LEVEL_PRESSURE) ** 0.1903)
        
        return roll, pitch, altitude

class MPCController:
    def __init__(self, horizon=10):
        self.A = np.array([[1, 1, 0, 0], [0, 1, -1, 0], [0, 0, 1, 1], [0, 0, 0, 1]])
        self.B = np.array([[0], [1], [0], [1]])
        
        self.Q = np.diag([10, 1, 10, 1])
        self.R = np.diag([1])

        self.horizon = horizon
        
        self.u = Variable((1, horizon))
        self.x = Variable((4, horizon + 1))

    def compute_control_input(self, x0):
        objective = 0
        constraints = [self.x[:, 0] == x0]
        
        for t in range(self.horizon):
            objective += quad_form(self.x[:, t], self.Q) + quad_form(self.u[:, t], self.R)
            constraints += [self.x[:, t + 1] == self.A @ self.x[:, t] + self.B @ self.u[:, t]]
            constraints += [self.u[:, t] >= 30, self.u[:, t] <= 160]
        
        objective += quad_form(self.x[:, self.horizon], self.Q)
        
        problem = Problem(Minimize(objective), constraints)
        problem.solve()
        
        return int(self.u.value[0, 0])

async def main():
    try:
        sensors = Sensors()
        mpc = MPCController()
        
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        ser.reset_output_buffer()
        
        while True:
            roll, pitch, altitude = await sensors.sensing()
        
            state_vector = np.array([roll, 0, pitch, 0])
        
            pwm_1 = mpc.compute_control_input(state_vector)
            pwm_2 = mpc.compute_control_input(state_vector)
            pwm_3 = mpc.compute_control_input(state_vector)
            pwm_4 = mpc.compute_control_input(state_vector)
        
            pwm_array = [pwm_1, pwm_2, pwm_3, pwm_4]
            message = b'\xFF' + struct.pack('4B', *pwm_array)
        
            ser.reset_output_buffer()
            ser.write(message)
        
            print(f'pwm_array: {message}, state_vector: {state_vector}')
            await asyncio.sleep(1/30)
    except KeyboardInterrupt:
        ser.reset_output_buffer()

if __name__ == "__main__":
    asyncio.run(main())
