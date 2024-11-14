import asyncio
import math
import time
import serial
import sys
import struct
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from bmp280 import BMP280
from smbus2 import SMBus

SEA_LEVEL_PRESSURE = 1013.25
j = False

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
        self.i = 0

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

        # For low pass filtering
        roll = 0.1* roll + 0.9 * self.last_roll
        pitch = 0.1 * pitch + 0.9 * self.last_pitch
        yaw = 0.1 * yaw + 0.9 * self.last_yaw
        self.last_roll = roll
        self.last_pitch = pitch
        self.last_yaw = yaw


        pressure = self.bmp280.get_pressure()
        altitude = 44330 * (1 - (pressure / SEA_LEVEL_PRESSURE) ** 0.1903)

        return roll, pitch, altitude

class PIDController:
    def __init__(self):
        self.kp_1, self.ki_1, self.kd_1 = 0.15, 0.0, 0.0
        self.kp_2, self.ki_2, self.kd_2 = 0.15, 0.0, 0.0
        self.kp_3, self.ki_3, self.kd_3 = 0.15, 0.0, 0.0
        self.kp_4, self.ki_4, self.kd_4 = 0.15, 0.0, 0.0

        self.prevErr_1 = self.prevErr_2 = self.prevErr_3 = self.prevErr_4 = 0.0
        self.intErr_1 = self.intErr_2 = self.intErr_3 = self.intErr_4 = 0.0
        
        self.dt = 1/30

    async def motor_1(self, err):
        await asyncio.sleep(0)

        defErr_1 = (err - self.prevErr_1) / self.dt
        self.intErr_1 += err * self.dt

        output = (self.kp_1 * err) + (self.ki_1 * self.intErr_1) + (self.kd_1 * defErr_1) 
        output = 66 + int(output)

        if output < 30:
            output = 30
        elif output > 160:
            output = 160

        self.prevErr_1 = err
        return output


    async def motor_2(self, err):
        await asyncio.sleep(0)

        defErr_2 = (err - self.prevErr_2) / self.dt
        self.intErr_2 += err * self.dt

        output = (self.kp_2 * err) + (self.ki_2 * self.intErr_2) + (self.kd_2 * defErr_2)
        output = 94 + int(output)

        if output < 65:
            output = 65
        elif output > 170:
            output = 170

        self.prevErr_2 = err
        return output


    async def motor_3(self, err):
        await asyncio.sleep(0) 

        defErr_3 = (err - self.prevErr_3) / self.dt
        self.intErr_3 += err * self.dt

        output = (self.kp_3 * err) + (self.ki_3 * self.intErr_3) + (self.kd_3 * defErr_3)
        output = 66 + int(output)

        if output < 30:
            output = 30
        elif output > 165:
            output = 165

        self.prevErr_3 = err
        return output

    async def motor_4(self, err):
        await asyncio.sleep(0)

        defErr_4 = (err - self.prevErr_4) / self.dt
        self.intErr_4 += err * self.dt

        output = (self.kp_4 * err) + (self.ki_4 * self.intErr_4) + (self.kd_4 * defErr_4)
        output = 66 + int(output)

        if output < 70:
            output = 70
        elif output > 170:
            output = 170

        self.prevErr_4 = err
        return output

async def main():
    try:
        sensors = Sensors()
        pid = PIDController()

        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        ser.reset_output_buffer()
        global j

        while True:
            roll, pitch, altitude = await sensors.sensing()

            motor_1_err = pitch
            motor_2_err = -1 * pitch
            motor_3_err = -1 * roll
            motor_4_err = roll

            motor_1_err = round(motor_1_err, 2)
            motor_2_err = round(motor_2_err, 2)
            motor_3_err = round(motor_3_err, 2)
            motor_4_err = round(motor_4_err, 2)
        
            pwm_1 = await pid.motor_1(motor_1_err)
            pwm_2 = await pid.motor_2(motor_2_err)
            pwm_3 = await pid.motor_3(motor_3_err)
            # pwm_4 = await pid.motor_4(motor_4_err)

            pwm_array = [pwm_1, pwm_2, pwm_3, pwm_4]

            message = b'\xFF'
            message += struct.pack('4B', *pwm_array)

            ser.reset_output_buffer()
            ser.write(message)
            print(f'pwm_array: {message}, errors: {motor_1_err}, {motor_2_err}, {motor_3_err}, {motor_4_err}')

            await asyncio.sleep(1/30)  # Keep the loop running at 30Hz
    except KeyboardInterrupt:
        ser.reset_output_buffer()

if __name__ == "__main__":
    asyncio.run(main())



