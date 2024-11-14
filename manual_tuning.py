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


class Sensors:
    def __init__(self):
        # Initialize MPU9250
        self.alpha = 0.1
        self.fil_roll = 0
        self.fil_pitch = 0
        self.fil_altitude = 0

        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68,  # MPU9250 I2C address
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

        # Initialize BMP280 barometer
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

        pressure = self.bmp280.get_pressure()
        altitude = 44330 * (1 - (pressure / SEA_LEVEL_PRESSURE) ** 0.1903)

        self.fil_roll = self.alpha * roll + (1 - self.alpha) * self.fil_roll
        self.fil_pitch = self.alpha * pitch + (1 - self.alpha) * self.fil_pitch
        self.fil_altitude = self.alpha * altitude + (1 - self.alpha) * self.fil_altitude

        return self.fil_roll, self.fil_pitch, self.fil_altitude


class PIDController:
    def __init__(self):
        self.kp, self.ki, self.kd = 2.5, 0.0, 0.0
        self.prevErr = 0.0
        self.alt = 0
        self.intErr = 0
        
        self.dt = 0.1

    async def motor(self, err):
        await asyncio.sleep(0)

        defErr = (err - self.prevErr) / self.dt
        self.intErr += err * self.dt

        output = (self.kp * err) + (self.ki * self.intErr) + (self.kd * defErr)
        output = int(output)

        if output < 30:
            output = 30
        elif output > 160:
            output = 160

        self.prevErr_1 = err
        return output


async def main():
    try:
        sensors = Sensors()
        pid = PIDController()

        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        ser.reset_output_buffer()

        while True:
            roll, pitch, altitude = await sensors.sensing()
            motor_err = pitch - 2.9
            motor_err = round(motor_err, 2)
        
            pwm = await pid.motor(motor_err)
            
            pwm_array = [pwm, 30, 30, 30]

            message = b'\xFF'
            message += struct.pack('4B', *pwm_array)

            ser.reset_output_buffer()
            ser.write(message)
            print(f'pitch_err: {motor_err}, pwm: {pwm}')

            await asyncio.sleep(1/30)
    except KeyboardInterrupt:
        ser.reset_output_buffer()
        ser.close()


if __name__ == "__main__":
    asyncio.run(main())

# Normal roll: -0.95
# Normal pitch: 2.90

# Motor 1: +ve pitch
# Motor 2: -ve pitch
# Motor 3: -ve roll
# Motor 4: +ve roll