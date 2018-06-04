import time

import smbus

from lsm9ds1 import I2C, LSM9DS1
from registers import WHO_AM_I_M

DEVICE_1 = 0x1e
DEVICE_2 = 0x6b

DEVICE_ACCELEROMETER = 0x68
DEVICE_MAGNETIC_SENSOR = 0x3d

ADDR_ACCELEROMETER_GYRO = None
ADDR_MAGNETIC_SENSOR = None

bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
value = bus.read_byte_data(DEVICE_1, WHO_AM_I_M)

if value == DEVICE_ACCELEROMETER:
    ADDR_ACCELEROMETER = DEVICE_1
else:
    ADDR_MAGNETIC_SENSOR = DEVICE_1

value = bus.read_byte_data(DEVICE_2, WHO_AM_I_M)

if value == DEVICE_ACCELEROMETER:
    ADDR_ACCELEROMETER_GYRO = DEVICE_2
else:
    ADDR_MAGNETIC_SENSOR = DEVICE_2

i2c_bus_acc = I2C(1, ADDR_ACCELEROMETER_GYRO)
i2c_bus_mag = I2C(1, ADDR_MAGNETIC_SENSOR)

lsm9ds1 = LSM9DS1(i2c_bus_acc, i2c_bus_mag)

while True:
    print('acc: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_acc()))
    time.sleep(0.1)
