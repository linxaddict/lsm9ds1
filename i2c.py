import time

import smbus

from lsm9ds1 import I2C, LSM9DS1, Registers

DEVICE_1 = 0x1e
DEVICE_2 = 0x6b

DEVICE_ACCELEROMETER = 0x68
DEVICE_MAGNETIC_SENSOR = 0x3d

ADDR_ACCELEROMETER_GYRO = None
ADDR_MAGNETIC_SENSOR = None

bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
value = bus.read_byte_data(DEVICE_1, Registers.WHO_AM_I_M.value)

if value == DEVICE_ACCELEROMETER:
    ADDR_ACCELEROMETER = DEVICE_1
else:
    ADDR_MAGNETIC_SENSOR = DEVICE_1

value = bus.read_byte_data(DEVICE_2, Registers.WHO_AM_I_M.value)

if value == DEVICE_ACCELEROMETER:
    ADDR_ACCELEROMETER_GYRO = DEVICE_2
else:
    ADDR_MAGNETIC_SENSOR = DEVICE_2

i2c_bus_acc = I2C(1, ADDR_ACCELEROMETER_GYRO)
i2c_bus_mag = I2C(1, ADDR_MAGNETIC_SENSOR)

lsm9ds1 = LSM9DS1(i2c_bus_acc, i2c_bus_mag)

acc_range = lsm9ds1.acc_range
print('acc range: ', acc_range)

mag_gain = lsm9ds1.mag_gain
print('mag gain: ', mag_gain)

gyro_scale = lsm9ds1.gyro_scale
print('gyro scale: ', gyro_scale)

while True:
    print('acc: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_acc()))
    print('mag: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_mag()))
    print('gyro: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_gyro()))

    time.sleep(0.1)
