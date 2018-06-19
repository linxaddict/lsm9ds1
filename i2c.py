import time

from lsm9ds1 import LSM9DS1
from rpi_bus import I2C, I2CDiscovery

i2c_discovery = I2CDiscovery()
result = i2c_discovery.discover()

if result:
    i2c_bus_acc = I2C(port=result.port, address=result.addr_acc)
    i2c_bus_mag = I2C(port=result.port, address=result.addr_mag)

    lsm9ds1 = LSM9DS1(i2c_bus_acc, i2c_bus_mag)

    acc_range = lsm9ds1.acc_range
    print('acc range: ', acc_range)

    mag_gain = lsm9ds1.mag_gain
    print('mag gain: ', mag_gain)

    gyro_scale = lsm9ds1.gyro_scale
    print('gyro scale: ', gyro_scale)

    print('result: ', result)

    while True:
        print('acc: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_acc()))
        print('mag: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_mag()))
        print('gyro: {:.2f} {:.2f} {:.2f}'.format(*lsm9ds1.read_gyro()))

        time.sleep(0.1)
else:
    print('cannot find a device, check wiring')
