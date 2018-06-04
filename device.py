import time

import spidev

from registers import CTRL_REG8, WHO_AM_I_XG

SPI_BUS = 0
SPI_DEVICE = 0

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)

to_send = [(CTRL_REG8 & 0x7F) & 0xFF, 0x05 & 0xFF]
result = spi.xfer2(to_send)

print("reset result: {0}".format(result))

# # soft reset & reboot magnetometer
# self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C)
time.sleep(0.01)

to_send = [(WHO_AM_I_XG | 0x80) & 0xFF]
result = spi.xfer2(to_send)

print("who am i result: {0}".format(result))

spi.close()
