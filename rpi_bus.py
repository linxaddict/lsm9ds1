from collections import namedtuple

import smbus

from lsm9ds1 import Bus, Registers


class I2C(Bus):
    def __init__(self, port: int, address: int):
        self._address = address
        self._smbus = smbus.SMBus(port)

    @property
    def address(self) -> int:
        return self._address

    def write_byte(self, register: int, value: int) -> None:
        self._smbus.write_byte_data(self.address, register, value)

    def read_byte(self, register: int) -> int:
        return self._smbus.read_byte_data(self.address, register)

    def read_bytes(self, register: int) -> [int]:
        return self._smbus.read_i2c_block_data(self.address, register)


class I2CDiscovery:
    DEVICE_ACCELEROMETER = 0x68
    DEVICE_MAGNETIC_SENSOR = 0x3d

    Result = namedtuple('Result', ['port', 'addr_acc', 'addr_mag'])

    def __init__(self, ports=None, addresses=None):
        if ports is None:
            self._ports = [0, 1]

        if addresses is None:
            self._addresses = [0x1e, 0x6b]

    def discover(self) -> Result:
        port = None
        addr_acc = None
        addr_mag = None

        for p in self._ports:
            try:
                port = p
                bus = smbus.SMBus(p)

                for address in self._addresses:
                    value = bus.read_byte_data(address, Registers.WHO_AM_I_M.value)

                    if value == self.DEVICE_ACCELEROMETER:
                        addr_acc = address
                    else:
                        addr_mag = address
            except FileNotFoundError:
                continue
            except OSError:
                continue

        if any([port is None, addr_acc is None, addr_mag is None]):
            # noinspection PyTypeChecker
            return None

        return self.Result(port=port, addr_acc=addr_acc, addr_mag=addr_mag)
