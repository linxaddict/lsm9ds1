import struct
import time
from abc import ABCMeta, abstractmethod
from enum import Enum


class Registers(Enum):
    CTRL_REG1_G = 0x10
    OUT_X_L_G = 0x18
    CTRL_REG5_XL = 0x1F
    CTRL_REG6_XL = 0x20
    CTRL_REG8 = 0x22
    OUT_X_L_XL = 0x28
    WHO_AM_I_M = 0x0F
    CTRL_REG2_M = 0x21
    CTRL_REG3_M = 0x22
    OUT_X_L_M = 0x28


class Bus(metaclass=ABCMeta):
    @abstractmethod
    def write_byte(self, register: int, value: int) -> None:
        pass

    @abstractmethod
    def read_byte(self, register: int) -> int:
        pass

    @abstractmethod
    def read_bytes(self, register: int) -> [int]:
        pass


class LSM9DS1:
    ACC_1G = 9.80665

    class AccRange(Enum):
        RANGE_2G = 0b00 << 3
        RANGE_4G = 0b10 << 3
        RANGE_8G = 0b11 << 3
        RANGE_16G = 0b01 << 3

    class MagGain(Enum):
        GAIN_4GAUSS = 0b00 << 5
        GAIN_8GAUSS = 0b01 << 5
        GAIN_12GAUSS = 0b10 << 5
        GAIN_16GAUSS = 0b11 << 5

    class GyroScale(Enum):
        SCALE_245DPS = 0b00 << 3
        SCALE_500DPS = 0b01 << 3
        SCALE_2000DPS = 0b11 << 3

    def _reset_acc(self):
        self._bus_acc.write_byte(Registers.CTRL_REG8.value, 0x05)

    def _reset_mag(self):
        self._bus_mag.write_byte(Registers.CTRL_REG2_M.value, 0x0C)

    def _enable_continous_mode_acc(self):
        self._bus_acc.write_byte(Registers.CTRL_REG5_XL.value, 0x38)
        self._bus_acc.write_byte(Registers.CTRL_REG6_XL.value, 0xC0)

    def _enable_continous_mode_gyro(self):
        self._bus_acc.write_byte(Registers.CTRL_REG1_G.value, 0xC0)

    def _enable_continous_mode_mag(self):
        self._bus_mag.write_byte(Registers.CTRL_REG3_M.value, 0x00)

    def __init__(self, bus_acc: Bus, bus_mag: Bus):
        self._bus_acc = bus_acc
        self._bus_mag = bus_mag

        self._reset_acc()
        self._reset_mag()

        time.sleep(0.01)

        self._enable_continous_mode_acc()
        self._enable_continous_mode_gyro()
        self._enable_continous_mode_mag()

        self.acc_range = LSM9DS1.AccRange.RANGE_2G
        self.gyro_scale = LSM9DS1.GyroScale.SCALE_245DPS
        self.mag_gain = LSM9DS1.MagGain.GAIN_4GAUSS

        self._acc_lsb_map = {
            LSM9DS1.AccRange.RANGE_2G: 0.061,
            LSM9DS1.AccRange.RANGE_4G: 0.122,
            LSM9DS1.AccRange.RANGE_8G: 0.244,
            LSM9DS1.AccRange.RANGE_16G: 0.732
        }

        self._mag_lsb_map = {
            LSM9DS1.MagGain.GAIN_4GAUSS: 0.14,
            LSM9DS1.MagGain.GAIN_8GAUSS: 0.29,
            LSM9DS1.MagGain.GAIN_12GAUSS: 0.43,
            LSM9DS1.MagGain.GAIN_16GAUSS: 0.58
        }

        self._gyro_lsb_map = {
            LSM9DS1.GyroScale.SCALE_245DPS: 0.00875,
            LSM9DS1.GyroScale.SCALE_500DPS: 0.01750,
            LSM9DS1.GyroScale.SCALE_2000DPS: 0.07000
        }

    @property
    def acc_range(self) -> AccRange:
        """
        CTRL_REG6_XL: ODR_XL2 ODR_XL1 ODR_XL0 FS1_XL FS0_XL BW_SCAL _ODR BW_XL1 BW_XL0

        FS1_XL and FS0_XL are used to encode accelerometer range.

        :return: accelerometer range
        """
        acc_range = self._bus_acc.read_byte(Registers.CTRL_REG6_XL.value)
        return LSM9DS1.AccRange((acc_range & 0x18) & 0xFF)

    @acc_range.setter
    def acc_range(self, value: AccRange):
        """
        Sets new accelerometer range.

        CTRL_REG6_XL: ODR_XL2 ODR_XL1 ODR_XL 0 FS1_XL FS0_XL BW_SCAL _ODR BW_XL1 BW_XL0

        FS1_XL and FS0_XL are used to encode the range.

        :param value: range
        """

        acc_range = self._bus_acc.read_byte(Registers.CTRL_REG6_XL.value)

        acc_range = (acc_range & ~0x18) & 0xFF
        acc_range |= value.value

        self._bus_acc.write_byte(Registers.CTRL_REG6_XL.value, acc_range)

    @property
    def mag_gain(self) -> MagGain:
        """
        CTRL_REG2_M: 0 FS1 FS0 0 REBOOT SOFT_RST 0 0

        FS1 and FS0 are used to encode the gain.

        :return: gain
        """

        gain = self._bus_mag.read_byte(Registers.CTRL_REG2_M.value)
        return LSM9DS1.MagGain((gain & 0x60) & 0xFF)

    @mag_gain.setter
    def mag_gain(self, value: MagGain):
        """
        Sets new magnetometer gain.

        CTRL_REG2_M: 0 FS1 FS0 0 REBOOT SOFT_RST 0 0

        FS1 and FS0 are used to encode the gain.

        :param value: gain
        """

        gain = self._bus_mag.read_byte(Registers.CTRL_REG2_M.value)
        gain = (gain & ~0x60) & 0xFF
        gain |= value.value

        self._bus_mag.write_byte(Registers.CTRL_REG2_M.value, gain)

    @property
    def gyro_scale(self) -> GyroScale:
        """
        CTRL_REG1_G: ODR_G2 ODR_G1 ODR_G0 FS_G1 FS_G0 0 BW_G1 BW_G0

        FS_G1 and FS_G0 are used to encode the scale.

        :return: scale
        """
        scale = self._bus_acc.read_byte(Registers.CTRL_REG1_G.value)
        return LSM9DS1.GyroScale((scale & 0x18) & 0xFF)

    @gyro_scale.setter
    def gyro_scale(self, value: GyroScale):
        """
        Sets new gyroscope scale.

        CTRL_REG1_G: ODR_G2 ODR_G1 ODR_G0 FS_G1 FS_G0 0 BW_G1 BW_G0

        FS_G1 and FS_G0 are used to encode the scale.

        :param value: scale
        """

        scale = self._bus_acc.read_byte(Registers.CTRL_REG1_G.value)
        scale = (scale & ~0x18) & 0xFF
        scale |= value.value

        self._bus_acc.write_byte(Registers.CTRL_REG1_G.value, scale)

    def read_acc(self):
        """
        Reads current acceleration and returns it as a list of 3 elements: ax, ay, az (m/s^2).

        :return: current acceleration
        """

        raw_acc_value = self._bus_acc.read_bytes(0x80 | Registers.OUT_X_L_XL.value)
        acc = struct.unpack_from('<hhh', struct.pack('BBBBBB', *raw_acc_value[:6]))
        acc_lsb = self._acc_lsb_map.get(self.acc_range)

        return [x * acc_lsb / 1000.0 * LSM9DS1.ACC_1G for x in acc]

    def read_mag(self):
        """
        Reads current magnetometer values are returns it as a list of 3 elements: mx, my, mz (gauss).

        :return: current magnetometer readings
        """

        raw_mag_value = self._bus_mag.read_bytes(0x80 | Registers.OUT_X_L_M.value)
        mag = struct.unpack_from('<hhh', struct.pack('BBBBBB', *raw_mag_value[:6]))
        mag_lsb = self._mag_lsb_map.get(self.mag_gain)

        return [x * mag_lsb / 1000.0 for x in mag]

    def read_gyro(self):
        """
        Reads current gyroscope values are returns it as a list of 3 elements: wx, wy, wz (deg/s).

        :return: current gyroscope readings
        """

        raw_gyro_value = self._bus_acc.read_bytes(0x80 | Registers.OUT_X_L_G.value)
        gyro = struct.unpack_from('<hhh', struct.pack('BBBBBB', *raw_gyro_value[:6]))
        gyro_lsb = self._gyro_lsb_map.get(self.gyro_scale)

        return [x * gyro_lsb for x in gyro]
