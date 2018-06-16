import struct
import time
from abc import ABCMeta, abstractmethod
from enum import Enum

import smbus


class Registers(Enum):
    CT_THS = 0x04
    ACT_DUR = 0x05
    INT_GEN_CFG_XL = 0x06
    INT_GEN_THS_X_XL = 0x07
    INT_GEN_THS_Y_XL = 0x08
    INT_GEN_THS_Z_XL = 0x09
    INT_GEN_DUR_XL = 0x0A
    REFERENCE_G = 0x0B
    INT1_CTRL = 0x0C
    INT2_CTRL = 0x0D
    WHO_AM_I_XG = 0x0F
    CTRL_REG1_G = 0x10
    CTRL_REG2_G = 0x11
    CTRL_REG3_G = 0x12
    ORIENT_CFG_G = 0x13
    INT_GEN_SRC_G = 0x14
    OUT_TEMP_L = 0x15
    OUT_TEMP_H = 0x16
    STATUS_REG_0 = 0x17
    OUT_X_L_G = 0x18
    OUT_X_H_G = 0x19
    OUT_Y_L_G = 0x1A
    OUT_Y_H_G = 0x1B
    OUT_Z_L_G = 0x1C
    OUT_Z_H_G = 0x1D
    CTRL_REG4 = 0x1E
    CTRL_REG5_XL = 0x1F
    CTRL_REG6_XL = 0x20
    CTRL_REG7_XL = 0x21
    CTRL_REG8 = 0x22
    CTRL_REG9 = 0x23
    CTRL_REG10 = 0x24
    INT_GEN_SRC_XL = 0x26
    STATUS_REG_1 = 0x27
    OUT_X_L_XL = 0x28
    OUT_X_H_XL = 0x29
    OUT_Y_L_XL = 0x2A
    OUT_Y_H_XL = 0x2B
    OUT_Z_L_XL = 0x2C
    OUT_Z_H_XL = 0x2D
    FIFO_CTRL = 0x2E
    FIFO_SRC = 0x2F
    INT_GEN_CFG_G = 0x30
    INT_GEN_THS_XH_G = 0x31
    INT_GEN_THS_XL_G = 0x32
    INT_GEN_THS_YH_G = 0x33
    INT_GEN_THS_YL_G = 0x34
    INT_GEN_THS_ZH_G = 0x35
    INT_GEN_THS_ZL_G = 0x36
    INT_GEN_DUR_G = 0x37
    OFFSET_X_REG_L_M = 0x05
    OFFSET_X_REG_H_M = 0x06
    OFFSET_Y_REG_L_M = 0x07
    OFFSET_Y_REG_H_M = 0x08
    OFFSET_Z_REG_L_M = 0x09
    OFFSET_Z_REG_H_M = 0x0A
    WHO_AM_I_M = 0x0F
    CTRL_REG1_M = 0x20
    CTRL_REG2_M = 0x21
    CTRL_REG3_M = 0x22
    CTRL_REG4_M = 0x23
    CTRL_REG5_M = 0x24
    STATUS_REG_M = 0x27
    OUT_X_L_M = 0x28
    OUT_X_H_M = 0x29
    OUT_Y_L_M = 0x2A
    OUT_Y_H_M = 0x2B
    OUT_Z_L_M = 0x2C
    OUT_Z_H_M = 0x2D
    INT_CFG_M = 0x30
    INT_SRC_M = 0x31
    INT_THS_L_M = 0x32
    INT_THS_H_M = 0x33
    WHO_AM_I_AG_RSP = 0x68
    WHO_AM_I_M_RSP = 0x3D


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


class LSM9DS1:
    class AccRange(Enum):
        RANGE_2G = (0b00 << 3)
        RANGE_4G = (0b10 << 3)
        RANGE_8G = (0b11 << 3)
        RANGE_16G = (0b01 << 3)

    class MagGain(Enum):
        GAIN_4GAUSS = (0b00 << 5)
        GAIN_8GAUSS = (0b01 << 5)
        GAIN_12GAUSS = (0b10 << 5)
        GAIN_16GAUSS = (0b11 << 5)

    class GyroScale(Enum):
        SCALE_245DPS = (0b00 << 3)
        SCALE_500DPS = (0b01 << 3)
        SCALE_2000DPS = (0b11 << 3)

    def _reset_acc(self):
        self._bus_acc.write_byte(Registers.CTRL_REG8.value, 0x05)

    def _reset_mag(self):
        self._bus_mag.write_byte(Registers.CTRL_REG2_M.value, 0x0C)

    def _enable_continous_mode_acc(self):
        self._bus_acc.write_byte(Registers.CTRL_REG5_XL.value, 0x38)
        self._bus_acc.write_byte(Registers.CTRL_REG6_XL.value, 0xC0)

    def _enable_continous_mode_mag(self):
        self._bus_mag.write_byte(Registers.CTRL_REG3_M.value, 0x00)

    def __init__(self, bus_acc: Bus, bus_mag: Bus):
        self._accel_mg_lsb = None
        self._mag_mgauss_lsb = None
        self._gyro_dps_digit = None

        self._acc_range = LSM9DS1.AccRange.RANGE_2G.value
        self._mag_gain = LSM9DS1.MagGain.GAIN_4GAUSS.value
        self._gyro_scale = LSM9DS1.GyroScale.SCALE_245DPS.value

        self._bus_acc = bus_acc
        self._bus_mag = bus_mag

        self._reset_acc()
        self._reset_mag()

        time.sleep(0.01)

        self._enable_continous_mode_acc()
        self._enable_continous_mode_mag()

        self.acc_range = LSM9DS1.AccRange.RANGE_2G
        self.gyro_scale = LSM9DS1.GyroScale.SCALE_245DPS
        self.mag_gain = LSM9DS1.MagGain.GAIN_4GAUSS

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
        gain = (gain & 0x60) & 0xFF
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
        scale = (scale & 0x18) & 0xFF
        scale |= value.value

        self._bus_acc.write_byte(Registers.CTRL_REG1_G.value, scale)

    def read_acc(self):
        value1 = self._bus_acc.read_bytes(0x80 | Registers.OUT_X_L_XL.value)
        data = struct.unpack_from('<hhh', struct.pack('BBBBBB', *value1[:6]))

        return [x * 0.061 / 1000.0 * 9.80665 for x in data]
