"""
# PyQMC

Python driver for the `QMC5883L` 3-Axis Magnetic Sensor.

## Usage example:
```
from PyQMC import QMC5883L
sensor = QMC5883L()
m = sensor.get_magnet_raw()
b = sensor.get_bearing_raw()
print(m)
print(b)
```

Returns three 16-bit signed integers, representing the values
of the magnetic sensor on axis `X`, `Y` and `Z`, e.g. `[-1257, 940, -4970]`.
"""

import smbus2 as smbus
from time import sleep
from datetime import datetime
from math import degrees, atan2


DFLT_BUS = 1                    # Default I2C bus.
DFLT_ADDRESS = 0x0d             # Default I2C address.

REG_XOUT_LSB    = 0x00          # Output Data Registers for magnetic sensor.
REG_XOUT_MSB    = 0x01
REG_YOUT_LSB    = 0x02
REG_YOUT_MSB    = 0x03
REG_ZOUT_LSB    = 0x04       
REG_ZOUT_MSB    = 0x05
REG_STATUS_1    = 0x06          # Status Register.
REG_TOUT_LSB    = 0x07          # Output Data Registers for temperature.
REG_TOUT_MSB    = 0x08
REG_CONTROL_1   = 0x09          # Control Register #1.
REG_CONTROL_2   = 0x0a          # Control Register #2.
REG_RST_PERIOD  = 0x0b          # SET/RESET Period Register.
REG_CHIP_ID     = 0x0d          # Chip ID register.

# Flags for Status Register #1.
STAT_DRDY  = 0b00000001         # Data Ready.
STAT_OVL   = 0b00000010         # Overflow flag.
STAT_DOR   = 0b00000100         # Data skipped for reading.

# Flags for Status Register #2.
INT_ENB    = 0b00000001         # Interrupt Pin Enabling.
POL_PNT    = 0b01000000         # Pointer Roll-over.
SOFT_RST   = 0b10000000         # Soft Reset.

# Flags for Control Register 1.
MODE_STBY  = 0b00000000         # Standby mode.
MODE_CONT  = 0b00000001         # Continuous read mode.
ODR_10HZ   = 0b00000000         # Output Data Rate - 10 Hz.
ODR_50HZ   = 0b00000100         # Output Data Rate - 50 Hz.
ODR_100HZ  = 0b00001000         # Output Data Rate - 100 Hz.
ODR_200HZ  = 0b00001100         # Output Data Rate - 200 Hz.
RNG_2G     = 0b00000000         # Range 2 Gauss: for magnetic-clean environments.
RNG_8G     = 0b00010000         # Range 8 Gauss: for strong magnetic fields.
OSR_512    = 0b00000000         # Over Sample Rate 512: less noise, more power.
OSR_256    = 0b01000000         # Over Sample Rate 256: default.
OSR_128    = 0b10000000         # Over Sample Rate 128: more noise, less power.
OSR_64     = 0b11000000         # Over Sample Rate 64:  more noise, less power.



class QMC5883L(object):
    """
    ## QMC5883L

    Interface for the `QMC5883L` 3-Axis Magnetic Sensor.
    """
    def _warn(self, msg: str) -> None:
        now = datetime.now()
        now_str = now.strftime("%H:%M:%S")
        now_str += f".{now.microsecond // 1000:03d}"
        print(f"\n[{now_str}]  >> {msg}\n")

    def __init__(self,
                 i2c_bus=DFLT_BUS,
                 address=DFLT_ADDRESS,
                 output_data_rate=ODR_50HZ,
                 # output_data_rate=ODR_10HZ,
                 output_range=RNG_2G,
                 oversampling_rate=OSR_512):

        self.address = address
        self.bus = smbus.SMBus(i2c_bus)
        self.output_range = output_range
        self._declination = 0.0
        self._calibration = [[1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0],
                             [0.0, 0.0, 1.0]]
        chip_id = self._read_byte(REG_CHIP_ID)
        if chip_id != 0xff:
            msg = f"Chip ID returned `0x{chip_id:x}` instead of `0xff`; is it the wrong chip?"
            self._warn(msg)
        self.mode_cont = (MODE_CONT | output_data_rate | output_range | oversampling_rate)
        self.mode_stby = (MODE_STBY | ODR_10HZ | RNG_2G | OSR_64)
        self.mode_continuous()

    def __del__(self):
        """
        Once finished using the sensor, switch to standby mode.
        """
        self.mode_standby()

    def mode_continuous(self) -> None:
        """
        Sets the device in continuous read mode.
        """
        self._write_byte(REG_CONTROL_2, SOFT_RST)           # Soft reset.
        self._write_byte(REG_CONTROL_2, INT_ENB)            # Disable interrupt.
        self._write_byte(REG_RST_PERIOD, 0x01)              # Define SET/RESET period.
        self._write_byte(REG_CONTROL_1, self.mode_cont)     # Set operation mode to continuous.

    def mode_standby(self) -> None:
        """
        Sets the device in standby mode.
        """
        self._write_byte(REG_CONTROL_2, SOFT_RST)           # Soft reset.
        self._write_byte(REG_CONTROL_2, INT_ENB)            # Disable interrupt.
        self._write_byte(REG_RST_PERIOD, 0x01)              # Define SET/RESET period.
        self._write_byte(REG_CONTROL_1, self.mode_stby)     # Set operation mode to standby.

    def _write_byte(self, registry: int, value: int) -> None:
        """
        Writes a byte value to a register.
        """
        self.bus.write_byte_data(self.address, registry, value)
        sleep(0.01)

    def _read_byte(self, registry: int) -> int:
        """
        Reads a byte value from a register.
        """
        return self.bus.read_byte_data(self.address, registry)

    def _read_word(self, registry: int) -> int:
        """
        Reads a 2-byte value stored as 𝑳𝑺𝑩 and 𝑴𝑺𝑩.
        """
        low = self.bus.read_byte_data(self.address, registry)
        high = self.bus.read_byte_data(self.address, registry + 1)
        val = (high << 8) + low
        return val

    def _read_word_2c(self, registry: int) -> int:
        """
        Calculates the 2's complement of a 2-byte value.
        """
        val = self._read_word(registry)
        if val >= 0x8000:           # 32768
            return val - 0x10000    # 65536
        else: return val


    def get_data(self) -> list[float]:
        """
        Reads data from magnetic and temperature data registers.

        Returns
        -------
        A list of 4 numbers:
        - 𝑿 ⎯ magnetic sensor value on axis 𝑿.
        - 𝒀 ⎯ magnetic sensor value on axis 𝒀.
        - 𝒁 ⎯ magnetic sensor value on axis 𝒁.
        - 𝑻 ⎯ temperature sensor value.
        """
        i = 0
        [x, y, z, t] = [None, None, None, None]
        while i < 20:                                   # Timeout after about 0.20 seconds.
            status = self._read_byte(REG_STATUS_1)
            if status & STAT_OVL:                       # Some values have reached an overflow.
                msg = ("Magnetic sensor overflow.")
                if self.output_range == RNG_2G:
                    msg += " Consider switching to RNG_8G output range."
                self._warn(msg)
            if status & STAT_DOR:
                x = self._read_word_2c(REG_XOUT_LSB)    # Previous measure was read partially, sensor in Data Lock.
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                continue
            if status & STAT_DRDY:                      # Data is ready to read.
                x = self._read_word_2c(REG_XOUT_LSB)
                y = self._read_word_2c(REG_YOUT_LSB)
                z = self._read_word_2c(REG_ZOUT_LSB)
                t = self._read_word_2c(REG_TOUT_LSB)
                break
            else:                                       # Waiting for DRDY.
                sleep(0.01)
                i += 1
        return [x, y, z, t]         # type: ignore


    def get_magnet_raw(self) -> list[float]:
        """
        Gets the 3 axis values from magnetic sensor.

        Returns
        -------
        A list of 3 numbers:
        - 𝑿 ⎯ magnetic sensor value on axis 𝑿.
        - 𝒀 ⎯ magnetic sensor value on axis 𝒀.
        - 𝒁 ⎯ magnetic sensor value on axis 𝒁.
        """
        [x, y, z, t] = self.get_data()
        return [x, y, z]


    def get_magnet(self) -> list[float]:
        """
        Gets the horizontal magnetic sensor vector components, with (𝒙, 𝒚) calibration applied.

        Returns
        -------
        A list of 2 numbers:
        - 𝑿 ⎯ magnetic sensor value on axis 𝑿.
        - 𝒀 ⎯ magnetic sensor value on axis 𝒀.
        """
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            [x1, y1] = [x, y]
        else:
            c = self._calibration
            x1 = x * c[0][0] + y * c[0][1] + c[0][2]
            y1 = x * c[1][0] + y * c[1][1] + c[1][2]
        return [x1, y1]


    def get_bearing_raw(self) -> float:
        """
        Gets the horizontal bearing (in degrees), without calibration or declination adjustment.
        
        Bearing is the angle between the magnetic north and the projection of the magnetic vector on the horizontal plane.
        
        Returns
        -------
        A float representing the horizontal bearing, in degrees.
        """
        [x, y, z] = self.get_magnet_raw()
        if x is None or y is None:
            return None                     # type: ignore
        else:
            b = degrees(atan2(y, x))
            if b < 0:
                b += 360.0
            return b


    def get_bearing(self) -> float:
        """
        Gets the horizontal bearing (in degrees), adjusted by calibration and declination.

        Adjusted bearing is the angle between the magnetic north and the projection of the calibrated magnetic vector on the horizontal plane, adjusted by the declination angle.
        
        Returns
        -------
        A float representing the horizontal bearing, in degrees.
        """
        [x, y] = self.get_magnet()
        if x is None or y is None:
            return None                     # type: ignore
        else:
            b = degrees(atan2(y, x))
            if b < 0:
                b += 360.0
            b += self._declination
            if b < 0.0:
                b += 360.0
            elif b >= 360.0:
                b -= 360.0
        return b


    def get_temp(self) -> float:
        """
        Gets the raw (uncalibrated) data from the temperature sensor.

        Returns
        -------
        A float representing the temperature, in degrees Celsius.
        """
        [x, y, z, t] = self.get_data()
        return t


    def set_declination(self, value: float) -> None:
        """
        Sets the magnetic declination, in degrees.

        Magnetic declination is the angle between the magnetic north and the geographic north, measured clockwise.

        Parameters
        ----------
        `value` ⎯ The magnetic declination, in degrees.
        """
        try:
            d = float(value)
            if d < -180.0 or d > 180.0:
                msg = (f"Declination must be >= -180 and <= 180. Got {d}.")
                self._warn(msg)
            else:
                self._declination = d
        except:
            self._warn(f"Declination must be a `float` value. Got `{value}` (type: {type(value)}).")


    def get_declination(self) -> float:
        """
        Gets the current set value of magnetic declination.

        Returns
        -------
        A float representing the magnetic declination, in degrees.
        """
        return self._declination


    def set_calibration(self, value: list[list[float]]) -> None:
        """
        Sets the 3x3 matrix for horizontal (𝒙, 𝒚) magnetic vector calibration.

        Parameters
        ----------
        `value` ⎯ A 3x3 matrix (list of lists containing floats) of values.
        """
        c = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        try:
            for i in range(0, 3):
                for j in range(0, 3):
                    c[i][j] = float(value[i][j])
            self._calibration = c
        except:
            self._warn(f"Calibration must be a 3x3 `float` matrix. Got `{value}`.")


    def get_calibration(self) -> list[list[float]]:
        """
        Gets the current set value of the calibration matrix.

        Returns
        -------
        A 3x3 matrix (list of lists containing floats) of values corresponding to the current calibration matrix.
        """
        return self._calibration


    declination = property(fget=get_declination,
                           fset=set_declination,
                           doc=u'Magnetic declination to adjust bearing.')

    calibration = property(fget=get_calibration,
                           fset=set_calibration,
                           doc=u'Transformation matrix to adjust (𝒙, 𝒚) magnetic vector.')


