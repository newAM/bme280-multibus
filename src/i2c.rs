/// I2C device address.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
#[repr(u8)]
pub enum Address {
    /// SDO pin is connected to GND.
    SdoGnd = 0x76,
    /// SDO pin is connected to V<sub>DDIO</sub>
    SdoVddio = 0x77,
}

/// BME280 bus.
#[derive(Debug)]
pub struct Bme280Bus<I2C> {
    address: u8,
    bus: I2C,
}

impl<I2C, E> Bme280Bus<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    /// Creates a new `Bme280Bus` from a I2C peripheral, and an I2C
    /// device address.
    ///
    /// # Example
    ///
    /// ```
    /// # use embedded_hal_mock as hal;
    /// # let i2c = hal::i2c::Mock::new(&[]);
    /// use bme280::i2c::{Address, Bme280Bus};
    ///
    /// let mut bme: Bme280Bus<_> = Bme280Bus::new(i2c, Address::SdoGnd);
    /// ```
    #[inline]
    pub fn new(bus: I2C, address: Address) -> Self {
        Self {
            bus,
            address: address as u8,
        }
    }

    /// Free the I2C bus from the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # use embedded_hal_mock as hal;
    /// # let i2c = hal::i2c::Mock::new(&[]);
    /// use bme280::i2c::{Address, Bme280Bus};
    ///
    /// let mut bme: Bme280Bus<_> = Bme280Bus::new(i2c, Address::SdoGnd);
    /// let i2c = bme.free();
    /// ```
    #[inline]
    pub fn free(self) -> I2C {
        self.bus
    }
}

impl<I2C, E> crate::Bme280Bus for Bme280Bus<I2C>
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    type Error = E;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address, &[reg], buf)
    }

    fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address, &[reg, data])
    }
}
