use eh1::spi::blocking::{SpiBusRead, SpiBusWrite};

/// BME280 bus.
#[derive(Debug)]
pub struct Bme280Bus<SPI> {
    spi: SPI,
}

/// SPI mode for the BME280.
///
/// The BME280 also supports mode 3.
pub const MODE: eh1::spi::Mode = eh1::spi::MODE_0;

impl<SPI, E> Bme280Bus<SPI>
where
    SPI: eh1::spi::blocking::SpiDevice<Error = E>,
{
    /// Creates a new `Bme280Bus` from a SPI peripheral and a chip select
    /// digital I/O pin.
    ///
    /// # Safety
    ///
    /// The chip select pin must be high before being passed to this function.
    ///
    /// # Example
    ///
    /// ```
    /// # let spi = ehm1::spi::Mock::new(&[]);
    /// use bme280_multibus::spi1::Bme280Bus;
    ///
    /// let mut bme: Bme280Bus<_> = Bme280Bus::new(spi);
    /// # Ok::<(), ehm1::MockError>(())
    /// ```
    #[inline]
    pub fn new(spi: SPI) -> Self {
        Bme280Bus { spi }
    }

    /// Free the SPI bus and CS pin from the W5500.
    ///
    /// # Example
    ///
    /// ```
    /// # let spi = ehm1::spi::Mock::new(&[]);
    /// use bme280_multibus::spi1::Bme280Bus;
    ///
    /// let mut bme: Bme280Bus<_> = Bme280Bus::new(spi);
    /// let spi = bme.free();
    /// # Ok::<(), ehm1::MockError>(())
    /// ```
    #[inline]
    pub fn free(self) -> SPI {
        self.spi
    }
}

impl<SPI, E> crate::Bme280Bus for Bme280Bus<SPI>
where
    SPI: eh1::spi::blocking::SpiDevice<Error = E>,
    SPI::Bus:
        eh1::spi::blocking::SpiBusRead<Error = E> + eh1::spi::blocking::SpiBusWrite<Error = E>,
{
    type Error = E;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transaction(|spi| {
            spi.write(&[reg | (1 << 7)])?;
            spi.read(buf)
        })
    }

    fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error> {
        let buf: [u8; 2] = [reg & !(1 << 7), data];
        self.spi.transaction(|spi| spi.write(&buf))
    }
}