/// BME280 device.
#[derive(Debug)]
pub struct Bme280Bus<SPI, CS> {
    bus: SPI,
    cs: CS,
}

/// SPI mode for the BME280.
///
/// The BME280 also supports mode 3.
pub const MODE: embedded_hal::spi::Mode = embedded_hal::spi::MODE_0;

/// Maximum SPI bus frequency in hertz.
pub const MAX_FREQ: u32 = 10_000_000;

/// BME280 error type.
#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI bus error wrapper.
    Spi(SpiError),
    /// GPIO pin error wrapper.
    Pin(PinError),
}

impl<SpiError, PinError> From<PinError> for Error<SpiError, PinError> {
    fn from(e: PinError) -> Self {
        Error::Pin(e)
    }
}

impl<SPI, CS, SpiError, PinError> Bme280Bus<SPI, CS>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: embedded_hal::digital::v2::OutputPin<Error = PinError>,
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
    /// # use embedded_hal_mock as hal;
    /// # let spi = hal::spi::Mock::new(&[]);
    /// # let mut pin = hal::pin::Mock::new(&[
    /// #    hal::pin::Transaction::set(hal::pin::State::High),
    /// # ]);
    /// use bme280::spi::Bme280Bus;
    /// use embedded_hal::digital::v2::OutputPin;
    ///
    /// pin.set_high()?;
    /// let mut bme: Bme280Bus<_, _> = Bme280Bus::new(spi, pin);
    /// # Ok::<(), hal::MockError>(())
    /// ```
    pub fn new(bus: SPI, cs: CS) -> Self {
        Bme280Bus { bus, cs }
    }

    /// Free the SPI bus and CS pin from the W5500.
    ///
    /// # Example
    ///
    /// ```
    /// # use embedded_hal_mock as hal;
    /// # let spi = hal::spi::Mock::new(&[]);
    /// # let mut pin = hal::pin::Mock::new(&[
    /// #    hal::pin::Transaction::set(hal::pin::State::High),
    /// # ]);
    /// use bme280::spi::Bme280Bus;
    /// use embedded_hal::digital::v2::OutputPin;
    ///
    /// pin.set_high()?;
    /// let mut bme: Bme280Bus<_, _> = Bme280Bus::new(spi, pin);
    /// let (spi, pin) = bme.free();
    /// # Ok::<(), hal::MockError>(())
    /// ```
    pub fn free(self) -> (SPI, CS) {
        (self.bus, self.cs)
    }

    #[inline(always)]
    fn with_chip_enable<T, E, F>(&mut self, mut f: F) -> Result<T, E>
    where
        F: FnMut(&mut SPI) -> Result<T, E>,
        E: core::convert::From<Error<SpiError, PinError>>,
    {
        self.cs.set_low().map_err(Error::Pin)?;
        let result: Result<T, E> = f(&mut self.bus);
        self.cs.set_high().map_err(Error::Pin)?;
        result
    }
}

impl<SPI, CS, SpiError, PinError> crate::Bme280Bus for Bme280Bus<SPI, CS>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: embedded_hal::digital::v2::OutputPin<Error = PinError>,
{
    type Error = Error<SpiError, PinError>;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.with_chip_enable(|spi| {
            spi.write(&[reg | (1 << 7)]).map_err(Error::Spi)?;
            spi.transfer(buf).map_err(Error::Spi)?;
            Ok(())
        })
    }

    fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error> {
        let mut buf: [u8; 2] = [reg & !(1 << 7), data];
        self.with_chip_enable(|spi| {
            spi.transfer(&mut buf).map_err(Error::Spi)?;
            Ok(())
        })
    }
}
