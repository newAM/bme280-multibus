/// BME280 bus.
#[derive(Debug)]
pub struct Bme280Bus<SPI> {
    spi: SPI,
}

/// SPI mode for the BME280.
///
/// The BME280 also supports mode 3.
pub const MODE: eh1::spi::Mode = eh1::spi::MODE_0;

impl<SPI> Bme280Bus<SPI> {
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
    /// # let spi = ehm::eh1::spi::Mock::new(&[]);
    /// use bme280_multibus::spi1::Bme280Bus;
    ///
    /// let mut bme: Bme280Bus<_> = Bme280Bus::new(spi);
    /// # bme.free().done();
    /// # Ok::<(), ehm::eh1::MockError>(())
    /// ```
    #[inline]
    #[allow(clippy::unnecessary_safety_doc)]
    pub fn new(spi: SPI) -> Self {
        Bme280Bus { spi }
    }

    /// Free the SPI bus and CS pin from the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # let spi = ehm::eh1::spi::Mock::new(&[]);
    /// use bme280_multibus::spi1::Bme280Bus;
    ///
    /// let mut bme: Bme280Bus<_> = Bme280Bus::new(spi);
    /// let mut spi = bme.free();
    /// # spi.done();
    /// # Ok::<(), ehm::eh1::MockError>(())
    /// ```
    #[inline]
    pub fn free(self) -> SPI {
        self.spi
    }
}

impl<SPI> crate::Bme280Bus for Bme280Bus<SPI>
where
    SPI: eh1::spi::SpiDevice,
{
    type Error = SPI::Error;

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        let a = &[reg | (1 << 7)];
        let mut ops = [
            eh1::spi::Operation::Write(a),
            eh1::spi::Operation::Read(buf),
        ];
        self.spi.transaction(&mut ops)
    }

    fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error> {
        let buf = &[reg & !(1 << 7), data];
        let mut ops = [eh1::spi::Operation::Write(buf)];
        self.spi.transaction(&mut ops)
    }
}

impl<SPI> crate::Bme280BusAsync for Bme280Bus<SPI>
where
    SPI: eha0a::spi::SpiDevice,
{
    type Error = SPI::Error;

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        let a = &[reg | (1 << 7)];
        let mut ops = [
            eh1::spi::Operation::Write(a),
            eh1::spi::Operation::Read(buf),
        ];
        self.spi.transaction(&mut ops).await
    }

    async fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error> {
        let buf = &[reg & !(1 << 7), data];
        let mut ops = [eh1::spi::Operation::Write(buf)];
        self.spi.transaction(&mut ops).await
    }
}
