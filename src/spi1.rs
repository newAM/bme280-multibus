use eh1::spi::blocking::{SpiBusRead, SpiBusWrite};

#[cfg(feature = "async")]
use eha0::spi::{SpiBusRead as SpiBusReadAsync, SpiBusWrite as SpiBusWriteAsync};

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
    SPI::Bus: eh1::spi::blocking::SpiBusRead + eh1::spi::blocking::SpiBusWrite,
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

#[cfg(feature = "async")]
impl<SPI, E> crate::Bme280BusAsync for Bme280Bus<SPI>
where
    SPI: eha0::spi::SpiDevice<Error = E>,
    <SPI as eha0::spi::SpiDevice>::Bus: eha0::spi::SpiBusRead + eha0::spi::SpiBusWrite,
{
    type Error = E;

    type ReadFuture<'a> = impl core::future::Future<Output = Result<(), Self::Error>> + 'a
        where Self: 'a, E: 'a;

    #[allow(unsafe_code)]
    fn read_regs<'a>(&'a mut self, reg: u8, buf: &'a mut [u8]) -> Self::ReadFuture<'a> {
        async move {
            eha0::spi::SpiDevice::transaction(&mut self.spi, move |bus| async move {
                let bus = unsafe { &mut *bus };
                bus.write(&[reg | (1 << 7)]).await?;
                bus.read(buf).await
            })
            .await
        }
    }

    type WriteFuture<'a> = impl core::future::Future<Output = Result<(), Self::Error>> + 'a
        where Self: 'a, E: 'a;

    #[allow(unsafe_code)]
    fn write_reg(&mut self, reg: u8, data: u8) -> Self::WriteFuture<'_> {
        let buf: [u8; 2] = [reg & !(1 << 7), data];
        async move { self.spi.write(&buf).await }
    }

    type CalibrateFuture<'a> = impl core::future::Future<Output = Result<crate::Calibration, Self::Error>> + 'a
        where Self: 'a, E: 'a;

    fn calibration(&mut self) -> Self::CalibrateFuture<'_> {
        async move {
            const FIRST: usize = (crate::reg::CALIB_25 - crate::reg::CALIB_00 + 1) as usize;
            debug_assert_eq!(FIRST, 26);
            const SECOND: usize = (crate::reg::CALIB_32 - crate::reg::CALIB_26 + 1) as usize;
            debug_assert_eq!((FIRST + SECOND), crate::NUM_CALIB_REG);

            let mut buf: [u8; crate::NUM_CALIB_REG] = [0; crate::NUM_CALIB_REG];
            self.read_regs(crate::reg::CALIB_00, &mut buf[0..FIRST])
                .await?;
            self.read_regs(crate::reg::CALIB_26, &mut buf[FIRST..(FIRST + SECOND)])
                .await?;

            Ok(buf.into())
        }
    }
}
