//! BME280 driver with support for I2C and SPI buses.
//!
//! # Example
//!
//! ```
//! # let i2c = ehm::eh0::i2c::Mock::new(&[
//! #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
//! #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
//! #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF2, 0b100]),
//! #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF4, 0b10010011]),
//! #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF5, 0b10110000]),
//! #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xF7], vec![0; 8]),
//! # ]);
//! use bme280_multibus::{i2c0::Address, Bme280, Sample, Standby};
//!
//! const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
//!     config: bme280_multibus::Config::RESET
//!         .set_standby_time(bme280_multibus::Standby::Millis1000)
//!         .set_filter(bme280_multibus::Filter::X16),
//!     ctrl_meas: bme280_multibus::CtrlMeas::RESET
//!         .set_osrs_t(bme280_multibus::Oversampling::X8)
//!         .set_osrs_p(bme280_multibus::Oversampling::X8)
//!         .set_mode(bme280_multibus::Mode::Normal),
//!     ctrl_hum: bme280_multibus::Oversampling::X8,
//! };
//!
//! let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
//! bme.settings(&SETTINGS)?;
//! let sample: Sample = bme.sample().unwrap();
//! # bme.free().free().done();
//! # Ok::<(), ehm::eh0::MockError>(())
//! ```
//!
//! # Features
//!
//! * `async`: Enable asynchronous implementations with `embedded-hal-async`.
//!   Requires a nightly toolchain.
//! * `serde`: Implement `Serialize` and `Deserialize` for `Sample`.
#![no_std]
#![cfg_attr(feature = "async", allow(async_fn_in_trait))]
#![cfg_attr(docsrs, feature(doc_cfg), feature(doc_auto_cfg))]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

use core::time::Duration;

pub use eh0;
pub use eh1;
#[cfg(feature = "async")]
pub use eha0a;

/// BME280 I2C bus implementation with embedded-hal version 0.2
pub mod i2c0;
/// BME280 I2C bus implementation with embedded-hal version 1
pub mod i2c1;
/// BME280 SPI bus implementation with embedded-hal version 0.2
pub mod spi0;
/// BME280 SPI bus implementation with embedded-hal version 1
pub mod spi1;

/// BME280 chip ID.
pub const CHIP_ID: u8 = 0x60;

const NUM_CALIB_REG: usize = 33;
const NUM_MEAS_REG: usize = 8;

/// Maximum SPI bus frequency in hertz.
pub const SPI_MAX_FREQ: u32 = 10_000_000;

/// BME280 calibration data.
#[derive(Debug)]
#[allow(missing_docs)]
pub struct Calibration {
    t1: u16, // 0x88..0x89 buf[00:01]
    t2: i16, // 0x8A..0x8B buf[02:03]
    t3: i16, // 0x8C..0x8D buf[04:05]
    p1: u16, // 0x8E..0x8F buf[06:07]
    p2: i16, // 0x90..0x91 buf[08:09]
    p3: i16, // 0x92..0x93 buf[10:11]
    p4: i16, // 0x94..0x95 buf[12:13]
    p5: i16, // 0x96..0x97 buf[14:15]
    p6: i16, // 0x98..0x99 buf[16:17]
    p7: i16, // 0x9A..0x9B buf[18:19]
    p8: i16, // 0x9C..0x9D buf[20:21]
    p9: i16, // 0x9E..0x9F buf[22:23]
    // INTENTIONAL ONE BYTE GAP (see datasheet)
    h1: u8,  // 0xA1       buf[25]
    h2: i16, // 0xE1..0xE2 buf[26:27]
    h3: u8,  // 0xE3       buf[28]
    h4: i16, // 0xE4..0xE5[3:0] = H4 [11:4]..[3:0]
    h5: i16, // 0xE5[7:4]..0xE6 = H5 [3:0]..[11:4]
    h6: i8,  // 0xE7       buf[32]
}

impl From<[u8; NUM_CALIB_REG]> for Calibration {
    fn from(buf: [u8; NUM_CALIB_REG]) -> Self {
        Calibration {
            t1: u16::from_le_bytes([buf[0], buf[1]]),
            t2: i16::from_le_bytes([buf[2], buf[3]]),
            t3: i16::from_le_bytes([buf[4], buf[5]]),
            p1: u16::from_le_bytes([buf[6], buf[7]]),
            p2: i16::from_le_bytes([buf[8], buf[9]]),
            p3: i16::from_le_bytes([buf[10], buf[11]]),
            p4: i16::from_le_bytes([buf[12], buf[13]]),
            p5: i16::from_le_bytes([buf[14], buf[15]]),
            p6: i16::from_le_bytes([buf[16], buf[17]]),
            p7: i16::from_le_bytes([buf[18], buf[19]]),
            p8: i16::from_le_bytes([buf[20], buf[21]]),
            p9: i16::from_le_bytes([buf[22], buf[23]]),
            // INTENTIONAL ONE BYTE GAP (see datasheet)
            h1: buf[25],
            h2: i16::from_le_bytes([buf[26], buf[27]]),
            h3: buf[28],
            h4: ((buf[29] as i8 as i16) << 4 | (buf[30] as i8 as i16) & 0xF),
            h5: (((buf[30] as i8 as i16) & 0xF0) >> 4) | ((buf[31] as i8 as i16) << 4),
            h6: buf[32] as i8,
        }
    }
}

const RESET_MAGIC: u8 = 0xB6;

/// Register addresses.
///
/// from Table 18: Memory Map
#[allow(dead_code)]
mod reg {
    pub const HUM_LSB: u8 = 0xFE;
    pub const HUM_MSB: u8 = 0xFB;
    pub const TEMP_XLSB: u8 = 0xFC;
    pub const TEMP_LSB: u8 = 0xFB;
    pub const TEMP_MSB: u8 = 0xFA;
    pub const PRESS_XLSB: u8 = 0xF9;
    pub const PRESS_LSB: u8 = 0xF8;
    pub const PRESS_MSB: u8 = 0xF7;
    pub const CONFIG: u8 = 0xF5;
    pub const CTRL_MEAS: u8 = 0xF4;
    pub const STATUS: u8 = 0xF3;
    pub const CTRL_HUM: u8 = 0xF2;
    pub const CALIB_41: u8 = 0xF0;
    pub const CALIB_40: u8 = 0xEF;
    pub const CALIB_39: u8 = 0xEE;
    pub const CALIB_38: u8 = 0xED;
    pub const CALIB_37: u8 = 0xEC;
    pub const CALIB_36: u8 = 0xEB;
    pub const CALIB_35: u8 = 0xEA;
    pub const CALIB_34: u8 = 0xE9;
    pub const CALIB_33: u8 = 0xE8;
    pub const CALIB_32: u8 = 0xE7;
    pub const CALIB_31: u8 = 0xE6;
    pub const CALIB_30: u8 = 0xE5;
    pub const CALIB_29: u8 = 0xE4;
    pub const CALIB_28: u8 = 0xE3;
    pub const CALIB_27: u8 = 0xE2;
    pub const CALIB_26: u8 = 0xE1;
    pub const RESET: u8 = 0xE0;
    pub const ID: u8 = 0xD0;
    pub const CALIB_25: u8 = 0xA1;
    pub const CALIB_24: u8 = 0xA0;
    pub const CALIB_23: u8 = 0x9F;
    pub const CALIB_22: u8 = 0x9E;
    pub const CALIB_21: u8 = 0x9D;
    pub const CALIB_20: u8 = 0x9C;
    pub const CALIB_19: u8 = 0x9B;
    pub const CALIB_18: u8 = 0x9A;
    pub const CALIB_17: u8 = 0x99;
    pub const CALIB_16: u8 = 0x98;
    pub const CALIB_15: u8 = 0x97;
    pub const CALIB_14: u8 = 0x96;
    pub const CALIB_13: u8 = 0x95;
    pub const CALIB_12: u8 = 0x94;
    pub const CALIB_11: u8 = 0x93;
    pub const CALIB_10: u8 = 0x92;
    pub const CALIB_09: u8 = 0x91;
    pub const CALIB_08: u8 = 0x90;
    pub const CALIB_07: u8 = 0x8F;
    pub const CALIB_06: u8 = 0x8E;
    pub const CALIB_05: u8 = 0x8D;
    pub const CALIB_04: u8 = 0x8C;
    pub const CALIB_03: u8 = 0x8B;
    pub const CALIB_02: u8 = 0x8A;
    pub const CALIB_01: u8 = 0x89;
    pub const CALIB_00: u8 = 0x88;
}

/// Oversampling settings for temperature, pressure, and humidity data.
#[derive(Debug, PartialEq, Eq, Clone, Copy, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Oversampling {
    /// Skipped, output set to `0x80000`.
    Skip = 0b000,
    /// Oversampling × 1
    X1 = 0b001,
    /// Oversampling × 2
    X2 = 0b010,
    /// Oversampling × 4
    X4 = 0b011,
    /// Oversampling × 8
    X8 = 0b100,
    /// Oversampling × 16
    X16 = 0b101,
}

impl From<Oversampling> for u8 {
    fn from(x: Oversampling) -> Self {
        x as u8
    }
}

impl Oversampling {
    /// Reset value of the osrs fields.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Oversampling;
    ///
    /// assert_eq!(Oversampling::RESET, Oversampling::Skip);
    /// ```
    pub const RESET: Self = Self::Skip;
}

impl Default for Oversampling {
    fn default() -> Self {
        Oversampling::RESET
    }
}

/// Sensor mode.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[repr(u8)]
pub enum Mode {
    /// Sleep mode.
    Sleep = 0b00,
    /// Forced mode.
    Forced = 0b01,
    /// Normal mode.
    Normal = 0b11,
}

impl From<Mode> for u8 {
    fn from(x: Mode) -> Self {
        x as u8
    }
}

impl Mode {
    /// Reset value of the mode field in the [`CtrlMeas`] register.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Mode;
    ///
    /// assert_eq!(Mode::RESET, Mode::Sleep);
    /// ```
    pub const RESET: Self = Mode::Sleep;
}

impl Default for Mode {
    fn default() -> Self {
        Mode::RESET
    }
}

/// t<sub>standby</sub> settings.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
#[repr(u8)]
pub enum Standby {
    /// 0.5 ms
    Micros500 = 0b000,
    /// 62.5 ms
    Micros62500 = 0b001,
    /// 125 ms
    Millis125 = 0b010,
    /// 250 ms
    Millis250 = 0b011,
    /// 500 ms
    Millis500 = 0b100,
    /// 1000 ms
    Millis1000 = 0b101,
    /// 10 ms
    Millis10 = 0b110,
    /// 20 ms
    Millis20 = 0b111,
}

impl Standby {
    /// Reset value of the standby field in the [`Config`] register.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Standby;
    ///
    /// assert_eq!(Standby::RESET, Standby::Micros500);
    /// ```
    pub const RESET: Self = Standby::Micros500;

    /// Convert the standby enumeration to a duration.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Standby;
    /// use core::time::Duration;
    ///
    /// assert_eq!(Standby::Micros500.duration(), Duration::from_micros(500));
    /// assert_eq!(
    ///     Standby::Micros62500.duration(),
    ///     Duration::from_micros(62500)
    /// );
    /// assert_eq!(Standby::Millis125.duration(), Duration::from_millis(125));
    /// assert_eq!(Standby::Millis250.duration(), Duration::from_millis(250));
    /// assert_eq!(Standby::Millis500.duration(), Duration::from_millis(500));
    /// assert_eq!(Standby::Millis1000.duration(), Duration::from_millis(1000));
    /// assert_eq!(Standby::Millis10.duration(), Duration::from_millis(10));
    /// assert_eq!(Standby::Millis20.duration(), Duration::from_millis(20));
    /// ```
    pub const fn duration(&self) -> Duration {
        match self {
            Standby::Micros500 => Duration::from_micros(500),
            Standby::Micros62500 => Duration::from_micros(62500),
            Standby::Millis125 => Duration::from_millis(125),
            Standby::Millis250 => Duration::from_millis(250),
            Standby::Millis500 => Duration::from_millis(500),
            Standby::Millis1000 => Duration::from_millis(1000),
            Standby::Millis10 => Duration::from_millis(10),
            Standby::Millis20 => Duration::from_millis(20),
        }
    }
}

impl From<&Standby> for Duration {
    fn from(s: &Standby) -> Self {
        s.duration()
    }
}

impl From<Standby> for Duration {
    fn from(s: Standby) -> Self {
        s.duration()
    }
}

impl PartialOrd for Standby {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.duration().cmp(&other.duration()))
    }
}

impl Ord for Standby {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.duration().cmp(&other.duration())
    }
}

impl Default for Standby {
    fn default() -> Self {
        Standby::RESET
    }
}

/// Filter settings.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Hash)]
#[repr(u8)]
pub enum Filter {
    /// Filter off.
    Off = 0b000,
    /// Filter coefficient of 2.
    X2 = 0b001,
    /// Filter coefficient of 4.
    X4 = 0b010,
    /// Filter coefficient of 8.
    X8 = 0b011,
    /// Filter coefficient of 16.
    X16 = 0b100,
}

impl Filter {
    /// Reset value of the filter field in the [`Config`] register.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Filter;
    ///
    /// assert_eq!(Filter::RESET, Filter::Off);
    /// ```
    pub const RESET: Self = Filter::Off;
}

impl Default for Filter {
    fn default() -> Self {
        Filter::RESET
    }
}

impl From<Filter> for u8 {
    fn from(x: Filter) -> Self {
        x as u8
    }
}

/// Config register.
///
/// This register sets the rate, filter, and interface options of the device.
/// Writes to the config register in normal mode may be ignored.
/// In sleep mode writes are not ignored.
///
/// All methods on this struct are constant so that you can create a
/// configuration value at compile time.
///
/// # Example
///
/// ```
/// use bme280_multibus::{Config, Filter, Standby};
///
/// const CONFIG: Config = Config::RESET
///     .set_standby_time(Standby::Millis1000)
///     .set_filter(Filter::X16)
///     .set_spi3w_en(false);
/// ```
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Config(u8);

impl Config {
    /// Get the reset value of the config register.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Config;
    ///
    /// assert_eq!(Config::RESET, Config::default());
    /// ```
    pub const RESET: Self = Config(0x00);

    /// Set the inactive duration t<sub>standby</sub> in normal mode.
    ///
    /// See [`Standby`] for settings, and chapter 3.3.4 in the [datasheet] for
    /// details.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::{Config, Standby};
    ///
    /// let mut cfg: Config = Config::default();
    /// assert_eq!(cfg.standby_time(), Standby::default());
    /// cfg = cfg.set_standby_time(Standby::Micros500);
    /// assert_eq!(cfg.standby_time(), Standby::Micros500);
    /// cfg = cfg.set_standby_time(Standby::Micros62500);
    /// assert_eq!(cfg.standby_time(), Standby::Micros62500);
    /// cfg = cfg.set_standby_time(Standby::Millis125);
    /// assert_eq!(cfg.standby_time(), Standby::Millis125);
    /// cfg = cfg.set_standby_time(Standby::Millis250);
    /// assert_eq!(cfg.standby_time(), Standby::Millis250);
    /// cfg = cfg.set_standby_time(Standby::Millis500);
    /// assert_eq!(cfg.standby_time(), Standby::Millis500);
    /// cfg = cfg.set_standby_time(Standby::Millis1000);
    /// assert_eq!(cfg.standby_time(), Standby::Millis1000);
    /// cfg = cfg.set_standby_time(Standby::Millis10);
    /// assert_eq!(cfg.standby_time(), Standby::Millis10);
    /// cfg = cfg.set_standby_time(Standby::Millis20);
    /// assert_eq!(cfg.standby_time(), Standby::Millis20);
    /// ```
    ///
    /// [datasheet]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    #[must_use = "set_standby_time returns a modified Config"]
    pub const fn set_standby_time(self, s: Standby) -> Config {
        Config((self.0 & 0x1F) | ((s as u8) << 5))
    }

    /// Get the standby time.
    pub const fn standby_time(&self) -> Standby {
        match self.0 >> 5 {
            0b000 => Standby::Micros500,
            0b001 => Standby::Micros62500,
            0b010 => Standby::Millis125,
            0b011 => Standby::Millis250,
            0b100 => Standby::Millis500,
            0b101 => Standby::Millis1000,
            0b110 => Standby::Millis10,
            _ => Standby::Millis20,
        }
    }

    /// Set the time constant of the IIR filter.
    ///
    /// See [`Filter`] for settings, and chapter 3.4.4 in the [datasheet] for
    /// details.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::{Config, Filter};
    ///
    /// let mut cfg: Config = Config::default();
    /// assert_eq!(cfg.filter(), Filter::default());
    /// cfg = cfg.set_filter(Filter::Off);
    /// assert_eq!(cfg.filter(), Filter::Off);
    /// cfg = cfg.set_filter(Filter::X2);
    /// assert_eq!(cfg.filter(), Filter::X2);
    /// cfg = cfg.set_filter(Filter::X4);
    /// assert_eq!(cfg.filter(), Filter::X4);
    /// cfg = cfg.set_filter(Filter::X8);
    /// assert_eq!(cfg.filter(), Filter::X8);
    /// cfg = cfg.set_filter(Filter::X16);
    /// assert_eq!(cfg.filter(), Filter::X16);
    /// ```
    ///
    /// [datasheet]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    #[must_use = "set_filter returns a modified Config"]
    pub const fn set_filter(self, f: Filter) -> Config {
        Config((self.0 & 0b11100011) | ((f as u8) << 2))
    }

    /// Get the filter coefficient.
    pub const fn filter(&self) -> Filter {
        match (self.0 >> 2) & 0b111 {
            0b000 => Filter::Off,
            0b001 => Filter::X2,
            0b010 => Filter::X4,
            0b011 => Filter::X8,
            _ => Filter::X16,
        }
    }

    /// Enables the 3-wire SPI itnerface when enabled.
    ///
    /// See chapter 6.3 in the [datasheet] for details.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Config;
    ///
    /// let mut cfg: Config = Config::default();
    /// assert_eq!(cfg.spi3w_en(), false);
    /// cfg = cfg.set_spi3w_en(true);
    /// assert_eq!(cfg.spi3w_en(), true);
    /// cfg = cfg.set_spi3w_en(false);
    /// assert_eq!(cfg.spi3w_en(), false);
    /// ```
    ///
    /// [datasheet]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    #[must_use = "set_spi3w_en returns a modified Config"]
    pub const fn set_spi3w_en(self, en: bool) -> Config {
        if en {
            Config(self.0 | 0b1)
        } else {
            Config(self.0 & !0b1)
        }
    }

    /// Returns `true` if 3-wire SPI is enabled.
    pub const fn spi3w_en(&self) -> bool {
        self.0 & 0b1 == 0b1
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::RESET
    }
}

/// Measurement control register.
///
/// This configures the pressure and temperature data acquisition options of the
/// device.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct CtrlMeas(u8);

impl CtrlMeas {
    /// Get the reset value of the ctrl_meas register.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::CtrlMeas;
    ///
    /// assert_eq!(CtrlMeas::RESET, CtrlMeas::default());
    /// ```
    pub const RESET: Self = CtrlMeas(0x00);

    /// Set the oversampling for temperature data.
    ///
    /// See [`Oversampling`] for settings, and chapter 3.4.3 in the [datasheet]
    /// for details.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::{CtrlMeas, Oversampling};
    ///
    /// let mut ctrl_meas: CtrlMeas = CtrlMeas::default();
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::default());
    /// ctrl_meas = ctrl_meas.set_osrs_t(Oversampling::Skip);
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::Skip);
    /// ctrl_meas = ctrl_meas.set_osrs_t(Oversampling::X1);
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::X1);
    /// ctrl_meas = ctrl_meas.set_osrs_t(Oversampling::X2);
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::X2);
    /// ctrl_meas = ctrl_meas.set_osrs_t(Oversampling::X4);
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::X4);
    /// ctrl_meas = ctrl_meas.set_osrs_t(Oversampling::X8);
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::X8);
    /// ctrl_meas = ctrl_meas.set_osrs_t(Oversampling::X16);
    /// assert_eq!(ctrl_meas.osrs_t(), Oversampling::X16);
    /// ```
    ///
    /// [datasheet]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    #[must_use = "set_osrs_t returns a modified CtrlMeas"]
    pub const fn set_osrs_t(self, os: Oversampling) -> CtrlMeas {
        CtrlMeas((self.0 & 0b00011111) | ((os as u8) << 5))
    }

    /// Get the temperature data oversampling.
    pub const fn osrs_t(&self) -> Oversampling {
        match (self.0 >> 5) & 0b111 {
            0b000 => Oversampling::Skip,
            0b001 => Oversampling::X1,
            0b010 => Oversampling::X2,
            0b011 => Oversampling::X4,
            0b100 => Oversampling::X8,
            _ => Oversampling::X16,
        }
    }

    /// Set the oversampling for pressure data.
    ///
    /// See [`Oversampling`] for settings, and chapter 3.4.2 in the [datasheet]
    /// for details.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::{CtrlMeas, Oversampling};
    ///
    /// let mut ctrl_meas: CtrlMeas = CtrlMeas::default();
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::default());
    /// ctrl_meas = ctrl_meas.set_osrs_p(Oversampling::Skip);
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::Skip);
    /// ctrl_meas = ctrl_meas.set_osrs_p(Oversampling::X1);
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::X1);
    /// ctrl_meas = ctrl_meas.set_osrs_p(Oversampling::X2);
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::X2);
    /// ctrl_meas = ctrl_meas.set_osrs_p(Oversampling::X4);
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::X4);
    /// ctrl_meas = ctrl_meas.set_osrs_p(Oversampling::X8);
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::X8);
    /// ctrl_meas = ctrl_meas.set_osrs_p(Oversampling::X16);
    /// assert_eq!(ctrl_meas.osrs_p(), Oversampling::X16);
    /// ```
    ///
    /// [datasheet]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    #[must_use = "set_osrs_p returns a modified CtrlMeas"]
    pub const fn set_osrs_p(self, os: Oversampling) -> CtrlMeas {
        CtrlMeas((self.0 & 0b11100011) | ((os as u8) << 2))
    }

    /// Get the pressure data oversampling.
    pub const fn osrs_p(&self) -> Oversampling {
        match (self.0 >> 2) & 0b111 {
            0b000 => Oversampling::Skip,
            0b001 => Oversampling::X1,
            0b010 => Oversampling::X2,
            0b011 => Oversampling::X4,
            0b100 => Oversampling::X8,
            _ => Oversampling::X16,
        }
    }

    /// Set the sensor mode for the device.
    ///
    /// See [`Mode`] for setting, and chapter 3.3 in the [datasheet] for details.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::{CtrlMeas, Mode};
    ///
    /// let mut ctrl_meas: CtrlMeas = CtrlMeas::default();
    /// assert_eq!(ctrl_meas.mode(), Mode::default());
    /// ctrl_meas = ctrl_meas.set_mode(Mode::Sleep);
    /// assert_eq!(ctrl_meas.mode(), Mode::Sleep);
    /// ctrl_meas = ctrl_meas.set_mode(Mode::Forced);
    /// assert_eq!(ctrl_meas.mode(), Mode::Forced);
    /// ctrl_meas = ctrl_meas.set_mode(Mode::Normal);
    /// assert_eq!(ctrl_meas.mode(), Mode::Normal);
    /// ```
    ///
    /// [datasheet]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
    #[must_use = "set_osrs_p returns a modified CtrlMeas"]
    pub const fn set_mode(self, m: Mode) -> CtrlMeas {
        CtrlMeas((self.0 & 0xFC) | (m as u8))
    }

    /// Get the mode.
    pub const fn mode(&self) -> Mode {
        match self.0 & 0b11 {
            0b00 => Mode::Sleep,
            0b11 => Mode::Normal,
            _ => Mode::Forced,
        }
    }
}

impl Default for CtrlMeas {
    fn default() -> Self {
        CtrlMeas::RESET
    }
}

/// Status register
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Status(u8);

impl Status {
    /// Get the reset value of the ctrl_meas register.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::Status;
    ///
    /// assert_eq!(Status::RESET, Status::default());
    /// ```
    pub const RESET: Self = Status(0x00);

    /// Measuring field.
    ///
    /// Automatically set to `true` whenever a conversion is running and back to
    /// `false` when the results have been transferred to the data registers.
    ///
    /// # Example
    ///
    /// ```
    /// assert!(!bme280_multibus::Status::RESET.measuring());
    /// ```
    pub const fn measuring(&self) -> bool {
        self.0 & (1 << 3) != 0
    }

    /// im_update field.
    ///
    /// Automatically set to `true` when the NVM data are being copied to image
    /// registers and back to `false` when the copying is done.
    /// The data is copied at power-on-reset and before every conversion.
    ///
    /// # Example
    ///
    /// ```
    /// assert!(!bme280_multibus::Status::RESET.im_update());
    /// ```
    pub const fn im_update(&self) -> bool {
        self.0 & 1 != 0
    }
}

impl core::fmt::Display for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Status")
            .field("measuring", &self.measuring())
            .field("im_update", &self.im_update())
            .finish()
    }
}

impl Default for Status {
    fn default() -> Self {
        Status::RESET
    }
}

/// BME280 initialization settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Settings {
    /// `config` register value.
    pub config: Config,
    /// `ctrl_meas` register value.
    pub ctrl_meas: CtrlMeas,
    /// `ctrl_hum` register value.
    pub ctrl_hum: Oversampling,
}

impl Settings {
    /// Create a new settings structure.
    ///
    /// # Example
    ///
    /// ```
    /// use bme280_multibus::{Config, CtrlMeas, Filter, Mode, Oversampling, Settings, Standby};
    ///
    /// const SETTINGS: Settings = Settings {
    ///     config: Config::RESET
    ///         .set_standby_time(Standby::Millis1000)
    ///         .set_filter(Filter::X16),
    ///     ctrl_meas: CtrlMeas::RESET
    ///         .set_osrs_t(Oversampling::X16)
    ///         .set_osrs_p(Oversampling::X16)
    ///         .set_mode(Mode::Normal),
    ///     ctrl_hum: Oversampling::X16,
    /// };
    /// ```
    pub const fn new() -> Settings {
        Settings {
            config: Config::RESET,
            ctrl_meas: CtrlMeas::RESET,
            ctrl_hum: Oversampling::RESET,
        }
    }
}

impl Default for Settings {
    fn default() -> Self {
        Settings::new()
    }
}

/// A sensor sample from the BME280.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sample {
    /// Temperature reading in celsius.
    pub temperature: f32,
    /// Pressure reading in pascal.
    pub pressure: f32,
    /// Humidity in perfect relative.
    pub humidity: f32,
}

impl Sample {
    fn from_raw<B>(buf: &[u8; NUM_MEAS_REG], cal: &Calibration) -> Result<Self, Error<B>> {
        // The magical math and magical numbers come from the datasheet.
        // I am not to blame for this.

        // msb [7:0] = p[19:12]
        // lsb [7:0] = p[11:4]
        // xlsb[7:4] = p[3:0]
        let p: u32 = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[2] as u32) >> 4);
        // msb [7:0] = t[19:12]
        // lsb [7:0] = t[11:4]
        // xlsb[7:4] = t[3:0]
        let t: u32 = ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | ((buf[5] as u32) >> 4);
        // msb [7:0] = h[15:8]
        // lsb [7:0] = h[7:0]
        let h: u32 = ((buf[6] as u32) << 8) | (buf[7] as u32);

        if t == 0x80000000 || p == 0x80000000 || h == 0x8000 {
            return Err(Error::Sample);
        }

        let p: i32 = p as i32;
        let t: i32 = t as i32;
        let h: i32 = h as i32;

        let var1: i32 = (((t >> 3) - ((cal.t1 as i32) << 1)) * (cal.t2 as i32)) >> 11;
        let var2: i32 = (((((t >> 4) - (cal.t1 as i32)) * ((t >> 4) - (cal.t1 as i32))) >> 12)
            * (cal.t3 as i32))
            >> 14;

        let t_fine: i32 = var1 + var2;

        let temperatue: i32 = (t_fine * 5 + 128) >> 8;
        let temperature: f32 = (temperatue as f32) / 100.0;

        let var1: i64 = (t_fine as i64) - 128000;
        let var2: i64 = var1 * var1 * (cal.p6 as i64);
        let var2: i64 = var2 + ((var1 * (cal.p5 as i64)) << 17);
        let var2: i64 = var2 + ((cal.p4 as i64) << 35);
        let var1: i64 = ((var1 * var1 * (cal.p3 as i64)) >> 8) + ((var1 * (cal.p2 as i64)) << 12);
        let var1: i64 = ((((1i64) << 47) + var1) * (cal.p1 as i64)) >> 33;
        let pressure: f32 = if var1 == 0 {
            0.0
        } else {
            let var3: i64 = 1048576 - (p as i64);
            let var3: i64 = (((var3 << 31) - var2) * 3125) / var1;
            let var1: i64 = ((cal.p9 as i64) * (var3 >> 13) * (var3 >> 13)) >> 25;
            let var2: i64 = ((cal.p8 as i64) * var3) >> 19;

            let var3: i64 = ((var3 + var1 + var2) >> 8) + ((cal.p7 as i64) << 4);
            (var3 as f32) / 256.0
        };

        let var1: i32 = t_fine - 76800i32;
        let var1: i32 =
            ((((h << 14) - ((cal.h4 as i32) << 20) - ((cal.h5 as i32) * var1)) + 16384i32) >> 15)
                * (((((((var1 * (cal.h6 as i32)) >> 10)
                    * (((var1 * (cal.h3 as i32)) >> 11) + (32768i32)))
                    >> 10)
                    + (2097152i32))
                    * (cal.h2 as i32)
                    + 8192)
                    >> 14);
        let var1: i32 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (cal.h1 as i32)) >> 4);
        let var1: i32 = if var1 < 0 { 0 } else { var1 };
        let var1: i32 = if var1 > 419430400 { 419430400 } else { var1 };
        let humidity: f32 = ((var1 >> 12) as f32) / 1024.0;

        Ok(Sample {
            temperature,
            pressure,
            humidity,
        })
    }
}

/// I2C device address.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Clone, Copy)]
#[repr(u8)]
pub enum Address {
    /// SDO pin is connected to GND.
    SdoGnd = 0x76,
    /// SDO pin is connected to V<sub>DDIO</sub>
    SdoVddio = 0x77,
}

/// Sampling error.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<B> {
    /// Bus error.
    Bus(B),
    /// Sample error.
    ///
    /// This is returned when the sample registers hold the reset value.
    /// This may indicate:
    /// * The power-on-time delay was not observed.
    /// * [`Oversampling::Skip`] was used for humidity, temperature, or pressure.
    /// * The BME280 was not configured.
    Sample,
}

/// BME280 bus.
pub trait Bme280Bus {
    /// BME280 bus error.
    type Error;

    /// Read from the BME280.
    ///
    /// # I2C
    ///
    /// ```text
    /// Read example (BME280 Datasheet Figure 10: I2C multiple byte read)
    /// +-------+---------------+----+------+------------------+------+
    /// | Start | Slave Address | RW | ACKS | Register Address | ACKS |
    /// +-------+---------------+----+------+------------------+------+
    /// | S     | 111011x       |  0 |      | xxxxxxxx         |      |
    /// +-------+---------------+----+------+------------------+------+
    ///
    ///     +-------+---------------+----+------+---------------+------+---------------+--------+------+
    /// ... | Start | Slave Address | RW | ACKS | Register Data | ACKM | Register Data | NOACKM | Stop |
    ///     +-------+---------------+----+------+---------------+------+---------------+--------+------+
    /// ... | S     | 111011x       |  1 |      | xxxxxxxx      |      | xxxxxxxx      |        | P    |
    ///     +-------+---------------+----+------+---------------+------+---------------+--------+------+
    /// ```
    ///
    /// # SPI
    ///
    /// ```text
    /// Read example (BME280 Datasheet Figure 13: SPI multiple byte read)
    /// +-------+----+------------------+---------------+
    /// | Start | RW | Register Address | Register Data |
    /// +-------+----+------------------+---------------+
    /// | CSB=0 |  1 | xxxxxxx          | xxxxxxxx      |
    /// +-------+----+------------------+---------------+
    ///
    ///     +---------------+-------+
    /// ... | Register Data | Stop  |
    ///     +---------------+-------+
    /// ... | xxxxxxxx      | CSB=0 |
    ///     +---------------+-------+
    /// ```
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write a single register to the BME280.
    ///
    /// # I2C
    ///
    /// ```text
    /// Write example (BME280 Datasheet Figure 9: I2C multiple byte write)
    /// +-------+---------------+----+------+------------------+------+---------------+------+
    /// | Start | Slave Address | RW | ACKS | Register Address | ACKS | Register Data | ACKS |
    /// +-------+---------------+----+------+------------------+------+---------------+------+
    /// | S     | 111011x       |  0 |      | xxxxxxxx         |      | xxxxxxxx      |      |
    /// +-------+---------------+----+------+------------------+------+---------------+------+
    ///
    ///     +------------------+------+---------------+------+------+
    /// ... | Register Address | ACKS | Register Data | ACKS | Stop |
    ///     +------------------+------+---------------+------+------+
    /// ... | xxxxxxxx         |      | xxxxxxxx      |      | P    |
    ///     +------------------+------+---------------+------+------+
    /// ```
    ///
    /// # SPI
    ///
    /// ```text
    /// Write example (BME280 Datasheet Figure 12: SPI multiple byte write)
    /// +-------+----+------------------+---------------+
    /// | Start | RW | Register Address | Register Data |
    /// +-------+----+------------------+---------------+
    /// | CSB=0 |  0 | xxxxxxx          | xxxxxxxx      |
    /// +-------+----+------------------+---------------+
    ///
    ///     +----+------------------+---------------+-------+
    /// ... | RW | Register Address | Register Data | Stop  |
    ///     +----+------------------+---------------+-------+
    /// ... |  0 | xxxxxxx          | xxxxxxxx      | CSB=0 |
    ///     +----+------------------+---------------+-------+
    /// ```
    fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error>;

    /// Read the calibration from the chip.
    fn calibration(&mut self) -> Result<Calibration, Self::Error> {
        const FIRST: usize = (reg::CALIB_25 - reg::CALIB_00 + 1) as usize;
        debug_assert_eq!(FIRST, 26);
        const SECOND: usize = (reg::CALIB_32 - reg::CALIB_26 + 1) as usize;
        debug_assert_eq!((FIRST + SECOND), NUM_CALIB_REG);

        let mut buf: [u8; NUM_CALIB_REG] = [0; NUM_CALIB_REG];
        self.read_regs(reg::CALIB_00, &mut buf[0..FIRST])?;
        self.read_regs(reg::CALIB_26, &mut buf[FIRST..(FIRST + SECOND)])?;

        Ok(buf.into())
    }
}

/// Asynchronous BME280 bus.
#[cfg(feature = "async")]
pub trait Bme280BusAsync {
    /// BME280 bus error.
    type Error;

    /// Read from the BME280.
    ///
    /// See [`Bme280Bus::read_regs`] for more information.
    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Write a single register to the BME280.
    ///
    /// See [`Bme280Bus::write_reg`] for more information.
    async fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), Self::Error>;

    /// Read the calibration from the chip.
    async fn calibration(&mut self) -> Result<crate::Calibration, Self::Error> {
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

/// BME280 driver.
#[derive(Debug)]
pub struct Bme280<B> {
    bus: B,
    cal: Calibration,
}

impl<I2C, E> Bme280<crate::i2c0::Bme280Bus<I2C>>
where
    I2C: eh0::blocking::i2c::Write<Error = E> + eh0::blocking::i2c::WriteRead<Error = E>,
{
    /// Creates a new `Bme280` driver from a I2C peripheral, and an I2C
    /// device address.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// # ]);
    /// use bme280_multibus::{i2c0::Address, Bme280};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn from_i2c0(i2c: I2C, address: crate::i2c0::Address) -> Result<Self, E> {
        let bus = crate::i2c0::Bme280Bus::new(i2c, address);
        Self::new(bus)
    }
}

impl<I2C, E> Bme280<crate::i2c1::Bme280Bus<I2C>>
where
    I2C: eh1::i2c::I2c<Error = E>,
{
    /// Creates a new `Bme280` driver from a I2C peripheral, and an I2C
    /// device address.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh1::i2c::Mock::new(&[
    /// #   ehm::eh1::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh1::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// # ]);
    /// use bme280_multibus::{i2c1::Address, Bme280};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c1(i2c, Address::SdoGnd)?;
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh1::MockError>(())
    /// ```
    pub fn from_i2c1(i2c: I2C, address: crate::i2c1::Address) -> Result<Self, E> {
        let bus = crate::i2c1::Bme280Bus::new(i2c, address);
        Self::new(bus)
    }
}

impl<SPI, CS, SpiError, PinError> Bme280<crate::spi0::Bme280Bus<SPI, CS>>
where
    SPI: eh0::blocking::spi::Transfer<u8, Error = SpiError>
        + eh0::blocking::spi::Write<u8, Error = SpiError>,
    CS: eh0::digital::v2::OutputPin<Error = PinError>,
{
    /// Creates a new `Bme280` driver from an embedded-hal version 0.2 SPI
    /// peripheral and a chip select digital I/O pin.
    ///
    /// # Safety
    ///
    /// The chip select pin must be high before being passed to this function.
    ///
    /// # Example
    ///
    /// ```
    /// # let spi = ehm::eh0::spi::Mock::new(&[
    /// #   ehm::eh0::spi::Transaction::write(vec![0x88]),
    /// #   ehm::eh0::spi::Transaction::transfer(vec![0; 26], vec![0; 26]),
    /// #   ehm::eh0::spi::Transaction::write(vec![0xE1]),
    /// #   ehm::eh0::spi::Transaction::transfer(vec![0; 7], vec![0; 7]),
    /// # ]);
    /// # let mut pin = ehm::eh0::pin::Mock::new(&[
    /// #    ehm::eh0::pin::Transaction::set(ehm::eh0::pin::State::High),
    /// #    ehm::eh0::pin::Transaction::set(ehm::eh0::pin::State::Low),
    /// #    ehm::eh0::pin::Transaction::set(ehm::eh0::pin::State::High),
    /// #    ehm::eh0::pin::Transaction::set(ehm::eh0::pin::State::Low),
    /// #    ehm::eh0::pin::Transaction::set(ehm::eh0::pin::State::High),
    /// # ]);
    /// use bme280_multibus::Bme280;
    /// use eh0::digital::v2::OutputPin;
    ///
    /// pin.set_high()?;
    /// let mut bme: Bme280<_> = Bme280::from_spi0(spi, pin)?;
    /// # let (mut spi, mut pin) = bme.free().free();
    /// # spi.done(); pin.done();
    /// # Ok::<(), bme280_multibus::spi0::Error<ehm::eh0::MockError, ehm::eh0::MockError>>(())
    /// ```
    #[allow(clippy::unnecessary_safety_doc)]
    pub fn from_spi0(spi: SPI, cs: CS) -> Result<Self, crate::spi0::Error<SpiError, PinError>> {
        let bus = crate::spi0::Bme280Bus::new(spi, cs);
        Self::new(bus)
    }
}

impl<SPI, E> Bme280<crate::spi1::Bme280Bus<SPI>>
where
    SPI: eh1::spi::SpiDevice<Error = E>,
{
    /// Creates a new `Bme280` driver from an embedded-hal version 1 SPI device.
    ///
    /// # Example
    ///
    /// ```
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::Bme280;
    ///
    /// let mut bme: Bme280<_> = Bme280::from_spi1(spi)?;
    /// # bme.free().free().done();
    /// # Ok::<(), eh1::spi::ErrorKind>(())
    /// ```
    pub fn from_spi1(spi: SPI) -> Result<Self, E> {
        let bus: crate::spi1::Bme280Bus<SPI> = crate::spi1::Bme280Bus::new(spi);
        Self::new(bus)
    }
}

#[cfg(feature = "async")]
impl<SPI, E> Bme280<crate::spi1::Bme280Bus<SPI>>
where
    SPI: eha0a::spi::SpiDevice<Error = E>,
{
    /// Creates a new `Bme280` driver from an embedded-hal-async SPI device.
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::Bme280;
    ///
    /// let bme: Bme280<_> = Bme280::from_spia0a(spi).await?;
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn from_spia0a(spi: SPI) -> Result<Self, E> {
        let bus = crate::spi1::Bme280Bus::new(spi);
        Self::new_async(bus).await
    }
}

#[cfg(feature = "async")]
impl<B, E> Bme280<B>
where
    B: Bme280BusAsync<Error = E>,
{
    /// Create a new BME280 from a [`Bme280BusAsync`].
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::{spi1::Bme280Bus, Bme280};
    ///
    /// let bus: Bme280Bus<_> = Bme280Bus::new(spi);
    /// let bme: Bme280<_> = Bme280::new_async(bus).await?;
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn new_async(mut bus: B) -> Result<Self, E> {
        let cal: Calibration = bus.calibration().await?;
        Ok(Self { bus, cal })
    }

    /// BME280 chip ID.
    ///
    /// The return value is a constant, [`CHIP_ID`].
    ///
    /// This register is useful as a sanity check to ensure communications are
    /// working with the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xD0),
    /// #   ehm::eh1::spi::Transaction::read(0x60),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::{spi1::Bme280Bus, Bme280, CHIP_ID};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_spia0a(spi).await?;
    /// let chip_id: u8 = bme.chip_id_async().await?;
    /// assert_eq!(chip_id, CHIP_ID);
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn chip_id_async(&mut self) -> Result<u8, E> {
        let mut buf: [u8; 1] = [0];
        self.bus.read_regs(reg::ID, &mut buf).await?;
        Ok(buf[0])
    }

    /// Reset the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xE0 & !0x80, 0xB6]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::{spi1::Bme280Bus, Bme280};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_spia0a(spi).await?;
    /// bme.reset_async().await?;
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn reset_async(&mut self) -> Result<(), E> {
        self.bus.write_reg(reg::RESET, RESET_MAGIC).await
    }

    /// Get the status of the device.
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xF3),
    /// #   ehm::eh1::spi::Transaction::read(0x00),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::{spi1::Bme280Bus, Bme280, Status};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_spia0a(spi).await?;
    /// let status: Status = bme.status_async().await?;
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn status_async(&mut self) -> Result<Status, E> {
        let mut buf: [u8; 1] = [0];
        self.bus.read_regs(reg::STATUS, &mut buf).await?;
        Ok(Status(buf[0]))
    }

    /// Configure the BME280 settings.
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xF2 & !0x80, 0b100]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xF4 & !0x80, 0b10010011]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xF5 & !0x80, 0b10110000]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::{
    ///     spi1::Bme280Bus, Bme280, Config, CtrlMeas, Filter, Mode, Oversampling, Settings, Standby,
    /// };
    ///
    /// const SETTINGS: Settings = Settings {
    ///     config: Config::RESET
    ///         .set_standby_time(Standby::Millis1000)
    ///         .set_filter(Filter::X16),
    ///     ctrl_meas: CtrlMeas::RESET
    ///         .set_osrs_t(Oversampling::X8)
    ///         .set_osrs_p(Oversampling::X8)
    ///         .set_mode(Mode::Normal),
    ///     ctrl_hum: Oversampling::X8,
    /// };
    ///
    /// let mut bme: Bme280<_> = Bme280::from_spia0a(spi).await?;
    /// bme.settings_async(&SETTINGS).await?;
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn settings_async(&mut self, settings: &Settings) -> Result<(), E> {
        self.bus
            .write_reg(reg::CTRL_HUM, settings.ctrl_hum as u8)
            .await?;
        self.bus
            .write_reg(reg::CTRL_MEAS, settings.ctrl_meas.0)
            .await?;
        self.bus.write_reg(reg::CONFIG, settings.config.0).await
    }

    /// Read a sample from the BME280.
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() -> Result<(), eh1::spi::ErrorKind> {
    /// # let spi = ehm::eh1::spi::Mock::new(&[
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0x88),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 26]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xE1),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 7]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xF2 & !0x80, 0b100]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xF4 & !0x80, 0b10010011]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write_vec(vec![0xF5 & !0x80, 0b10110000]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// #   ehm::eh1::spi::Transaction::transaction_start(),
    /// #   ehm::eh1::spi::Transaction::write(0xF7),
    /// #   ehm::eh1::spi::Transaction::read_vec(vec![0; 8]),
    /// #   ehm::eh1::spi::Transaction::transaction_end(),
    /// # ]);
    /// use bme280_multibus::{
    ///     spi1::Bme280Bus, Bme280, Config, CtrlMeas, Filter, Mode, Oversampling, Sample, Settings,
    ///     Standby,
    /// };
    ///
    /// const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
    ///     config: bme280_multibus::Config::RESET
    ///         .set_standby_time(bme280_multibus::Standby::Millis1000)
    ///         .set_filter(bme280_multibus::Filter::X16),
    ///     ctrl_meas: bme280_multibus::CtrlMeas::RESET
    ///         .set_osrs_t(bme280_multibus::Oversampling::X8)
    ///         .set_osrs_p(bme280_multibus::Oversampling::X8)
    ///         .set_mode(bme280_multibus::Mode::Normal),
    ///     ctrl_hum: bme280_multibus::Oversampling::X8,
    /// };
    ///
    /// let mut bme: Bme280<_> = Bme280::from_spia0a(spi).await?;
    /// bme.settings_async(&SETTINGS).await?;
    /// let sample: Sample = bme.sample_async().await.unwrap();
    /// # bme.free().free().done(); Ok(()) }
    /// ```
    pub async fn sample_async(&mut self) -> Result<Sample, Error<E>> {
        let mut buf: [u8; NUM_MEAS_REG] = [0; NUM_MEAS_REG];
        self.bus
            .read_regs(reg::PRESS_MSB, &mut buf)
            .await
            .map_err(Error::Bus)?;

        Sample::from_raw(&buf, &self.cal)
    }
}

impl<B, E> Bme280<B>
where
    B: Bme280Bus<Error = E>,
{
    /// Create a new BME280 from a [`spi0::Bme280Bus`] or
    /// a [`i2c0::Bme280Bus`].
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// # ]);
    /// use bme280_multibus::{
    ///     i2c0::{Address, Bme280Bus},
    ///     Bme280,
    /// };
    ///
    /// let bus: Bme280Bus<_> = Bme280Bus::new(i2c, Address::SdoGnd);
    /// let bme: Bme280<_> = Bme280::new(bus)?;
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn new(mut bus: B) -> Result<Self, E> {
        let cal: Calibration = bus.calibration()?;
        Ok(Self { bus, cal })
    }

    /// Free the I2C bus from the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// # ]);
    /// use bme280_multibus::{
    ///     i2c0::{Address, Bme280Bus},
    ///     Bme280,
    /// };
    ///
    /// let bus: Bme280Bus<_> = Bme280Bus::new(i2c, Address::SdoGnd);
    /// let bme: Bme280<_> = Bme280::new(bus)?;
    /// let bus: Bme280Bus<_> = bme.free();
    /// # bus.free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    #[inline]
    pub fn free(self) -> B {
        self.bus
    }

    /// BME280 chip ID.
    ///
    /// The return value is a constant, [`CHIP_ID`].
    ///
    /// This register is useful as a sanity check to ensure communications are
    /// working with the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xD0], vec![0x60]),
    /// # ]);
    /// use bme280_multibus::{i2c0::Address, Bme280, CHIP_ID};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
    /// let chip_id: u8 = bme.chip_id()?;
    /// assert_eq!(chip_id, CHIP_ID);
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn chip_id(&mut self) -> Result<u8, E> {
        let mut buf: [u8; 1] = [0];
        self.bus.read_regs(reg::ID, &mut buf)?;
        Ok(buf[0])
    }

    /// Reset the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xE0, 0xB6]),
    /// # ]);
    /// use bme280_multibus::{i2c0::Address, Bme280};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
    /// bme.reset()?;
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn reset(&mut self) -> Result<(), E> {
        self.bus.write_reg(reg::RESET, RESET_MAGIC)
    }

    /// Get the status of the device.
    ///
    /// # Example
    ///
    /// Check if a conversion is running.
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xF3], vec![0]),
    /// # ]);
    /// use bme280_multibus::{i2c0::Address, Bme280, Status};
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
    /// let status: Status = bme.status()?;
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn status(&mut self) -> Result<Status, E> {
        let mut buf: [u8; 1] = [0];
        self.bus.read_regs(reg::STATUS, &mut buf)?;
        Ok(Status(buf[0]))
    }

    /// Configure the BME280 settings.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF2, 0b100]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF4, 0b10010011]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF5, 0b10110000]),
    /// # ]);
    /// use bme280_multibus::{
    ///     i2c0::Address, Bme280, Config, CtrlMeas, Filter, Mode, Oversampling, Settings, Standby,
    /// };
    ///
    /// const SETTINGS: Settings = Settings {
    ///     config: Config::RESET
    ///         .set_standby_time(Standby::Millis1000)
    ///         .set_filter(Filter::X16),
    ///     ctrl_meas: CtrlMeas::RESET
    ///         .set_osrs_t(Oversampling::X8)
    ///         .set_osrs_p(Oversampling::X8)
    ///         .set_mode(Mode::Normal),
    ///     ctrl_hum: Oversampling::X8,
    /// };
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
    /// bme.settings(&SETTINGS)?;
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn settings(&mut self, settings: &Settings) -> Result<(), E> {
        self.bus.write_reg(reg::CTRL_HUM, settings.ctrl_hum as u8)?;
        self.bus.write_reg(reg::CTRL_MEAS, settings.ctrl_meas.0)?;
        self.bus.write_reg(reg::CONFIG, settings.config.0)
    }

    /// Read a sample from the BME280.
    ///
    /// # Example
    ///
    /// ```
    /// # let i2c = ehm::eh0::i2c::Mock::new(&[
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF2, 0b100]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF4, 0b10010011]),
    /// #   ehm::eh0::i2c::Transaction::write(0x76, vec![0xF5, 0b10110000]),
    /// #   ehm::eh0::i2c::Transaction::write_read(0x76, vec![0xF7], vec![0; 8]),
    /// # ]);
    /// use bme280_multibus::{i2c0::Address, Bme280, Sample, Standby};
    ///
    /// const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
    ///     config: bme280_multibus::Config::RESET
    ///         .set_standby_time(bme280_multibus::Standby::Millis1000)
    ///         .set_filter(bme280_multibus::Filter::X16),
    ///     ctrl_meas: bme280_multibus::CtrlMeas::RESET
    ///         .set_osrs_t(bme280_multibus::Oversampling::X8)
    ///         .set_osrs_p(bme280_multibus::Oversampling::X8)
    ///         .set_mode(bme280_multibus::Mode::Normal),
    ///     ctrl_hum: bme280_multibus::Oversampling::X8,
    /// };
    ///
    /// let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
    /// bme.settings(&SETTINGS)?;
    /// let sample: Sample = bme.sample().unwrap();
    /// # bme.free().free().done();
    /// # Ok::<(), ehm::eh0::MockError>(())
    /// ```
    pub fn sample(&mut self) -> Result<Sample, Error<E>> {
        let mut buf: [u8; NUM_MEAS_REG] = [0; NUM_MEAS_REG];
        self.bus
            .read_regs(reg::PRESS_MSB, &mut buf)
            .map_err(Error::Bus)?;

        Sample::from_raw(&buf, &self.cal)
    }
}
