# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Added
- Added SPI implementations for embedded-hal version 1.0.0-alpha.11.
- Added SPI implementations for embedded-hal-async version 0.2.0-alpha.2.

### Changed
- Renamed the `i2c` module to `i2c0`.
- Renamed the `spi` module to `spi0`.
- Renamed `Bme280::from_spi` to `Bme280::from_spi0`.
- Renamed `Bme280::from_i2c` to `Bme280::from_i2c0`.
- Moved `bme280_multibus::spi::MAX_FREQ` to `bme280_multibus::SPI_MAX_FREQ`.
- Changed `pub const fn reset` to a `RESET` constant for:
  - `Mode`
  - `Standby`
  - `Filter`
  - `Config`
  - `CtrlMeas`
  - `Status`

## [0.2.2] - 2022-08-13
### Fixed
- Fixed sign extension of the h4 and h5 calibration value, thanks @mbuesch.

## [0.2.1] - 2022-01-16
### Added
- Added a `"serde"` feature to enable `Serialize` and `Deserialize` on `Sample`.

## [0.2.0] - 2022-01-02
### Added
- Added an error type for sample errors.

### Changed
- Updated the edition from 2018 to 2021.
- Changed `sample` to return an error if the registers contain the reset value,
  previously this function panicked in debug builds if the sample registers
  contained the reset value. 

## [0.1.0] - 2022-01-01
- Initial release

[Unreleased]: https://github.com/newAM/bme280-multibus/compare/v0.2.2...HEAD
[0.2.2]: https://github.com/newAM/bme280-multibus/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/newAM/bme280-multibus/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/newAM/bme280-multibus/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/newAM/bme280-multibus/releases/tag/v0.1.0
