# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

[Unreleased]: https://github.com/newAM/bme280-multibus/compare/v0.2.1...HEAD
[0.2.1]: https://github.com/newAM/bme280-multibus/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/newAM/bme280-multibus/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/newAM/bme280-multibus/releases/tag/v0.1.0
