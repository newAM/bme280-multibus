[![crates.io](https://img.shields.io/crates/v/bme280-multibus.svg)](https://crates.io/crates/bme280-multibus)
[![docs.rs](https://docs.rs/bme280-multibus/badge.svg)](https://docs.rs/bme280-multibus/)
[![CI](https://github.com/newAM/bme280-multibus/workflows/CI/badge.svg)](https://github.com/newAM/bme280-multibus/actions)

# bme280-multibus

BME280 driver with support for I2C and SPI buses.

## Example

```rust
use bme280_multibus::{i2c0::Address, Bme280, Sample, Standby};

const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
    config: bme280_multibus::Config::RESET
        .set_standby_time(bme280_multibus::Standby::Millis1000)
        .set_filter(bme280_multibus::Filter::X16),
    ctrl_meas: bme280_multibus::CtrlMeas::RESET
        .set_osrs_t(bme280_multibus::Oversampling::X8)
        .set_osrs_p(bme280_multibus::Oversampling::X8)
        .set_mode(bme280_multibus::Mode::Normal),
    ctrl_hum: bme280_multibus::Oversampling::X8,
};

let mut bme: Bme280<_> = Bme280::from_i2c0(i2c, Address::SdoGnd)?;
bme.settings(&SETTINGS)?;
let sample: Sample = bme.sample().unwrap();
```

## Features

* `async` Enable asynchronous implementations with `embedded-hal-async`.
  Requires a nightly toolchain.
* `serde` Add `Serialize` and `Deserialize` to `Sample`.
