[![CI](https://github.com/newAM/bme280-rs/workflows/CI/badge.svg)](https://github.com/newAM/bme280-rs/actions)

# bme280

Another incomplete BME280 crate.

The BME280 presents some design challenges, it is stateful, and it has
multiple bus options (3-wire SPI, 4-wire SPI, I2C).

This is just another BME280 driver implementation that I made for my own
usecases (I2C or SPI + continuous sampling).

## Example

```rust
use bme280::{i2c::Address, Bme280, Sample, Standby};

const SETTINGS: bme280::Settings = bme280::Settings {
    config: bme280::Config::reset()
        .set_standby_time(bme280::Standby::Millis1000)
        .set_filter(bme280::Filter::X16),
    ctrl_meas: bme280::CtrlMeas::reset()
        .set_osrs_t(bme280::Oversampling::X8)
        .set_osrs_p(bme280::Oversampling::X8)
        .set_mode(bme280::Mode::Normal),
    ctrl_hum: bme280::Oversampling::X8,
};

let mut bme: Bme280<_> = Bme280::from_i2c(i2c, Address::SdoGnd)?;
bme.settings(&SETTINGS)?;
let sample: Sample = bme.sample()?;
```
