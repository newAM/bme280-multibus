![Maintenance](https://img.shields.io/badge/maintenance-experimental-blue.svg)
[![CI](https://github.com/newAM/bme280-rs/workflows/CI/badge.svg)](https://github.com/newAM/bme280-rs/actions)

# bme280

Another incomplete BME280 crate.

The BME280 presents some design challenges, it is stateful, and it has
multiple bus options (3-wire SPI, 4-wire SPI, I2C).

This is just another BME280 driver implementation that I made for my own
usecases.

## Example

```rust
use bme280::{
    Address, Bme280, Config, CtrlMeas, Filter, Init, Mode, Oversampling, Sample, Settings,
    Standby, Uninit,
};

const SETTINGS: Settings = Settings {
    config: Config::reset()
        .set_standby_time(Standby::Millis1000)
        .set_filter(Filter::X16),
    ctrl_meas: CtrlMeas::reset()
        .set_osrs_t(Oversampling::X8)
        .set_osrs_p(Oversampling::X8)
        .set_mode(Mode::Normal),
    ctrl_hum: Oversampling::X8,
};

let mut bme: Bme280<_, Uninit> = Bme280::new(i2c, Address::SdoGnd);
let mut bme: Bme280<_, Init> = bme.init(&SETTINGS)?;
let sample: Sample = bme.sample()?;
```
