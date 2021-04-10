//! This example reads the chip ID from a Bosch BME280.
//!
//! The hardware for this example can be purchased from adafruit:
//!
//! * https://www.adafruit.com/product/2264
//! * https://www.adafruit.com/product/2652
//! * https://www.adafruit.com/product/4399
//! * https://www.adafruit.com/product/4472

use bme280::{Bme280, CHIP_ID};
use ftd2xx_embedded_hal as hal;

const SETTINGS: bme280::Settings = bme280::Settings {
    config: bme280::Config::reset()
        .set_standby_time(bme280::Standby::Millis125)
        .set_filter(bme280::Filter::X8),
    ctrl_meas: bme280::CtrlMeas::reset()
        .set_osrs_t(bme280::Oversampling::X8)
        .set_osrs_p(bme280::Oversampling::X8)
        .set_mode(bme280::Mode::Normal),
    ctrl_hum: bme280::Oversampling::X8,
};

fn main() {
    let ftdi = hal::Ft232hHal::new()
        .expect("Failed to open FT232H device")
        .init_default()
        .expect("Failed to initialize MPSSE");
    let i2c: hal::I2c<_> = ftdi.i2c().expect("Failed to initialize I2C");

    let mut bme: Bme280<hal::I2c<_>, bme280::Uninit> = Bme280::new(i2c, bme280::Address::SdoVddio)
        .reset()
        .expect("Failed to reset");

    std::thread::sleep(std::time::Duration::from_millis(2));

    // sanity check
    assert_eq!(bme.chip_id().expect("Failed to read chip ID"), CHIP_ID);
    println!("Chip ID ok");

    let mut bme: Bme280<hal::I2c<_>, bme280::Init> =
        bme.init(&SETTINGS).expect("Failed to initialize BME280");

    std::thread::sleep(std::time::Duration::from_millis(250));

    let sample: bme280::Sample = bme.sample().expect("Failed to sample BME280");
    println!("sample = {:#?}", sample);
}
