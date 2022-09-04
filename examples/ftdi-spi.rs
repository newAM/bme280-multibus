//! This example reads the chip ID from a Bosch BME280.
//!
//! The hardware for this example can be purchased from adafruit:
//!
//! * https://www.adafruit.com/product/2264
//! * https://www.adafruit.com/product/2652
//! * https://www.adafruit.com/product/4472
//!
//! Make the following connections:
//!
//! * Connect SCK to D0
//! * Connect MOSI to D1
//! * Connect MISO to D2
//! * Connect CS to D3
//! * Connect Vdd to 3.3V or 5V
//! * Connect Vss to GND

use bme280_multibus::{Bme280, CHIP_ID};
use ftdi_embedded_hal::{
    libftd2xx::{self, Ft232h},
    FtHal, OutputPin, Spi,
};

const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
    config: bme280_multibus::Config::reset()
        .set_standby_time(bme280_multibus::Standby::Millis125)
        .set_filter(bme280_multibus::Filter::X8),
    ctrl_meas: bme280_multibus::CtrlMeas::reset()
        .set_osrs_t(bme280_multibus::Oversampling::X8)
        .set_osrs_p(bme280_multibus::Oversampling::X8)
        .set_mode(bme280_multibus::Mode::Normal),
    ctrl_hum: bme280_multibus::Oversampling::X8,
};

fn main() {
    let device: Ft232h = libftd2xx::Ftdi::new().unwrap().try_into().unwrap();
    let hal_dev: FtHal<Ft232h> = FtHal::init_default(device).unwrap();

    let spi: Spi<Ft232h> = hal_dev.spi().unwrap();
    let cs: OutputPin<Ft232h> = hal_dev.ad3().unwrap();

    let mut bme: Bme280<_> = Bme280::from_spi0(spi, cs).expect("Failed to initialize BME280");
    bme.reset().expect("Failed to reset");

    std::thread::sleep(std::time::Duration::from_millis(2));

    // sanity check
    assert_eq!(bme.chip_id().expect("Failed to read chip ID"), CHIP_ID);
    println!("Chip ID ok");

    bme.settings(&SETTINGS)
        .expect("Failed to initialize BME280");

    std::thread::sleep(std::time::Duration::from_millis(250));

    let sample: bme280_multibus::Sample = bme.sample().expect("Failed to sample BME280");
    println!("sample = {:#?}", sample);
}
