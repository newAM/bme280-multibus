use bme280_multibus::{i2c0::Address, Bme280, Sample};

#[test]
fn i2c1_sample() {
    let i2c = ehm::eh1::i2c::Mock::new(&[
        ehm::eh1::i2c::Transaction::write_read(0x76, vec![0x88], vec![0; 26]),
        ehm::eh1::i2c::Transaction::write_read(0x76, vec![0xE1], vec![0; 7]),
        ehm::eh1::i2c::Transaction::write(0x76, vec![0xF2, 0b100]),
        ehm::eh1::i2c::Transaction::write(0x76, vec![0xF4, 0b10010011]),
        ehm::eh1::i2c::Transaction::write(0x76, vec![0xF5, 0b10110000]),
        ehm::eh1::i2c::Transaction::write_read(0x76, vec![0xF7], vec![0; 8]),
    ]);

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

    let mut bme: Bme280<_> = Bme280::from_i2c1(i2c, Address::SdoGnd).unwrap();
    bme.settings(&SETTINGS).unwrap();
    let _sample: Sample = bme.sample().unwrap();
    bme.free().free().done();
}
