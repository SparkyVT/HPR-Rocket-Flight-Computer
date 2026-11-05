use crate::types::Raw3;
use crate::register::{Error, RegisterController};

// chip addresses
const WHO_AM_I_ADDR: u8 = 0x0F;
const CTRL1_XL_ADDR: u8 = 0x10;
const CTRL2_G_ADDR: u8 = 0x11;
const CTRL3_C_ADDR: u8 = 0x12;
const OUTX_L_G: u8 = 0x22;

const CTRL1_XL_VAL: u8 = 0b10000100;
const CTRL2_G_VAL: u8 = 0b10001100;
const CTRL3_C_VAL: u8 = 0x44;

const CONFIG: &[(u8, u8)] = &[
    (CTRL3_C_ADDR, CTRL3_C_VAL),
    (CTRL2_G_ADDR, CTRL2_G_VAL),
    (CTRL1_XL_ADDR, CTRL1_XL_VAL),
];

// who am i expected value
const WHO_AM_I_VAL: u8 = 0x6C;

// chip constants
const ACCEL_SCALE: f32 = 0.000488;      // g per count
const ACCEL_SATURATION: i16 = 32112;    // 98% of full scale
const GYRO_SCALE: f32 = 0.07;           // °/s per count
const GYRO_SATURATION: i16 = 32112;     // 98% of full scale
const SAMPLE_PERIOD_US: u32 = 602;      // 1660 Hz

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sample {
    gyro: Raw3,
    accel: Raw3,
}

impl Sample {
    pub fn new(gyro: Raw3, accel: Raw3) -> Self {
        Self {
            gyro,
            accel,
        }
    }
    pub fn gyro(&self) -> Raw3 { self.gyro }
    pub fn accel(&self) -> Raw3 { self.accel }
}

pub struct Lsm6dsoxDriver<R> {
    reg_ctrl: R,
}

impl<R: RegisterController> Lsm6dsoxDriver<R> {
    pub fn new(mut reg_ctrl: R) -> Result<Self, Error<R::Error>> {
        reg_ctrl.check_id(WHO_AM_I_ADDR, WHO_AM_I_VAL)?;

        reg_ctrl.configure(CONFIG)?; 

        Ok(Self { reg_ctrl })
    }

    pub fn read(&mut self) -> Result<Sample, Error<R::Error>> {
        let bytes = self.reg_ctrl.read_bytes::<12>(OUTX_L_G)?;
        Ok(parse(&bytes))
    }

    pub fn accel_scale(&self) -> f32 { ACCEL_SCALE }

    pub fn accel_saturation(&self) -> i16 { ACCEL_SATURATION }

    pub fn gyro_scale(&self) -> f32 { GYRO_SCALE }

    pub fn gyro_saturation(&self) -> i16 { GYRO_SATURATION }

    pub fn sample_period_us(&self) -> u32 { SAMPLE_PERIOD_US }
}

fn parse(bytes: &[u8; 12]) -> Sample {
    let word = |i: usize| i16::from_le_bytes([bytes[i], bytes[i+1]]);

    Sample::new(Raw3::new(word(0), word(2), word(4)), Raw3::new(word(6), word(8), word(10)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::I2cBus;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    const ADDR: u8 = 0x6A;

    /// The full startup conversation, in the order `new` does it.
    fn init_expectations() -> Vec<Transaction> {
        vec![
            Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![WHO_AM_I_VAL]),
            Transaction::write(ADDR, vec![CTRL3_C_ADDR, CTRL3_C_VAL]),
            Transaction::write_read(ADDR, vec![CTRL3_C_ADDR], vec![CTRL3_C_VAL]),
            Transaction::write(ADDR, vec![CTRL2_G_ADDR, CTRL2_G_VAL]),
            Transaction::write_read(ADDR, vec![CTRL2_G_ADDR], vec![CTRL2_G_VAL]),
            Transaction::write(ADDR, vec![CTRL1_XL_ADDR, CTRL1_XL_VAL]),
            Transaction::write_read(ADDR, vec![CTRL1_XL_ADDR], vec![CTRL1_XL_VAL]),
        ]
    }

    #[test]
    fn new_succeeds_with_correct_chip() {
        let mut mock = Mock::new(&init_expectations());

        let result = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(result.is_ok());
        mock.done();
    }

    #[test]
    fn new_fails_on_wrong_chip() {
        let expectations = [Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![0x69])];
        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(matches!(result, Err(Error::WrongId { expected: WHO_AM_I_VAL, found: 0x69 })));
        mock.done();
    }

    #[test]
    fn new_fails_when_first_setting_does_not_stick() {
        let expectations = [
            Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![WHO_AM_I_VAL]),
            Transaction::write(ADDR, vec![CTRL3_C_ADDR, CTRL3_C_VAL]),
            Transaction::write_read(ADDR, vec![CTRL3_C_ADDR], vec![0x00]), // chip ignored the write
        ];
        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(matches!(
            result,
            Err(Error::UnverifiedWrite { reg: CTRL3_C_ADDR, expected: CTRL3_C_VAL, found: 0x00 })
        ));
        mock.done();
    }

    #[test]
    fn new_fails_when_last_setting_does_not_stick() {
        let mut expectations = init_expectations();
        expectations.pop(); // drop the good CTRL1_XL read-back...
        expectations.push(Transaction::write_read(ADDR, vec![CTRL1_XL_ADDR], vec![0x00])); // ...use a bad one

        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(matches!(
            result,
            Err(Error::UnverifiedWrite { reg: CTRL1_XL_ADDR, expected: CTRL1_XL_VAL, found: 0x00 })
        ));
        mock.done();
    }

    #[test]
    fn new_reports_bus_error() {
        let expectations = [Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![0x00])
            .with_error(ErrorKind::Other)];
        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(matches!(result, Err(Error::Bus(ErrorKind::Other))));
        mock.done();
    }

    #[test]
    fn parse_puts_bytes_in_right_places() {
        //          gyro x   gyro y   gyro z   accel x  accel y  accel z
        let buf = [1, 0,    2, 0,    3, 0,    4, 0,    5, 0,    0xFF, 0xFF];

        let sample = parse(&buf);

        assert_eq!(sample.gyro(), Raw3::new(1, 2, 3));
        assert_eq!(sample.accel(), Raw3::new(4, 5, -1));
    }

    #[test]
    fn parse_handles_full_range() {
        let buf = [0xFF, 0x7F,  0x00, 0x80,  0x01, 0x08,  0, 0,  0, 0,  0, 0];

        let sample = parse(&buf);

        assert_eq!(sample.gyro(), Raw3::new(32767, -32768, 2049));
    }

    #[test]
    fn read_returns_parsed_sample() {
        let mut expectations = init_expectations();
        expectations.push(Transaction::write_read(
            ADDR,
            vec![OUTX_L_G],
            vec![1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0],
        ));
        let mut mock = Mock::new(&expectations);

        let mut driver = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR)).unwrap();
        let sample = driver.read().unwrap();

        assert_eq!(sample.gyro(), Raw3::new(1, 2, 3));
        assert_eq!(sample.accel(), Raw3::new(4, 5, 6));
        mock.done();
    }

    #[test]
    fn read_reports_bus_error() {
        let mut expectations = init_expectations();
        expectations.push(
            Transaction::write_read(ADDR, vec![OUTX_L_G], vec![0; 12]).with_error(ErrorKind::Other),
        );
        let mut mock = Mock::new(&expectations);

        let mut driver = Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), ADDR)).unwrap();

        assert!(matches!(driver.read(), Err(Error::Bus(ErrorKind::Other))));
        mock.done();
    }
}