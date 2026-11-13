use crate::types::Raw3;
use crate::register::{Error, RegisterController};

const WHO_AM_I_ADDR: u8 = 0x0F;
const WHO_AM_I_VAL: u8 = 0x3D;
const CTRL1_ADDR: u8 = 0x20;
const CTRL2_ADDR: u8 = 0x21;
const CTRL3_ADDR: u8 = 0x22;
const CTRL4_ADDR: u8 = 0x23;
const OUT_X_L: u8 = 0x28;
const CTRL1_VAL: u8 = 0b01111000;
const CTRL2_VAL: u8 = 0x00;
const CTRL3_VAL: u8 = 0x00;
const CTRL4_VAL: u8 = 0b00001100;

const CONFIG: &[(u8, u8)] = &[
    (CTRL1_ADDR, CTRL1_VAL),
    (CTRL2_ADDR, CTRL2_VAL),
    (CTRL4_ADDR, CTRL4_VAL),
    (CTRL3_ADDR, CTRL3_VAL), 
];

pub const MAG_SCALE: f32 = 1.0 / 6842.0;  
pub const MAG_SATURATION: i16 = 32112;   
pub const SAMPLE_PERIOD_US: u32 = 25000; 

pub struct Lis3mdlDriver<R> {
    reg_ctrl: R,
}

impl<R: RegisterController> Lis3mdlDriver<R> {
    pub fn new(mut reg_ctrl: R) -> Result<Self, Error<R::Error>> {
        reg_ctrl.check_id(WHO_AM_I_ADDR, WHO_AM_I_VAL)?;

        reg_ctrl.configure(CONFIG)?;

        Ok(Self { reg_ctrl })
    }

    pub fn read(&mut self) -> Result<Raw3, Error<R::Error>> {
        let bytes = self.reg_ctrl.read_bytes::<6>(OUT_X_L)?;
        Ok(parse(&bytes))
    }

    pub fn mag_scale(&self) -> f32 { MAG_SCALE }

    pub fn mag_saturation(&self) -> i16 { MAG_SATURATION }

    pub fn sample_period_us(&self) -> u32 { SAMPLE_PERIOD_US }
}

fn parse(bytes: &[u8; 6]) -> Raw3 {
    let word = |i: usize| i16::from_le_bytes([bytes[i], bytes[i+1]]);

    Raw3::new(word(0), word(2), word(4))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::I2cBus;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    const ADDR: u8 = 0x1C;

    fn init_expectations() -> Vec<Transaction> {
        let mut tx = vec![Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![WHO_AM_I_VAL])];
        for &(reg, val) in CONFIG {
            tx.push(Transaction::write(ADDR, vec![reg, val]));
            tx.push(Transaction::write_read(ADDR, vec![reg], vec![val]));
        }
        tx
    }

    #[test]
    fn new_succeeds() {
        let mut mock = Mock::new(&init_expectations());

        let result = Lis3mdlDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(result.is_ok());
        mock.done();
    }

    #[test]
    fn new_fails_on_wrong_chip() {
        let mut mock = Mock::new(&[Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![0x00])]);

        let result = Lis3mdlDriver::new(I2cBus::new(mock.clone(), ADDR));

        assert!(matches!(result, Err(Error::WrongId { expected: WHO_AM_I_VAL, found: 0x00 })));
        mock.done();
    }

    #[test]
    fn parse_handles_full_range() {
        //          x = max      y = min      z = -1
        let bytes = [0xFF, 0x7F, 0x00, 0x80, 0xFF, 0xFF];

        assert_eq!(parse(&bytes), Raw3::new(32767, -32768, -1));
    }

    #[test]
    fn read_returns_parsed_sample() {
        let mut expectations = init_expectations();
        expectations.push(Transaction::write_read(ADDR, vec![OUT_X_L], vec![1, 0, 2, 0, 3, 0]));
        let mut mock = Mock::new(&expectations);

        let mut driver = Lis3mdlDriver::new(I2cBus::new(mock.clone(), ADDR)).unwrap();

        assert_eq!(driver.read().unwrap(), Raw3::new(1, 2, 3));
        mock.done();
    }

    #[test]
    fn read_reports_bus_error() {
        let mut expectations = init_expectations();
        expectations.push(Transaction::write_read(ADDR, vec![OUT_X_L], vec![0; 6]).with_error(ErrorKind::Other));
        let mut mock = Mock::new(&expectations);

        let mut driver = Lis3mdlDriver::new(I2cBus::new(mock.clone(), ADDR)).unwrap();

        assert!(matches!(driver.read(), Err(Error::Bus(ErrorKind::Other))));
        mock.done();
    }
}