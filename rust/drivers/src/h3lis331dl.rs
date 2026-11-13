use crate::types::Raw3;
use crate::register::{Error, RegisterController};

const WHO_AM_I_ADDR: u8 = 0x0F;
const WHO_AM_I_VAL: u8 = 0x32;
const CTRL1_ADDR: u8 = 0x20;
const CTRL4_ADDR: u8 = 0x23;
const OUT_X_L: u8 = 0x28;
const CTRL1_VAL: u8 = 0b00111111;
const CTRL4_VAL: u8 = 0x00;

const CONFIG: &[(u8, u8)] = &[
    (CTRL4_ADDR, CTRL4_VAL),
    (CTRL1_ADDR, CTRL1_VAL),
];

pub const ACCEL_SCALE: f32 = 0.049;
pub const ACCEL_SATURATION: i16 = 2007;
pub const SAMPLE_PERIOD_US: u32 = 1000;

pub struct H3lis331dlDriver<R> {
    reg_ctrl: R,
}

impl<R: RegisterController> H3lis331dlDriver<R> {
    pub fn new(mut reg_ctrl: R) -> Result<Self, Error<R::Error>> {
        reg_ctrl.check_id(WHO_AM_I_ADDR, WHO_AM_I_VAL)?;

        reg_ctrl.configure(CONFIG)?;

        Ok(Self { reg_ctrl })
    }

    pub fn read(&mut self) -> Result<Raw3, Error<R::Error>> {
        let bytes = self.reg_ctrl.read_bytes::<6>(OUT_X_L)?;
        Ok(parse(&bytes))
    }

    pub fn accel_scale(&self) -> f32 { ACCEL_SCALE }

    pub fn accel_saturation(&self) -> i16 { ACCEL_SATURATION }

    pub fn sample_period_us(&self) -> u32 { SAMPLE_PERIOD_US }
}

fn parse(bytes: &[u8; 6]) -> Raw3 {
    let word = |i: usize| i16::from_le_bytes([bytes[i], bytes[i+1]]) >> 4;

    Raw3::new(word(0), word(2), word(4))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{SpiBus, SpiMasks};
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    const MASKS: SpiMasks = SpiMasks { read: 0x80, write: 0x00, inc: 0x40 };

    /// One SPI read: send the register (with masks), get `data` back.
    fn spi_read(cmd: u8, data: Vec<u8>) -> Vec<Transaction<u8>> {
        vec![
            Transaction::transaction_start(),
            Transaction::write_vec(vec![cmd]),
            Transaction::read_vec(data),
            Transaction::transaction_end(),
        ]
    }

    /// One SPI register write.
    fn spi_write(reg: u8, val: u8) -> Vec<Transaction<u8>> {
        vec![
            Transaction::transaction_start(),
            Transaction::write_vec(vec![reg, val]),
            Transaction::transaction_end(),
        ]
    }

    /// The full startup conversation, in the order `new` does it.
    fn init_expectations() -> Vec<Transaction<u8>> {
        let mut tx = spi_read(WHO_AM_I_ADDR | 0x80, vec![WHO_AM_I_VAL]);
        for &(reg, val) in CONFIG {
            tx.extend(spi_write(reg, val));
            tx.extend(spi_read(reg | 0x80, vec![val]));
        }
        tx
    }

    #[test]
    fn new_succeeds() {
        let mut mock = Mock::new(&init_expectations());

        let result = H3lis331dlDriver::new(SpiBus::new(mock.clone(), MASKS));

        assert!(result.is_ok());
        mock.done();
    }

    #[test]
    fn new_fails_on_wrong_chip() {
        let mut mock = Mock::new(&spi_read(WHO_AM_I_ADDR | 0x80, vec![0x00]));

        let result = H3lis331dlDriver::new(SpiBus::new(mock.clone(), MASKS));

        assert!(matches!(result, Err(Error::WrongId { expected: WHO_AM_I_VAL, found: 0x00 })));
        mock.done();
    }

    #[test]
    fn parse_shifts_and_keeps_sign() {
        //          x = 304      y = 0       z = -16
        let bytes = [0x30, 0x01, 0x00, 0x00, 0xF0, 0xFF];

        assert_eq!(parse(&bytes), Raw3::new(19, 0, -1));
    }

    #[test]
    fn read_sets_inc_bit_and_parses() {
        let mut expectations = init_expectations();
        expectations.extend(spi_read(OUT_X_L | 0x80 | 0x40, vec![0x30, 0x01, 0x00, 0x00, 0xF0, 0xFF]));
        let mut mock = Mock::new(&expectations);

        let mut driver = H3lis331dlDriver::new(SpiBus::new(mock.clone(), MASKS)).unwrap();

        assert_eq!(driver.read().unwrap(), Raw3::new(19, 0, -1));
        mock.done();
    }
}