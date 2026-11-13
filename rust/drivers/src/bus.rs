use embedded_hal::i2c::I2c;
use embedded_hal::spi::{Operation, SpiDevice};

pub trait DeviceBus {
    type Error;

    fn read_bytes<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Self::Error>;

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::Error>;

    fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error>;

    fn read_byte(&mut self, reg: u8) -> Result<u8, Self::Error> {
        self.read_bytes::<1>(reg).map(|bytes| bytes[0])
    }

    fn write_then_read_reg(&mut self, reg: u8, val: u8) -> Result<u8, Self::Error> {
        self.write_reg(reg, val)?;
        self.read_byte(reg)
    }
}

pub struct I2cBus<I> {
    i2c: I,
    addr: u8,
    inc: u8,
}

impl<I: I2c> I2cBus<I> {
    pub fn new(i2c: I, addr: u8) -> Self {
        Self { i2c, addr, inc: 0 }
    }

    pub fn with_inc(mut self, inc: u8) -> Self {
        self.inc = inc;
        self
    }

    pub fn release(self) -> I {
        self.i2c
    }
}

impl<I: I2c> DeviceBus for I2cBus<I> {
    type Error = I::Error;

    fn read_bytes<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Self::Error> {

        let mut buf = [0u8; N];

        if N == 0 {
            return Ok(buf);            
        }

        let reg = if N > 1 { reg | self.inc } else { reg };
        
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;

        Ok(buf)
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.addr, &[reg, val])
    }

    fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.addr, &[cmd])
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SpiMasks {
    pub read: u8,
    pub write: u8,
    pub inc: u8,
}

pub struct SpiBus<S> {
    spi: S,
    masks: SpiMasks,
}

impl<S: SpiDevice> SpiBus<S> {
    pub fn new(spi: S, masks: SpiMasks) -> Self {
        Self { spi, masks }
    }

    pub fn release(self) -> S {
        self.spi
    }
}

impl<S: SpiDevice> DeviceBus for SpiBus<S> {
    type Error = S::Error;

    fn read_bytes<const N: usize>(&mut self, reg: u8) -> Result<[u8; N], Self::Error> {
        let mut buf = [0u8; N];
        if N == 0 {
            return Ok(buf);
        }

        let inc = if N > 1 { self.masks.inc } else { 0 };
        let cmd = [reg | self.masks.read | inc];
        self.spi.transaction(&mut [
            Operation::Write(&cmd),
            Operation::Read(&mut buf)
        ])?;

        Ok(buf)
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Self::Error> {
        self.spi.write(&[reg | self.masks.write, val])
    }

    fn write_cmd(&mut self, cmd: u8) -> Result<(), Self::Error> {
        self.spi.write(&[cmd])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTx};

    const ADDR: u8 = 0x6A;

    #[test]
    fn i2c_read_and_write() {
        let expected = [
            I2cTx::write_read(ADDR, vec![0x0F], vec![0x6C]),
            I2cTx::write(ADDR, vec![0x10, 0x84]),
        ];
        let mut bus = I2cBus::new(I2cMock::new(&expected), ADDR);

        assert_eq!(bus.read_byte(0x0F).unwrap(), 0x6C);
        bus.write_reg(0x10, 0x84).unwrap();
        bus.release().done();
    }

    #[test]
    fn i2c_inc_only_on_multi_byte_reads() {
        let expected = [
            I2cTx::write_read(ADDR, vec![0x28 | 0x80], vec![1, 2, 3, 4, 5, 6]),
            I2cTx::write_read(ADDR, vec![0x0F], vec![0x32]),
        ];
        let mut bus = I2cBus::new(I2cMock::new(&expected), ADDR).with_inc(0x80);

        let data: [u8; 6] = bus.read_bytes(0x28).unwrap();
        assert_eq!(data, [1, 2, 3, 4, 5, 6]);
        assert_eq!(bus.read_byte(0x0F).unwrap(), 0x32);
        bus.release().done();
    }

    #[test]
    fn spi_read_sets_read_and_inc_bits() {
        let expected = [
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x28 | 0x80 | 0x40]),
            SpiTx::read_vec(vec![1, 2, 3, 4, 5, 6]),
            SpiTx::transaction_end(),
        ];
        let masks = SpiMasks { read: 0x80, write: 0x00, inc: 0x40 };
        let mut bus = SpiBus::new(SpiMock::new(&expected), masks);

        let data: [u8; 6] = bus.read_bytes(0x28).unwrap();
        assert_eq!(data, [1, 2, 3, 4, 5, 6]);
        bus.release().done();
    }

    #[test]
    fn spi_write_sets_write_bit() {
        let expected = [
            SpiTx::transaction_start(),
            SpiTx::write_vec(vec![0x01 | 0x80, 0x81]),
            SpiTx::transaction_end(),
        ];
        let masks = SpiMasks { read: 0x00, write: 0x80, inc: 0x00 };
        let mut bus = SpiBus::new(SpiMock::new(&expected), masks);

        bus.write_reg(0x01, 0x81).unwrap();
        bus.release().done();
    }
}