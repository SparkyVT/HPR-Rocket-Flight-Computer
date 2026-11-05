use crate::bus::DeviceBus;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    WrongId { expected: u8, found: u8 },
    UnverifiedWrite { reg: u8, expected: u8, found: u8 },
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self { Error::Bus(e) }
}

pub trait RegisterController: DeviceBus {
    fn check_id(&mut self, reg: u8, expected: u8) -> Result<(), Error<Self::Error>> {
        let found = self.read_byte(reg)?;
        if found != expected {
            return Err(Error::WrongId { expected, found });
        }
        Ok(())
    }

    fn write_verified(&mut self, reg: u8, val: u8) -> Result<(), Error<Self::Error>> {
        let found = self.write_then_read_reg(reg, val)?;
        if found != val {
            return Err(Error::UnverifiedWrite { reg, expected: val, found });
        }
        Ok(())
    }

    fn configure(&mut self, settings: &[(u8, u8)]) -> Result<(), Error<Self::Error>> {
        for &(reg, val) in settings {
            self.write_verified(reg, val)?;
        }
        Ok(())
    }
}

impl<B: DeviceBus> RegisterController for B {}