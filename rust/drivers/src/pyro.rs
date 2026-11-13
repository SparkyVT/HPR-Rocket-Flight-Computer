use embedded_hal::digital::{InputPin, OutputPin};

#[derive(Debug)]
pub enum Error<CE, FE> {
    Continuity(CE), // failure reading continuity pin
    Fire(FE),       // failure setting fire pin
}

pub struct PyroChannel<C, F> {
    continuity: C,
    fire: F,
}

/*
    This driver only sets fire pin/reads continuity pin. 
    On new, we set fire pin low to ensure it doesnt fire, but timing, other safety, etc is not covered here.
 */
impl<C: InputPin, F: OutputPin> PyroChannel<C, F> {
    pub fn new(continuity: C, mut fire: F) -> Result<Self, Error<C::Error, F::Error>> {
        fire.set_low().map_err(Error::Fire)?; // make sure it doesn't fire
        Ok(Self { continuity, fire })
    }

    pub fn set_firing(&mut self, on: bool) -> Result<(), Error<C::Error, F::Error>> {
        if on {
            self.fire.set_high().map_err(Error::Fire)
        } else {
            self.fire.set_low().map_err(Error::Fire)
        }
    }

    pub fn has_continuity(&mut self) -> Result<bool, Error<C::Error, F::Error>> {
        self.continuity.is_high().map_err(Error::Continuity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::MockError;
    use embedded_hal_mock::eh1::digital::{Mock, State, Transaction};
    use std::io::ErrorKind;

    /// channel whose `new` already set fire LOW, plus handles to check the mocks afterwards
    fn channel(cont_tx: &[Transaction], fire_tx: &[Transaction]) -> (PyroChannel<Mock, Mock>, Mock, Mock) {
        let mut fire_expect = vec![Transaction::set(State::Low)]; // from new
        fire_expect.extend_from_slice(fire_tx);
        let cont = Mock::new(cont_tx);
        let fire = Mock::new(&fire_expect);
        let pyro = PyroChannel::new(cont.clone(), fire.clone()).unwrap();
        (pyro, cont, fire)
    }

    #[test]
    fn new_sets_fire_low() {
        let (_pyro, mut cont, mut fire) = channel(&[], &[]);
        cont.done();
        fire.done();
    }

    #[test]
    fn set_firing_drives_pin() {
        let (mut pyro, mut cont, mut fire) =
            channel(&[], &[Transaction::set(State::High), Transaction::set(State::Low)]);
        pyro.set_firing(true).unwrap();
        pyro.set_firing(false).unwrap();
        cont.done();
        fire.done();
    }

    #[test]
    fn has_continuity_reads_pin() {
        let (mut pyro, mut cont, mut fire) =
            channel(&[Transaction::get(State::High), Transaction::get(State::Low)], &[]);
        assert!(pyro.has_continuity().unwrap());
        assert!(!pyro.has_continuity().unwrap());
        cont.done();
        fire.done();
    }

    #[test]
    fn new_reports_fire_error() {
        let mut cont = Mock::new(&[]);
        let mut fire = Mock::new(&[Transaction::set(State::Low).with_error(MockError::Io(ErrorKind::Other))]);
        let result = PyroChannel::new(cont.clone(), fire.clone());
        assert!(matches!(result, Err(Error::Fire(MockError::Io(_)))));
        cont.done();
        fire.done();
    }

    #[test]
    fn has_continuity_reports_error() {
        let (mut pyro, mut cont, mut fire) =
            channel(&[Transaction::get(State::High).with_error(MockError::Io(ErrorKind::Other))], &[]);
        assert!(matches!(pyro.has_continuity(), Err(Error::Continuity(MockError::Io(_)))));
        cont.done();
        fire.done();
    }
}