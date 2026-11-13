use crate::bus::DeviceBus;
use embedded_hal::delay::DelayNs;

const RESET_CMD: u8 = 0x1E; // reset command for new
const CALIBRATION_START: u8 = 0xA2; // first address of calibration numbers to read on start
const CALIBRATION_LEN: usize = 6; // number of calibration numbers to read

const TEMP_START_CMD: u8 = 0x58; // command to begin temperature measurement
const PRESSURE_START_CMD: u8 = 0x48; // command to begin new pressure measurement
const READING_ADDR: u8 = 0x00; // measurement read address

pub const SAMPLE_PERIOD_US: u32 = 9090; // main loop period between samples
pub const READING_WAIT_US: u32 = 9040; // wait time to read measurement after requesting it
pub const SAMPLES_BETWEEN_TEMP: u8 = 10; // number of pressure readings between each temp reading

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    FailedCalibration { addr: u8 }, // only possible on start if calibration numbers are junk (0x0000 or 0xFFFF)
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self { Error::Bus(e) }
}

pub struct Ms5611Driver<B> {
    bus: B,
    calibration: [u16; CALIBRATION_LEN],
    last_temp_reading: TempReading, // need to remember temp reading between samples to convert pressure readings
    measuring: Measuring,
    pressure_count: u8,
}

enum Measuring {
    Temperature,
    Pressure,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct TempReading {
    temp: i32,
    dt: i64,
}

impl<B: DeviceBus> Ms5611Driver<B> {
    pub fn new(mut bus: B, delay: &mut impl DelayNs) -> Result<Self, Error<B::Error>> {
        // send reset and wait
        bus.write_cmd(RESET_CMD)?;

        delay.delay_ms(100);

        let mut calibration = [0u16; CALIBRATION_LEN];

        // populate calibration with calibration numbers and make sure they arent junk
        for i in 0..CALIBRATION_LEN {
            let addr = CALIBRATION_START + 2*i as u8;
            let bytes = bus.read_bytes::<2>(addr)?;
            calibration[i] = u16::from_be_bytes(bytes);
            // 0x0000 or 0xFFFF means something went wrong
            if calibration[i] == 0x0000 || calibration[i] == 0xFFFF {
                return Err(Error::FailedCalibration { addr })
            }
        }

        // start first temperature measurement
        bus.write_cmd(TEMP_START_CMD)?;

        delay.delay_us(READING_WAIT_US);

        // manually convert first temp reading, no self yet
        let bytes = bus.read_bytes::<3>(READING_ADDR)?;
        let initial_temp = convert_temp(&calibration, u32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]));

        // start first pressure reading
        bus.write_cmd(PRESSURE_START_CMD)?;

        Ok(Self { bus, 
                calibration, 
                last_temp_reading: initial_temp, 
                measuring: Measuring::Pressure, 
                pressure_count: 0 })
    }

    pub fn poll(&mut self) -> Result<Option<i32>, Error<B::Error>> {
        let raw_reading = self.read_and_convert()?;

        match self.measuring {
            // if measuring temp: convert and save reading and reset counter, start pressure
            Measuring::Temperature => {
                self.last_temp_reading = convert_temp(&self.calibration, raw_reading);
                self.pressure_count = 0;
                self.start_reading(Measuring::Pressure)?;
                Ok(None)
            }

            // if measuring pressure: convert reading, increment, start next reading
            Measuring::Pressure => {
                let pressure_reading = convert_pressure(&self.calibration, raw_reading, self.last_temp_reading);
                self.pressure_count += 1;
                let next_measurement = if self.pressure_count >= SAMPLES_BETWEEN_TEMP {
                    Measuring::Temperature
                } else {
                    Measuring::Pressure
                };

                self.start_reading(next_measurement)?;

                Ok(Some(pressure_reading))
            }
        }
    }

    pub fn sample_period_us(&self) -> u32 { SAMPLE_PERIOD_US }

    // helper to ensure measurement state always matches command we send
    fn start_reading(&mut self, measuring: Measuring) -> Result<(), Error<B::Error>> {
        let cmd = match measuring {
            Measuring::Temperature => TEMP_START_CMD,
            Measuring::Pressure => PRESSURE_START_CMD,
        };

        self.bus.write_cmd(cmd)?;
        self.measuring = measuring;

        Ok(())
    }

    // helper to turn 3 byte reading into u32
    fn read_and_convert(&mut self) -> Result<u32, Error<B::Error>> {
        let bytes = self.bus.read_bytes::<3>(READING_ADDR)?;
        Ok(u32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]))
    }
}

// honestly not sure what either of the two converts do, some sort of wizard math based on factory calibration settings to get the right temp/pressure from a raw reading. Implementation from original driver
fn convert_temp(calibration: &[u16; CALIBRATION_LEN], raw_temp: u32) -> TempReading {
    let dt = raw_temp as i64 - ((calibration[4] as i64) << 8);
    let mut temp = 2000 + ((dt * calibration[5] as i64) >> 23);

    if temp < 2000 {
        temp -= (dt*dt) >> 31;
    }

    TempReading { temp: temp as i32, dt }
}

fn convert_pressure(calibration: &[u16; CALIBRATION_LEN], raw_pressure: u32, last_temp_reading: TempReading) -> i32 {

    let last_temp = last_temp_reading.temp as i64;
    let last_dt = last_temp_reading.dt;

    let mut off = ((calibration[1] as i64) << 16) + ((calibration[3] as i64 * last_dt) >> 7);
    let mut sens = ((calibration[0] as i64) << 15) + ((calibration[2] as i64 * last_dt) >> 8);

    if last_temp < 2000 {
        let t = last_temp - 2000;
        let mut off2 = 5 * t * t / 2;
        let mut sens2 = 5 * t * t / 4;

        if last_temp < -1500 {
            let t = last_temp + 1500;
            off2 += 7 * t * t;
            sens2 += 11 * t * t / 2;
        }

        off -= off2;
        sens -= sens2;
    }

    ((((raw_pressure as i64 * sens) >> 21) - off) >> 15) as i32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::I2cBus;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    const ADDR: u8 = 0x77;

    // datasheet example values
    const CAL: [u16; 6] = [40127, 36924, 23317, 23282, 33464, 28312];
    const D1: u32 = 9085466; // raw pressure
    const D2: u32 = 8569150; // raw temperature

    /// 3 bytes the chip would send for a raw reading
    fn adc_bytes(raw: u32) -> Vec<u8> {
        raw.to_be_bytes()[1..].to_vec()
    }

    /// Everything `new` says to the chip, in order.
    fn init_expectations() -> Vec<Transaction> {
        let mut tx = vec![Transaction::write(ADDR, vec![RESET_CMD])];
        for (i, c) in CAL.iter().enumerate() {
            let addr = CALIBRATION_START + 2 * i as u8;
            tx.push(Transaction::write_read(ADDR, vec![addr], c.to_be_bytes().to_vec()));
        }
        tx.push(Transaction::write(ADDR, vec![TEMP_START_CMD]));
        tx.push(Transaction::write_read(ADDR, vec![READING_ADDR], adc_bytes(D2)));
        tx.push(Transaction::write(ADDR, vec![PRESSURE_START_CMD]));
        tx
    }

    #[test]
    fn convert_temp_matches_datasheet() {
        assert_eq!(convert_temp(&CAL, D2), TempReading { temp: 2007, dt: 2366 });
    }

    #[test]
    fn convert_pressure_matches_datasheet() {
        let temp = convert_temp(&CAL, D2);
        assert_eq!(convert_pressure(&CAL, D1, temp), 100009);
    }

    #[test]
    fn new_succeeds() {
        let mut mock = Mock::new(&init_expectations());

        let result = Ms5611Driver::new(I2cBus::new(mock.clone(), ADDR), &mut NoopDelay::new());

        assert!(result.is_ok());
        mock.done();
    }

    #[test]
    fn new_rejects_bad_calibration() {
        let expectations = [
            Transaction::write(ADDR, vec![RESET_CMD]),
            Transaction::write_read(ADDR, vec![0xA2], vec![0x9C, 0xBF]),
            Transaction::write_read(ADDR, vec![0xA4], vec![0xFF, 0xFF]), // C2 is garbage
        ];
        let mut mock = Mock::new(&expectations);

        let result = Ms5611Driver::new(I2cBus::new(mock.clone(), ADDR), &mut NoopDelay::new());

        assert!(matches!(result, Err(Error::FailedCalibration { addr: 0xA4 })));
        mock.done();
    }

    #[test]
    fn new_reports_bus_error() {
        let expectations = [Transaction::write(ADDR, vec![RESET_CMD]).with_error(ErrorKind::Other)];
        let mut mock = Mock::new(&expectations);

        let result = Ms5611Driver::new(I2cBus::new(mock.clone(), ADDR), &mut NoopDelay::new());

        assert!(matches!(result, Err(Error::Bus(ErrorKind::Other))));
        mock.done();
    }

    #[test]
    fn poll_returns_pressure() {
        let mut expectations = init_expectations();
        expectations.push(Transaction::write_read(ADDR, vec![READING_ADDR], adc_bytes(D1)));
        expectations.push(Transaction::write(ADDR, vec![PRESSURE_START_CMD]));
        let mut mock = Mock::new(&expectations);

        let mut driver = Ms5611Driver::new(I2cBus::new(mock.clone(), ADDR), &mut NoopDelay::new()).unwrap();

        assert_eq!(driver.poll().unwrap(), Some(100009));
        mock.done();
    }

    #[test]
    fn poll_reads_temp_after_ten_pressures() {
        let mut expectations = init_expectations();
        // 9 pressures, each followed by another pressure start
        for _ in 0..9 {
            expectations.push(Transaction::write_read(ADDR, vec![READING_ADDR], adc_bytes(D1)));
            expectations.push(Transaction::write(ADDR, vec![PRESSURE_START_CMD]));
        }
        // 10th pressure, then a temperature start
        expectations.push(Transaction::write_read(ADDR, vec![READING_ADDR], adc_bytes(D1)));
        expectations.push(Transaction::write(ADDR, vec![TEMP_START_CMD]));
        // temperature result, then back to pressure
        expectations.push(Transaction::write_read(ADDR, vec![READING_ADDR], adc_bytes(D2)));
        expectations.push(Transaction::write(ADDR, vec![PRESSURE_START_CMD]));
        let mut mock = Mock::new(&expectations);

        let mut driver = Ms5611Driver::new(I2cBus::new(mock.clone(), ADDR), &mut NoopDelay::new()).unwrap();

        for _ in 0..10 {
            assert_eq!(driver.poll().unwrap(), Some(100009));
        }
        assert_eq!(driver.poll().unwrap(), None); // the temperature reading
        mock.done();
    }
}