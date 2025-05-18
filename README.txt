*NOTE: I get many messages expressing interest in this system.  I plan on offering an Arduino programmable filght computer with open source code by late-2025. You can contact me at https://www.rocketryforum.com/members/sparkyvt.810/

HPR Rocket Flight Computer
Original sketch by SparkyVT
NAR #85720, L3
TRA #12111, L3

--------DESCRIPTION----------
This code base is the software to run an advanced flight computer in high-power amateur rockets. Hardware setups vary, 
but typically can fit within a 38mm airframe. It has similar capability as a TeleMega, but without a smart-phone app 
or advanced interfaces. Flight-tested on over 250 successful flights, including M-and-N powered supersonic flights to
over 34K feet and Mach 2.3. For large or high-power projects, a commercially avialble backup computer is strongly 
recommended.  

--------FEATURES----------
Full-featured dual deploy/multi-stage/airstart rocket flight computer capable to 100,000ft or more
Tilt-sensing safety lockout for ignition of second stages and/or airstarts
Live telemetry over 433MHz or 915MHz LoRa (433MHz: USA amateur 70cm band, EUR licencse free) (915MHz: USA licence free) 
4 programmable high-current pyro outputs with continuity checks
Advanced MEMS sensor package: GNSS, accelerometers, gyroscope, magnetometer, barometer, and LoRa radio
High Data-Capture rate: approximately 50,000 samples per second recorded to SD card
--1600Hz 3-axis digital 24G and 100G accelerometer data logging
--1600Hz 3-axis digital 2000dps gyroscope data logging
--1600Hz of flight events & continuity data logging
--1600Hz of sensor-fuzed speed & altitude
--100Hz of pitch, yaw, roll rotation
--40Hz of of magnetic data logging and magnetic roll
--30Hz-100Hz of digital barometric data logging (Altitude, pressure, temperature)
--30Hz of main battery voltage (1400Hz during pyro events)
--20Hz of LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GNSS position, signal strength)
--5Hz-25Hz of GNSS data logging (chip-dependent data rates & constellations)
--Separate data file for each flight up to 100 flights
--Optional separate GNSS NMEA capture file for plotting in Google Earth, UBLOX NEO-M9N configurtion approved for TRA altitude record attempts
Simple, easy-to-use configuration interface through the SD card
--User Selectable Flight Mode: Single-Stage, Two-Stage, Airstart, or Booster
--Configurable Apogee delay
--Optional Magnetic Switch Startup & Shut-down
--Preflight audible reporting options: Perfectflight or Marsa
--User selectable telemetry frequency & power settings
--8 configurable servo outputs
--User selectable inflight brownout recovery
Mach immune, sensor-fusion based apogee event
Barometric based main deploy event
Audible pre-flight continuity report
Audible Post-flight max altitude & speed report
Mount in any orientation, automatic orientation detection with built-in self calibration mode
Bench-test mode activated w/ tactile button to simulate flight and test charges; user configurable status messages over USB Serial
Report in SI or Metric units
Compatible with Teensy 3.2, 3.5, 3.6, 4.0, 4.1
--Plug-and-play sensor package; connect any sensor to any available I2C or SPI bus, connect UBLOX GPS unit to any available HW Serial port
--Create your own custom hardware setup with configurable pins for continuity, firing, and servos

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 4.1 (recommended), 4.0, 3.5, or 3.6
               IMU: LSM6DSOX (recommended), LSM9DS1, LSM6DS33, MPU6050, or LSM303 & L3GD20H combo, LSM6DS3TR
      Magnetometer: LIS3MDL (recommended), LSM9DS1, or LSM303
100G Accelerometer: H3LIS331DL (recommended), ADXL377, or ADXL377 & ADS1115 combo
   Pressure Sensor: MS5611 (recommended), MS5607, MPL3115A2, BMP180, BMP280, or BMP388 (Note: BMP280 & BMP388 incompatible with telemetry due to EMI)
              GNSS: UBLOX M6, M7, M8, M9, M10 or Adafruit Ultimate GPS
    LoRa Telemetry: RFM96W or RFM95W, both capable of TX on 433MHz or 915MHz (433MHz USA: Amateur License Required, EUR: License Free)

--------GROUND STATION COMPONENTS----------
               MCU: Arduino Nano 33 BLE w/ IMU
         Telemetry: RFM96W 433MHz (USA: Ham License Required, EUR: License Free) or RFM95W 915MHz (USA: License Free)
               GPS: Ublox SAM-M8Q (optional)
               LCD: 20X4 Sparkfun LCD
           SD Card: Adafruit SD Card breakout board
           Antenna: 433Mhz 5-element Yagi (Ham Only) or 900Mhz (License Free)

--------ADDITIONAL LIBRARIES REQUIRED----------
TinyGPS++
SDfat and/or SD

