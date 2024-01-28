*NOTE: I get many messages expressing interest in this system.  I plan on offering an Arduino programmable filght computer with open source code by mid-2024.
Until then, there are 5 hardware options to use this code.  Scroll to the bottom to view the 5 options.  My website is not yet running, but you can contact me at 
https://www.rocketryforum.com/members/sparkyvt.810/

HPR Rocket Flight Computer
Original sketch by SparkyVT
NAR #85720, L3
TRA #12111, L3

--------DESCRIPTION----------
This flight computer is designed for rockets 38mm in diameter or greater, and will fit inside a 38mm tube coupler.
Dimensions are 4.0in x 1.25in x 0.5in, antenna configuration may add more length
It has similar capability as a TeleMega, but without a smart-phone app or advanced interfaces  
Flight-tested on multiple M-and-N powered supersonic flights to over 34K feet and Mach 2.3.  
For large or high-power projects, a commercially avialble backup computer is strongly recommended.  

--------FEATURES----------
Full-featured dual deploy/multi-stage/airstart rocket flight computer capable to 100,000ft or more
Tilt-sensing lockout for ignition of second stages and/or airstarts
Live telemetry over 433MHz or 915MHz LoRa (433MHz: USA amateur 70cm band, EUR licencse free) (915MHz: USA licence free) 
4 high-current pyro outputs with continuity checks
Advanced MEMS sensor package: GNSS, accelerometers, gyroscope, magnetometer, barometer, and LoRa radio
High Data-Capture rate: approximately 50,000 samples per second recorded to SD card
--1000Hz 3-axis digital 24G and 100G accelerometer data logging
--1000Hz 3-axis digital 2000dps gyroscope data logging
--1000Hz of flight events & continuity data logging
--1000Hz of sensor-fuzed speed & altitude
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
--Optional Audible Battery Voltage report at startup
--Optional Magnetic Switch Startup & Shut-down
--Preflight audible reporting options: Perfectflight or Marsa
--User selectable telemetry frequency & power settings
--8 configurable servo outputs (4 powered, 4 un-powered)
--User selectable inflight brownout recovery
Mach immune, sensor-fusion based apogee event
Barometric based main deploy event
Audible pre-flight continuity report
Audible Post-flight max altitude & speed report
Mount in any orientation, automatic orientation detection with built-in self calibration mode
Bench-test mode activated w/ tactile button; user configurable status messages over USB Serial
Report in SI or Metric units
Compatible with Teensy 3.2, 3.5, 3.6, 4.0, 4.1
--Connect any sensor to any available I2C or SPI bus
--Create your own custom setup with configurable pins for continuity, firing, and servos
--Connect UBLOX GPS unit to any available HW Serial port

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 4.1, 3.5, or 3.6 (compatible with provided PCB file) or Teensy 4.0/3.2 (no PCB file provided)
               IMU: LSM6DS0X (recommended), LSM9DS1, LSM6DS33, MPU6050, or LSM303 & L3GD20H combo
      Magnetometer: LIS3MDL (recommended), LSM9DS1, or LSM303
100G Accelerometer: H3LIS331DL (recommended), ADXL377, or ADXL377 & ADS1115 combo
   Pressure Sensor: MS5611 (recommended), MS5607, MPL3115A2, BMP180, BMP280, or BMP388 (Note: BMP280 & BMP388 incompatible with telemetry due to EMI)
              GNSS: UBLOX M6, M7, M8, M9 (NEO footprint supported in PCB file) or Adafruit Ultimate GPS
    LoRa Telemetry: RFM96W or RFM95W, both capable of TX on 433MHz or 915MHz (433MHz USA: Amateur License Required, EUR: License Free)

--------GROUND STATION COMPONENTS----------
               MCU: Arduino Nano 33 BLE
         Telemetry: RFM96W 433MHz (USA: Ham License Required, EUR: License Free) or RFM95W 915MHz (USA: License Free)
               GPS: Ublox SAM-M8Q (optional)
               IMU: Sparkfun LSM9DS1 breakout
               LCD: 20X4 Sparkfun LCD
           SD Card: Adafruit SD Card breakout board
           Antenna: 433Mhz 5-element Yagi (Ham Only) or 900Mhz (License Free)

--------ADDITIONAL LIBRARIES REQUIRED----------
TinyGPS++
SDfat and/or SD

----------------HARDWARE OPTIONS----------------
OPTION 1: Assemble the unit yourself.  Purchase a Teensy 4.0 or 4.1 and integrate the system with commercially availble sensor breakout boards.  If you want to use high-current pyro outputs, then it is up to you
to develop the continuity and firing circuitry.  I have threaded standoffs and screws that are a perfect fit for the Adafruit breakout boards.

OPTION 2: Purchase a custom Teensy breakout board and add your own sensors.  The breakout includes a Teensy 4.1, integrates 4x high-current pyro outputs, and 6 easy-to-use terminal blocks to integrate into your high-power rocket.  Includes Qwiic connector to easily integrate your custom sensor package over I2C.


OPTION 3: Purchase a fully-functional Frankenstein-board.  This integrates all sensors breakout boards onto a single main board that is 4.0 x 1.25 x 0.5 inches.  Comes fully assembled, pre-programmed, calibrated, and tested.  Configurations will vary, but quality will not.  


OPTION 4: Try your hand at assembling the Frankenstein-board yourself.  This is a single bare-board with no breakouts attached.  You will need to reflow the GNSS chip and hand-solder approximately 30 components.  Instructions included.

OPTION 5: Purchase a fully assembled and tested ground station.  Includes ground unit with 2 radios and 20x4 LCD mounted in a custom form-fitting box.  

