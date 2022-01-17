*NOTE: If you are planning to use this code or board files, please send me a PM on Rocketry Forum 
to ensure I have posted the latest updates.  https://www.rocketryforum.com/members/sparkyvtflyer.810/  

HPR Rocket Flight Computer
Original sketch by SparkyVT
NAR #85720, L3
TRA #12111, L3

--------DESCRIPTION----------
This flight computer is designed for rockets 38mm in diameter or greater, and will fit inside a 38mm tube coupler.
Dimensions are 4.0in x 1.25in x 0.5in, not including antenna
It has similar capability as a TeleMega, but without a smart-phone app or advanced interfaces  
It has flown on multiple M-powered supersonic flights to over 24K feet and Mach 2.0.  
For large or high-power projects, an independent backup computer such as the PerfectFlite StratoLogger, MissileWorks RRC2, or EggTimer Quark is strongly recommended.  

--------FEATURES----------
High data logging rate: 52,000 data points per second, 3-5 million data points per flight is typical
1400Hz 3-axis digital 24G and 100G accelerometer data logging
1400Hz 3-axis digital 2000dps gyroscope data logging
1400Hz of flight events across 13 distict parameters
1400Hz of integrated speed and altitude
1400Hz of pyro status (continuity detection, closed/open circuit, firing status, and pin debugging)
100Hz of pitch, yaw, roll rotation calculations using quaternion differentials
40Hz of magnetic field data logging
30Hz - 100Hz of digital barometric data logging (Altitude, pressure, temperature)
30Hz of main battery voltage (1400Hz during firing events)
20Hz of LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GNSS position/alt)
10Hz - 25Hz of GNSS location data on GPS, GLONASS, Galileo and Baidou systems (UBLOX chip dependent)
All data written to an easy to read CSV text file
4 programmable high-current pyro outputs with continuity detection and open circuit reporting
4 programmable "plug 'n play" servo connections, can be actuated at flight events (active stabilization in development)
4 additional programmable un-powered servo outputs (requires JST connector)
User Selectable Flight Mode: Single-Stage, Two-Stage Sustainer, Two-Stage Booser, Airstart
"Smart Config" will default to single-stage mode if complex pyros not detected
Mach immune flight event detection algorithms
Sensor Fusion based apogee event
Barometric based main deploy event
False-liftoff detection prevents accidental pyro discharge due to chuffing or accidental bump at the pad
Optional Apogee delay
Optional Audible Continuity & Battery Voltage report at startup
Optional Magnetic Switch activation/shutdown of startup sequence to increase safety of 2-stage and complex flight modes
Optional inflight recovery will resume flight functions if the computer is reset due to momentary power loss (user configurable)
Audible Post-flight max altitude & speed report
Telemetry over amateur 70cm band (433MHz) or 915MHz ISM band or license-free 915MHz FHSS (SMA antenna connector)
Can be mounted in any orientation, self-detects orientation (requires 1-time calibration)
Separate file for each flight up to 100 flights
Bench-test mode activated w/ tactile button to simulate flight tests on the ground and test pyro outputs
USB Serial debugging & status reporting in bench-test & calibration modes
Built-in self-calibration & orientation detection mode
User defined flight profile read from SD card
Code compatible with multiple sensors from different manufacturers (read list below)
Configurable GPIO pin and I2C bus options
Report in SI or Metric units
Preflight audible reporting options: Perfectflight or Marsa
One-time hardware configuration via SD card, settings stored in EEPROM

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 3.5 or 3.6 (compatible with provided PCB file) or Teensy 4.1/4.0/3.2 (no PCB file provided)
         9 DoF IMU: LSM9DS1, LSM6DS33, or LSM303 & L3GD20H combo
100G Accelerometer: H3LIS331DL, ADXL377, or ADXL377 & ADS1115 combo
   Pressure Sensor: MS5611, MS5607, MPL3115A2, BMP180, BMP280, or BMP388 (Note: BMP280 & BMP388 incompatible with telemetry due to EMI)
              GNSS: UBLOX M6, M7, M8, M9 (NEO footprint supported in PCB file)
    LoRa Telemetry: RFM96W 433MHz (USA: Amateur License Required, EUR: License Free) or RFM95W 915MHz (USA: FHSS License Free or Amateur License without FHSS)
   Tactile Buttons: 0.2 in spacing
               PCB: Create your own or use the provided design file for PCBexpress
           Battery: 2-cell 400mAh 20C LiPo recommended (DO NOT EXCEED 10V with provided PCB)

--------GROUND STATION COMPONENTS----------
                  MCU: Teensy 3.2: 
            Telemetry: RFM96W 433MHz (USA: Ham License Required, EUR: License Free) or RFM95W 915MHz (USA: License Free)
                  GPS: Ublox SAM-M8Q
                  IMU: Sparkfun LSM9DS1 breakout
                  LCD: 20X4 Sparkfun LCD
              SD Card: Adafruit SD Card breakout board
              Antenna: 433Mhz 5-element Yagi (Ham Only) or 900Mhz (License Free)
                 Case: File provided for 3D printed case
           BNC to SMA: 6-inch cable
            SMA cable: 6-inch cable
              Battery: Recommended 7.4V 2-cell LiPo 450MAh
      1K Ohm Resistor: 1/8 Watt
     10K Ohm Resistor: 1/8 Watt

--------ADDITIONAL LIBRARIES REQUIRED----------
SDFat V1.1.4 (V2.1 is not compatible)
TinyGPS++
RadioHead

--------INTENDED FUTURE REVISIONS----------
1) Upgrade ground station with GPS, digital compass, & Bluetooth
2) Develop Android App for ground station
3) Optional remote start & shutdown command

--------NOTES----------
Note: All of the above components were purchased through Digikey.  Other
sources for the same components should work, but are untested and may not
fit the provided PCB

Note: 1.25in x 4.0in size board.  Will fit in a 38mm coupler tube.
Components mount on both sides.  See provided pictures for examples.

Note: Estimated Flight Computer Cost = $154
Note: Estimated Ground Station Cost = $137 (does not include antenna)
Note: Estimated build time = 3hrs
Note: Estimated programming, setup, calibration, & debugging time = 4hrs
