*NOTE: If you are planning to use this code or board files, please send me a PM on Rocketry Forum 
to ensure I have posted the latest updates.  https://www.rocketryforum.com/members/sparkyvtflyer.810/  

HPR Rocket Flight Computer
Original sketch by SparkyVT
NAR #85720, L3
TRA #12111, L3

--------DESCRIPTION----------
This flight computer is designed for rockets 2-inches in diameter or greater.
Dimensions are 3.8in x 1.25in x 0.5in, not including antenna
It has similar capability as a TeleMega, but without a smart-phone app or advanced interfaces  
It has flown on multiple M-powered supersonic flights to over 24K feet and Mach 2.0.  
For large or high-power projects, a backup computer such as the PerfectFlite StratoLogger is strongly recommended.  

--------FEATURES----------
High data logging rate: 3-5 million data points per flight is typical
1400Hz 3-axis digital 24G and 100G accelerometer data logging
1400Hz 3-axis digital 2000dps gyroscope data logging
1400Hz of flight events
1400Hz of integrated speed and altitude
1400Hz of pyro status (continuity detection, closed/open circuit, and pin debugging)
100Hz - 500Hz of user selectable quaternion rotation calculation rate
40Hz of magnetic field data logging
30Hz of digital barometric data logging (Altitude, pressure, temperature)
30Hz of main battery voltage
20Hz of LoRa telemetry output on either ham amateur band 433MHz or license-free 915MHz FHSS (time, event, acceleration, speed, altitude, rotation, GNSS)
10Hz - 20Hz of GNSS location data on GPS, GLONASS, and Galileo systems (UBLOX chip dependent, 0.1Hz post-flight location updates)
All data written to an easy to read CSV text file
4 programmable pyro outputs with continuity checks, detects and reports open circuits.
4 programmable "plug 'n play" servo connections, can be actuated at flight events (active stabilization in development)
4 additional programmable un-powered servo outputs (requires JST connector)
User Selectable Flight Mode: Single-Stage, Two-Stage Sustainer, Two-Stage Booser, Airstart
"Smart Config" will default to single-stage mode if complex pyros not detected
Mach immune flight event detection algorithms
Sensor Fusion based apogee event
Barometric based main deploy event
False-liftoff detection prevents accidental pyro discharge due to chuffing or accidental bump at the pad
Optional Apogee delay
Optional Audible Continuity report at startup
Optional Audible Battery Voltage report at startup
Optional Magnetic Switch activation/shutdown of startup sequence to increase safety of 2-stage and complex flight modes
Optional SMA antenna connector
Audible Post-flight max altitude & speed report
Can be mounted in any orientation, self-detects orientation (requires 1-time calibration)
Separate file for each flight up to 100 flights
Bench-test mode activated w/ tactile button to simulate flight tests on the ground and test pyro outputs
USB Serial debugging & status reporting in bench-test & calibration modes
Built-in self-calibration & orientation detection mode
User defined flight profile read from SD card
Compatible with multiple different sensors (read list below)
Configurable GPIO pin and I2C bus options
Kalman smoothing of High-G acceleration & barometric data (coming in V4.3)
Report in SI or Metric units
Preflight audible reporting options: Perfectflight or Marsa
One-time hardware configuration via SD card, settings stored in EEPROM

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 3.5 or 3.6 (compatible with provided motherBoard PCB file) or Teensy4.1, 4.0, or 3.2 (no PCB file provided)
         9 DoF IMU: LSM9DS1, or LSM303 & L3GD20H combo
100G Accelerometer: H3LIS331DL, ADXL377, or ADXL377 & ADS1115 combo
   Pressure Sensor: MPL3115A2, BMP180, BMP280, or BMP388 (Note: BMP280 & BMP388 are EMI sensitive and unsuitable for use with 433MHz telemetry)
              GNSS: UBLOX M6, M7, M8, M9 (NEO-M8N/Q supported in PCB file)
    LoRa Telemetry: RFM96W 433MHz (USA: Ham License Required, EUR: License Free) or RFM95W 915MHz (USA: License Free)
   Tactile Buttons: 0.2 in spacing
               PCB: Create your own or use the provided design file for PCBexpress
           Battery: 2-cell 500mAh 20C LiPo recommended (DO NOT EXCEED 10V with provided PCB)

--------GROUND STATION COMPONENTS----------
Adafruit Feather LoRa: ATMega 32u4 w/ RFM96W 433MHz (USA: Ham License Required, EUR: License Free) or RFM95W 915MHz (USA: License Free)
                  LCD: 20X4 LCD
              SD Card: SD Card breakout board
              Antenna: 433Mhz 5-element Yagi (Ham Only) or 900Mhz (License Free)
                 Case: https://www.alliedelec.com/hammond-manufacturing-1591xxcgy/70165817/
           BNC to SMA: 6-inch cable
           SMA to uFl: 6-inch cable
              Battery: Standard 9V
      1K Ohm Resistor: 1/8 Watt
     10K Ohm Resistor: 1/8 Watt

--------ADDITIONAL LIBRARIES REQUIRED----------
SDFat
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

Note: 1.25in x 3.8in size board.  Will fit in a 2-inch avionics bay that is at least 
1.5 inch wide and 4 inches long, antenna dependent.  Components mount on both sides.  See provided pictures for examples.

Note: Estimated Flight Computer Cost = $150
Note: Estimated Ground Station Cost = $130-$180 (configuration dependent)
Note: Estimated build time = 3hrs
Note: Estimated programming, setup, calibration, & debugging time = 4hrs
