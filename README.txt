HPR Rocket Flight Computer
Original sketch by SparkyVT
NAR #85720, L3
TRA #12111, L3

*NOTE: If you are planning to use this code or board files, please send me a PM on Rocketry Forum 
to ensure I have posted the latest updates.  https://www.rocketryforum.com/members/sparkyvtflyer.810/  

--------DESCRIPTION----------
This flight computer is designed for rockets 2-inches in diameter or greater.
It has similar capability as a TeleMega, but without a smart-phone app or advanced interfaces  
It has flown on multiple M-powered supersonic flights to over 18K feet.  
For large & high-power projects, a backup computer such as the PerfectFlite StratoLogger is strongly recommended.  

--------FEATURES----------
//1400Hz 3-axis digital 24G and 100G accelerometer data logging
//1400Hz 3-axis digital 2000dps gyroscope data logging
//1400Hz of flight events
//1400Hz of integrated speed, altitude, continuity, events
//1000Hz of quaternion rotation, user selectable down to 100Hz
//30Hz of digital barometric data logging (Altitude, pressure, temperature)
//30Hz of main battery voltage
//20Hz of 70cm LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GPS)
//10Hz of magnetic data logging
//10Hz of GPS data logging
//All data collected to a CSV text file, 3-5 million data points typical per flight 
//4 programmable pyro outputs with continuity checks
//4 programmable powered servo connections (plug 'n play)
//4 programmable un-powered servo outputs (requires JST connector)
//User Selectable Flight Mode: Single-Stage, Two-Stage Sustainer, Two-Stage Booser, Airstart
//Mach immune events
//Sensor Fusion based apogee event
//Barometric based main deploy event
//Optional Apogee delay
//Optional Audible Continuity report at startup
//Optional Audible Battery Voltage report at startup
//Optional Magnetic Switch activation of startup sequence
//Optional antenna SMA connector
//Audible Post-flight max altitude & speed report
//Can be mounted in any orientation, self-detects orientation
//Separate file for each flight up to 100 flights
//Bench-test mode activated w/ tactile button
//USB Serial status reporting in bench-test & calibration modes
//Built-in self-calibration & orientation detection mode
//User defined flight profile read from SD card
//Compatible with multiple different sensors
//Configurable GPIO pin and I2C bus options
//Kalman smoothing of High-G acceleration & barometric data
//Report in SI or Metric units
//Preflight audible reporting options: Perfectflight or Marsa
//One-time hardware configuration via SD card, settings stored in EEPROM

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 3.5 (tested) or 4.1, 4.0, 3.6 or 3.2 (untested)
         9 DoF IMU: LSM9DS1, or LSM303 & L3GD20H combo
100G Accelerometer: H3LIS331DL, ADXL377, or ADXL377 & ADS1115 combo
   Pressure Sensor: MPL3115A2, BMP180, BMP280, or BMP388
               GPS: UBLOX M6, M7, M8, M9
    LoRa Telemetry: RFM95W (License Free - untested) or RFM96W (Ham License Only - tested)
   Tactile Buttons: 0.25 in spacing
               PCB: Create your own or use the provided design file for PCBexpress
           Battery: 2-cell 500mAh LiPo recommended

--------GROUND STATION COMPONENTS----------
Adafruit Feather LoRa: ATMega 32u4 w/ RFM95W (License Free - untested) or RFM96W (Ham License Only - tested)
                  LCD: 20X4 LCD
              SD Card: SD Card breakout board
              Antenna: 900Mhz (License Free) or 433Mhz 5-element Yagi (Ham Only)
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
1) 915MHz option & LoRaWAN compatibility
2) Upgrade ground station with GPS, digital compass, & Bluetooth
3) Develop Android App for ground station

--------NOTES----------
Note: All of the above components were purchased through Digikey.  Other
sources for the same components should work, but are untested, but may not
fit the provided PCB

Note: 1.2in x 3.8in size board.  Will fit in a 2-inch avionics bay that is at least 
1 inch wide and 4 inches long.  Components mount on both sides.  See provided pictures for examples.

Note: Estimated Flight Computer Cost = $150
Note: Estimated Ground Station Cost = $130-$180 (configuration dependent)
Note: Estimated build time = 3hrs
Note: Estimated programming, setup, calibration, & debugging time = 4hrs
