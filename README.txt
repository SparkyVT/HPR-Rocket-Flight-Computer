HPR Rocket Flight Computer
Original sketch by Bryan Sparkman
NAR #85720, L3
TRA #12111, L3

--------DESCRIPTION----------
This flight computer is designed for rockets 2-inches in diameter or greater.
It has similar functionality as a TeleMega, but without a smart-phone app and
extra pyro channels.  It has been flown and tested on multiple M-powered
supersonic flights to 18K feet.  For large projects, a backup computer such 
as the PerfectFlite StratoLogger is strongly recommended.  

--------FEATURES----------
//1500Hz 3-axis digital 24G and 100G accelerometer data logging
//1500Hz 3-axis digital 2000dps gyroscope data logging
//1500Hz of flight events
//1500Hz of integrated speed, altitude, DCM2D rotation, continuity, events
//100Hz of user selectable quaternion rotation
//30Hz of digital barometric data logging (Altitude, pressure, temperature)
//30Hz of main battery voltage
//20Hz of telemetry output (time, event, acceleration, speed, altitude, rotation, GPS)
//10Hz of magnetic data logging
//8Hz of GPS data logging
//4 programmable pyro outputs with continuity checks
//User Selectable Flight Mode: Single-Stage, Two-Stage, Airstart
//Mach immune events
//Sensor Fusion based apogee event
//Barometric based main deploy event
//Optional Apogee delay
//Optional Audible Continuity report at startup
//Optional Audible Battery Voltage report at startup
//Optional Magnetic Switch Flight Activation
//Audible Post-flight max altitude & speed report
//Can be mounted in any orientation
//Separate file for each flight up to 100 flights
//Bench-test mode activated w/ tactile button
//Built-in self-calibration mode
//Reads user flight profile from SD card
//Compatible with multiple different sensors
//Configurable pyro pin outputs and I2C bus options
//Report in SI or Metric units
//Preflight audible reporting options: Perfectflight, Marsa, Raven

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 3.5 (tested) or 3.6 or 3.2 (untested)
         9 DoF IMU: LSM9DS1, or LSM303 & L3GD20H combo
100G Accelerometer: H3LIS331DL, ADXL377, or ADXL377 & ADS1115 combo
   Pressure Sensor: MPL5115A2, BMP180, BMP280, or BMP388
               GPS: UBLOX M6, M7, M8
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
1) Develop Android App for ground station

--------NOTES----------
Note: All of the above components were purchased through Digikey.  Other
sources for the same components should work, but are untested, but may not
fit the provided PCB

Note: TinyGPS++ is only compatible with GPGGA and GPRMC sentences.  If using
a UBLOX M8N or other GNSS unit, then TinyGPS++ must be modified to accept
the GNGGA & GNRMC format.  Support available upon request.

Note: 1.2 in x 3.8 in size board.  Will fit in a 2-inch avionics bay that is at least 
1 inch wide and 4 inches long.  Components mount on both sides.  See provided pictures for examples.

Note: Estimated Flight Computer Cost = $150
Note: Estimated Ground Station Cost = $165
Note: Estimated build time = 3hrs
Note: Estimated programing & debugging time = 10hrs
