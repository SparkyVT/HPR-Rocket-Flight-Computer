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
1000Hz 3-axis digital 24G and 100G accelerometer data logging
1000Hz 3-axis digital 2000dps gyroscope data logging
1000Hz of flight events and continuity
1000Hz of integrated speed, altitude, and rotation
20Hz of digital barometric data logging
20Hz of telemetry output (time, event, speed, altitude, accel, GPS, baro, rotation)/10Hz of magnetic data logging
8Hz of GPS data logging
Mach immune events
Sensor Fusion based apogee event
Barometric based main deploy event
Optional Apogee delay
Optional Two-Stage mode w/ tilt-sensing safety features
Audible Pre-flight main deploy setting and previous flight report
Audible Pre-flight pyro continuity and battery voltage report
Audible Post-flight altitude & velocity report
Separate file for each flight
Optional test mode for bench testing
Built-in self-calibration mode
User flight profile settings read from SD card
Optional magnetic startup switch
Onboard flight data recorded to SD card
Separate output file for each flight up to 99 stored flights
User selectable test mode for bench testing via button
User selectable self-calibration mode via button
Telemetry data recorded to ground station SD card

--------FLIGHT COMPUTER COMPONENTS----------
    Microprocessor: Teensy 3.5 (tested) or 3.6 or 3.2 (untested)
 9 DoF IMU: LSM9DS1
100G Accelerometer: H3LIS331DL
   Pressure Sensor: BMP280
               GPS: UBLOX NEO-M8N or SAM-M8Q
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
2) Airstart mode
3) Customizable pyro ouputs

--------NOTES----------
Note: All of the above components were purchased through Digikey.  Other
sources for the same components should work, but are untested, but may not
fit the provided PCB

Note: TinyGPS++ is only compatible with GPGGA and GPRMC sentences.  If using
a UBLOX M8N or other GNSS unit, then TinyGPS++ must be modified to accept
the GNGGA & GNRMC format.  Support available upon request.

Note: Required mounting space is 9.5 square inches.  Will fit in
a 2-inch avionics bay that is at least 1 inch wide and 8 inches long.  
Components mount on both sides.  See provided pictures for examples.

Note: Estimated Flight Computer Cost = $150
Note: Estimated Ground Station Cost = $165
Note: Estimated build time = 2hrs
Note: Estimated programing & debugging time = 10hrs
