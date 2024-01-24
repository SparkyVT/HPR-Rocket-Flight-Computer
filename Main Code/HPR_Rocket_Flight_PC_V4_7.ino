//High-Power Rocketry Flight Computer (TeensyFlight)
//Original sketch by Bryan Sparkman, TRA #12111, NAR #85720, L3
//Built for Teensy 3.2, 3.5, 3.6, 4.0, and 4.1
//Code Line Count: 10223 lines = 1640 MainFile + 773 Header + 923 Calibration + 327 Event_Logic + 553 Rotation + 625 SpeedTrig + 683 SD + 418 Telemetry + 466 Inflight_Recover + 352 Bus_Mgmt + 578 Device_Mgmt + 1931 Device_Drivers + 336 Device_Drivers_External + 340 SX127X_Driver + 278 UBLOX_GNSS_Config      
//--------FEATURES----------
//Dual-deploy flight computer capable to over 100,000ft 
//Two-stage & airstart capable with tilt-sensing safety features
//Live telemetry over 433MHz or 915MHz bands (433MHz: USA amateur 70cm band, EUR licencse free) (915MHz: USA FHSS licence free)
//--Optional APRS packets at 0.5Hz or telemetry data packets at 5Hz (APRS in development)
//--Optional LoRa or GFSK modulation (in development)
//--Optional 915MHz single channel or FHSS
//--Auto-adjusting power gain
//4 programmable high-current pyro outputs with continuity checks
//Captures high-rate data at approximately 50,000 data points per second on SD card
//--1000Hz 3-axis digital 16G and 100G accelerometer data logging
//--1000Hz 3-axis digital 2000dps gyroscope data logging
//--1000Hz of 21 flight events & continuity data logging on 4 pyros
//--1000Hz of sensor-fuzed speed & altitude
//--100Hz of roll, pitch, yaw, and combined pitch-yaw 3D rotation
//--40Hz of of magnetic data logging and magnetic roll
//--30Hz-100Hz of digital barometric data logging (Altitude, pressure, temperature - data rate is chip dependent)
//--30Hz of main battery voltage (1000Hz during pyro events)
//--20Hz of LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GNSS altitude, GNSS position, signal strength, packet number)
//--5Hz-25Hz of GNSS data logging (chip-dependent data rates & constellations)
//--Separate data file for each flight up to 100 flights
//Simple, easy-to-use setup through the SD card
//--User Selectable Flight Mode: Single-Stage, Two-Stage, Airstart, or Booster
//--Configurable Apogee delay
//--Optional Audible Battery Voltage report at startup
//--Optional Magnetic Switch Startup & Shut-down
//--Preflight audible reporting options: Perfectflight or Marsa
//--8 configurable servo outputs (4 powered, 4 un-powered)
//--User selectable inflight brownout recovery
//--User selectable active stabilization for roll, pitch, and yaw correction
//--User selectable return-to-pad controlled recovery
//Tilt-sensing lockout for ignition of second stages and/or airstarts
//Mach immune, sensor-fusion based apogee event
//Barometric based main deploy event
//Optional GNSS NMEA Log for TRA altitude contest reporting
//Audible pre-flight continuity report
//Audible post-flight max altitude & speed report
//Mount in any orientation, automatic orientation detection during calibration
//Bench-test mode activated w/ tactile button; user configurable status messages over USB Serial
//Built-in self-calibration mode 
//Report in SI or Metric units
//Compatible with Teensy 3.2, 3.5, 3.6, 4.0, 4.1
//--Compatible with any sensor over I2C or SPI, optimized drivers support specific sensors, external libraries can be used for unsupported devices
//--Configurable GPIO pins and hardware I2C/SPI bus options
//--GNSS receiver can be wired to any available HW Serial port (UBLOX chipsets & Adafruit Ultimate GPS supported, or use external library)
//-----------Change Log------------
//V4_0_0 combines all previous versions coded for specific hardware setups; now compatible across multiple hardware configurations and can be mounted in any orientation
//V4_1_0 adds upgrades for airstart capability, more flight event codes, improved settings file, PWM Pyro firing, quaternion rotation, improved continuity reporting, and timer interrupts for the radios
//V4_1_1 eliminates timer interrupts since they interfere with the micros() command, improves telemetry timing, enables the Galileo GNSS system
//V4_1_2 increases the GPS data rates and fixes some bugs in the pyro firing timing
//V4_1_3 adds features to the settings file, moves some settings to the EEPROM file, adds calibration for the MPL3115A2, and fixes several smaller bugs
//V4_1_4 adds magnetic rotation and increases the magnetic data capture rate (not functional yet)
//V4_2_0 adds 915MHz FHSS telemetry capability, configurable GNSS & radio debugging
//V4_2_1 was an alternate 915MHZ FHSS prototype: not used
//V4_2_2 adds support for the MS5611 pressure sensor
//V4_2_3 adds inflight recovery from brown-out or MCU reset, sensor fusion altitude and velocity, eliminates gyro high-pass filter, and revamps the EEPROM mapping for greater flexibility
//V4_3_0 updates Quaternion rotation for the best possible precision, corrects some GNSS bugs, adds a faster version of the original rotation code
//V4_3_1 makes FHSS an option on 915MHz and corrects some bugs in the FHSS power settings code, adds serial debug options into the settings file
//V4_3_2 fixes bugs in the orientation & rotation routines, forces inline functions for faster execution, fixes multiple bugs picked up by the new Arduino compiler, breaks out more tabs for readability
//V4_3_3 creates an user interface over serial to calibrate the barometer, extends gyro orientation integration through the entire flight
//V4_3_3_1 is a bridge to fix instability in the initial V4_3_4 code
//V4_3_4 enables active stabilization, creates basic fin trim code, and enables auto-adjusting gain on the IMU accelerometer
//V4_3_5 removes auto-adjusting gain (no benefit in testing), improved calibration routines, extends baro calibration to all sensors, removed high-G axis mode from user settings, adds H3LIS331DL SPI bus support, added Ublox M9N support, fixed MS5611 bugs, added all settings to SD footer
//V4_3_5_2 is a stand-alone version that evaluates cycle time diagnostics in order to ID chokepoints in the loop.  Results are nearly the same as the previous test done in 2019
//V4_3_6 adds return-to-base functionality (beta testing), adds support for LSM6DS33, LIS3MDL, and MS5607, updates code for separate mag sensors, update for no high-G accelerometer, added sensor calibration checks & warnings, I2C bus & SDFat compatible with Teensy 3.2, 4.0, and 4.1
//V4_4_0 creates flexible I2C and SPI bus routines so that any device can work on any bus.  Supports all I2C and/or SPI buses across Teensy3.X and Teensy4.X platforms
//V4_4_1 builds on the unsuccessful 4_4_0 and attempts to fix bus management through simpler interface functions, overhauls orientation method for simplicity 
//V4_4_2 further streamlines the bus management functions with more efficient pointers, overclocks I2C speed on some sensors, implements controlled sampling of all sensors, continuously checks pre-flight continuity, improved barometric smoothing, fixed orientation bugs
//V4_5_0 eliminates RadioHead library due to interferences with the PWMServo library, now uses an interval timer for precise control of the radio packet timing, fixes bugs in the telemetry timestamps
//V4_5_1 adds optional GPS output file to capture NMEA sentences for contest flights, creates better radio resilliency by adding the callsign back into the header of the packet (coded but not yet implemented), fixed bug when TX and beep are both off in test mode
//V4_5_2 finishes the draft RTB code, adds a canard calibration flight mode, fixed GPS bug with NEO-M8N
//V4_5_3 uses the RFM96W in both 70cm and 900MHz mode based on user frequency input for 3 options: (1)900MHz FHSS up to 20dB, (2)900MHz dedicated frequency at 2dB, (3)70cm dedicated frequecy up to 20dB
//V4_5_4 fixes a bug in the sensor timing that reduced the effective data capture rate
//V4_5_5 removes launch detection user options since the algorithm is proven reliable, fixes a timing bug with the barometers, makes GPS configuration code more portable, adds LPS25H support, fixes bugs with LSM6DS33 and LIS3MDL, fixes SD card pre-processor bugs, added support for Adafruit Ultimate GPS
//V4_6_0 improves portability of quaternion rotation code, added MPU6050 support, fixed calibration routine bug, cleaned up some of the GPS data processing, corrected major bug in high-G moving average, fixed a bug with the UBLOX power save mode
//V4_7_0 adds support for any sensor with an external library, fixed a bug at liftoff that caused intermittent resetting when moving up the rail, created breakout file for sensor management, cleaned up some of the structs
//-------FUTURE UPGRADES----------
//Active Stabilization (started)
//Return-to-Base capability (started)
//3D position physics model
//Remote Arm & Shutdown Commands over telemetry
//Ground Station Bluetooth Datalink to smartphone (started)
//Smartphone App
//------TO DO LIST------
//Finish bench testing of inflight recovery routines
//Flight test airstart code
//Debug Inflight Recovery
//stuck-in-a-loop detection and breakout
//-------CODE START--------
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <PWMServo.h>
#include <SPI.h>
#include "declarations.h"

//Teensy 3.X
#if defined (__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MK20DX256__)
  //i2c_t3 is needed to enable the different pin options of bus0
  #include <i2c_t3.h>
  //NOTE: typedef declarations must be in the main file and not the tabs
  typedef i2c_t3 TwoWire;
//Teensy 4.X
#else
  #include <Wire.h>
#endif

//Hardware Serial for GPS
HardwareSerial *HWSERIAL;

// GPS Setup
TinyGPSPlus GPS;

//Servo Setup
PWMServo canardYaw1; 
PWMServo canardYaw2;
PWMServo canardPitch3;
PWMServo canardPitch4;
PWMServo actionServo5;
PWMServo actionServo6;
PWMServo actionServo7;
PWMServo actionServo8;

//Timer for the radio

#if defined (STM32_CORE_VERSION)
  #if defined (TIM1)
    TIM_TypeDef *Instance = TIM1;
  #else
    TIM_TypeDef *Instance = TIM2;
  #endif
  HardwareTimer *radioTimer = new HardwareTimer(Instance);
  HardwareTimer *syncTimer  = new HardwareTimer(Instance);
#else
  IntervalTimer radioTimer;
  IntervalTimer syncTimer;
#endif

//GLOBAL VARIABLES
//-----------------------------------------
//Set code version
//-----------------------------------------
const float codeVersion = 4.7;

//-----------------------------------------
//Bus Variables
//-----------------------------------------
//This struct contains all necessary bus info for each device
typedef struct{
  char type;
  TwoWire *wire = &Wire;
  uint32_t i2cRate = 400000;
  uint8_t i2cAddress;
  SPIClass *spi = &SPI;
  SPISettings spiSet = SPISettings(10000000, MSBFIRST, SPI_MODE0);
  byte cs;
  uint8_t writeMask = 0x00;
  uint8_t readMask = 0x00;
  uint8_t incMask = 0x00;
} _bus;
_bus accelBus;
_bus gyroBus;
_bus magBus;
_bus highGBus;
_bus baroBus;
_bus radioBus;
_bus sdBus;
_bus *activeBus;

//typedef void (*getAccel)

//any routines called in the main flight loop need to be as fast as possible, so set them as "always inline" for the compiler
inline void checkEvents(void) __attribute__((always_inline));
inline void getDCM2DRotn(void) __attribute__((always_inline));
inline void getQuatRotn(void) __attribute__((always_inline));
inline void radioSendPacket(void) __attribute__((always_inline));
inline void writeSDflightData(void) __attribute__((always_inline));
inline void writeFloatData(void) __attribute__((always_inline));
inline void writeIntData(void) __attribute__((always_inline));
inline void writeBoolData(void) __attribute__((always_inline));
inline void writeLongData(void) __attribute__((always_inline));
inline void writeULongData(void) __attribute__((always_inline));
inline void updateStrPosn(void) __attribute__((always_inline));
inline void burstRead(void) __attribute__((always_inline));

void setup(void) {
  
  Serial.begin(38400);
  
  delay(500);

  //check to see if we are restarting from a powerloss
  //if so, then we need to rapidly get the system back up and going
  //call the rapidReset routine and immediately exit void setup
  uint8_t lastEvent = EEPROM.read(eeprom.lastEvent);
  if(lastEvent != 27){
    EEPROM.update(eeprom.lastEvent, 27);
    lastEvent = 27;}
  if(lastEvent == (byte)255){
    lastEvent = 27;
    EEPROM.update(eeprom.lastEvent, lastEvent);}
  settings.inflightRecover = EEPROM.read(eeprom.inflightRecover);
  if(settings.inflightRecover > 0 && (lastEvent < 24 && lastEvent != 6 && lastEvent != 7)){
    
    //execute the rapid recovery routine
    Serial.print("Rapid Reset Initiated, setting: ");Serial.println(settings.inflightRecover);
    if(rapidReset()){
      //exit setup and immediately re-enter the loop
      return;}}
  
  //Start SD card
  beginSD();

  //if an EEPROM settings file exists on the SD card, read it and store it into EEPROM
  parseEEPROMsettingsSD();
    
  //Read pin settings from EEPROM
  Serial.print(F("Reading EEPROM..."));
  pins.accelCS = EEPROM.read(eeprom.accelCSpin);
  pins.gyroCS = EEPROM.read(eeprom.gyroCSpin);
  pins.magCS = EEPROM.read(eeprom.magCSpin);
  pins.highG_CS = EEPROM.read(eeprom.highGcsPin);
  pins.SD_CS = EEPROM.read(eeprom.sdCSpin);
  pins.baroCS = EEPROM.read(eeprom.baroCSpin);
  pins.pyro1Cont = EEPROM.read(eeprom.pyro1ContPin);
  pins.pyro1Fire = EEPROM.read(eeprom.pyro1FirePin);
  pins.pyro2Cont = EEPROM.read(eeprom.pyro2ContPin);
  pins.pyro2Fire = EEPROM.read(eeprom.pyro2FirePin);
  pins.pyro3Cont = EEPROM.read(eeprom.pyro3ContPin);
  pins.pyro3Fire = EEPROM.read(eeprom.pyro3FirePin);
  pins.pyro4Cont = EEPROM.read(eeprom.pyro4ContPin);
  pins.pyro4Fire = EEPROM.read(eeprom.pyro4FirePin);
  pins.nullCont = EEPROM.read(eeprom.nullPin);
  pins.nullFire = pins.nullCont;
  pins.beep = EEPROM.read(eeprom.beepPin);
  pins.batt = EEPROM.read(eeprom.battReadPin);
  pins.testGnd = EEPROM.read(eeprom.testModeGndPin);
  pins.testRead = EEPROM.read(eeprom.testModeRdPin);
  pins.radioCS = EEPROM.read(eeprom.radioCSpin);
  pins.radioIRQ = EEPROM.read(eeprom.radioIRQpin);
  pins.radioRST = EEPROM.read(eeprom.radioRstPin);
  pins.radioEN = EEPROM.read(eeprom.radioEnPin);
  pins.servo1 = EEPROM.read(eeprom.servo1pin);
  pins.servo2 = EEPROM.read(eeprom.servo2pin);
  pins.servo3 = EEPROM.read(eeprom.servo3pin);
  pins.servo4 = EEPROM.read(eeprom.servo4pin);
  pins.servo5 = EEPROM.read(eeprom.servo5pin);
  pins.servo6 = EEPROM.read(eeprom.servo6pin);
  pins.servo7 = EEPROM.read(eeprom.servo7pin);
  pins.servo8 = EEPROM.read(eeprom.servo8pin);
  //read sensor settings from EEPROM
  sensors.accel = EEPROM.read(eeprom.accelID);
  sensors.mag = EEPROM.read(eeprom.magID);
  sensors.gyro = EEPROM.read(eeprom.gyroID);
  sensors.highG = EEPROM.read(eeprom.highGID);
  sensors.baro = EEPROM.read(eeprom.baroID);
  sensors.radio = EEPROM.read(eeprom.radioID);
  sensors.GNSS = EEPROM.read(eeprom.GPSID);
  sensors.accelBusType = (char)EEPROM.read(eeprom.accelBusType);
  sensors.gyroBusType = (char)EEPROM.read(eeprom.gyroBusType);
  sensors.magBusType = (char)EEPROM.read(eeprom.magBusType);
  sensors.highGBusType = (char)EEPROM.read(eeprom.highGBusType);
  sensors.baroBusType = (char)EEPROM.read(eeprom.baroBusType);
  sensors.sdBusType = (char)EEPROM.read(eeprom.sdBusType);
  sensors.gnssBusType = (char)EEPROM.read(eeprom.gpsBusType);
  sensors.radioBusType = (char)EEPROM.read(eeprom.radioBusType);
  sensors.accelBusNum = EEPROM.read(eeprom.accelBusNum);
  sensors.gyroBusNum = EEPROM.read(eeprom.gyroBusNum);
  sensors.magBusNum = EEPROM.read(eeprom.magBusNum);
  sensors.highGBusNum = EEPROM.read(eeprom.highGBusNum);
  sensors.baroBusNum = EEPROM.read(eeprom.baroBusNum);
  sensors.sdBusNum = EEPROM.read(eeprom.sdBusNum);
  sensors.gnssBusNum = EEPROM.read(eeprom.gpsBusNum);
  sensors.radioBusNum = EEPROM.read(eeprom.radioBusNum);
  for(byte i = 0; i < sizeof(settings.callSign); i++){settings.callSign[i] = EEPROM.read(eeprom.callSign+i);}
  settings.callSign[sizeof(settings.callSign)-1] = '\0';
  settings.HWid[0] = (char)EEPROM.read(eeprom.HWversion);
  settings.HWid[1] = '.';
  settings.HWid[2] = (char)EEPROM.read(eeprom.HWsubVersion);
  settings.HWid[3] = '.';
  settings.HWid[4] = (char)EEPROM.read(eeprom.HWunitNum);
  settings.HWid[5] = '\0';
  Serial.println(F("complete!"));
  
  //Set the mode of the output pins
  pinMode(pins.nullCont, INPUT);
  pinMode(pins.nullFire, INPUT);
  pinMode(pins.pyro1Cont, INPUT);           
  pinMode(pins.pyro2Cont, INPUT);          
  pinMode(pins.pyro3Cont, INPUT);         
  pinMode(pins.pyro4Cont, INPUT);           
  pinMode(pins.pyro1Fire, OUTPUT);             
  pinMode(pins.pyro2Fire, OUTPUT);            
  pinMode(pins.pyro3Fire, OUTPUT);
  pinMode(pins.pyro4Fire, OUTPUT);   
  pinMode(pins.beep, OUTPUT);            
  pinMode(pins.testRead, INPUT_PULLUP);   
  pinMode(pins.testGnd, OUTPUT); 
  pinMode(pins.radioCS, OUTPUT);       
  pinMode(pins.highG_CS, OUTPUT);
  //Set the CS pins to HIGH
  digitalWrite(pins.radioCS, HIGH);
  digitalWrite(pins.highG_CS, HIGH);
  //Set the pyro firing pins to LOW for safety
  digitalWrite(pins.pyro1Fire, LOW);
  digitalWrite(pins.pyro2Fire, LOW);
  digitalWrite(pins.pyro3Fire, LOW);
  digitalWrite(pins.pyro4Fire, LOW);
  
  //Start Harware Serial communication
  setHWSERIAL();
  if(sensors.GNSS == 3){
    HWSERIAL->begin(38400);
    Serial.println("Starting HWSerial at 38400 baud");}
  else{
    HWSERIAL->begin(9600);
    Serial.println("Starting HWSerial at 9600 baud");}
  
  //check if the test mode button is being held
  digitalWrite(pins.testGnd, LOW);
  delay(50);
  if(digitalRead(pins.testRead) == LOW){
    settings.testMode = true; 
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Bench-Test Mode Confirmed"));
    Serial.println(F("Press the button again to calibrate the accelerometers."));
    Serial.println(F("Enter 'b' into the serial monitor to calibrate the barometer"));
    Serial.println(F("Enter 'c' into the serial monitor to adjust the canard trims"));
    Serial.println(F("------------------------------------------------------------"));}

  //setup the ADC for sampling the battery and ADXL377 if present
  #if defined (__MK66FX1M0__) || defined (__MK64FX512__) || defined (__MK20DX256__)
    //16 bit for Teensy3.5 and 3.6
    analogReadResolution(16);
  #elif defined (__IMXRT1062__)
    //12 bit resolution for Teensy 4.0 & 4.1
    analogReadResolution(12);
    adcConvert = 0.000244140625;//=1/4096
    ADCmidValue = 2048;
  #endif

  //read the flight settings from the SD card
  readFlightSettingsSD();

  //Start Sensors
  Serial.println(F("Starting Sensors..."));
  beginAccel();
  beginGyro();
  beginMag();
  beginHighG();
  beginBaro();
  
  //Initialize the radio
  //if the Adafruit RFM9XW board is used, make sure its on
  if(pins.radioEN != pins.nullCont){pinMode(pins.radioEN, OUTPUT);digitalWrite(pins.radioEN, HIGH);}
  //start the radio
  if(settings.TXenable){
    beginRadio();
    beginTelemetry();
    if(!sensors.statusRadio){Serial.println(F("Radio Not Found!"));}}
  else{Serial.println("Radio Disabled!");}
  
  //safety override of user settings
  if (settings.apogeeDelay > 5000000UL) {settings.apogeeDelay = 5000000UL;} //5s max apogee delay
  if (settings.fireTime > 1000000UL) {settings.fireTime = 1000000UL;} //1s max firing length
  if (settings.mainDeployAlt > 458){settings.mainDeployAlt = 458;}//max of 1500 ft
  if (settings.mainDeployAlt < 30) {settings.mainDeployAlt = 30;}//minimum of 100ft
  if (settings.sustainerFireDelay > 8000000UL){settings.sustainerFireDelay = 8000000UL;}//maximum 8s 2nd stage ignition delay
  if (settings.boosterSeparationDelay > 3000000UL){settings.boosterSeparationDelay = 3000000UL;}//max 3s booster separation delay after burnout
  if (settings.airStart1Delay > 2000000UL){settings.airStart1Delay = 2000000UL;}//max 2s airstart delay
  if (settings.airStart2Delay > 2000000UL){settings.airStart2Delay = 2000000UL;}//max 2s airstart delay
  if (settings.altThreshold < 91){settings.altThreshold = 91;}//minimum 100ft threshold
  if (settings.maxAngle > 450){settings.maxAngle = 450;}//maximum 45 degree off vertical
  if (settings.rcdTime < 300000000UL){settings.rcdTime = 300000000UL;}//min 5min of recording time
  if (settings.fireTime < 200000UL){settings.fireTime = 200000UL;}//min 0.2s of firing time
  if (settings.fireTime > 1000000UL){settings.fireTime = 1000000UL;}//max 1.0s of firing time
  if (settings.setupTime > 60000UL) {settings.setupTime = 60000UL;}//max 60 seconds from power-on to preflight start
  if (settings.setupTime < 3000UL) {settings.setupTime = 3000UL;}//min 3 seconds of setup time
  if (settings.TXpwr > 20){settings.TXpwr = 20;}//power cant exceed 100mW
  if (settings.TXpwr < 2){settings.TXpwr = 2;}//power setting can't go below 2
  if (settings.FHSS && settings.TXfreq < 900){settings.FHSS = false;}//FHSS not used on 70cm band
  if (settings.fltProfile == '2' or settings.fltProfile == 'A'){boosterBurpTime = min(1000000UL, settings.boosterSeparationDelay-10000UL);}
  
  //Update the EEPROM with the new settings
  EEPROM.update(eeprom.fltProfile, settings.fltProfile);
  EEPROM.update(eeprom.units, settings.units);
  EEPROM.update(eeprom.reportStyle, settings.reportStyle);
  ulongUnion.val = settings.setupTime;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.setupTime + i, ulongUnion.Byte[i]);}
  floatUnion.val = settings.mainDeployAlt;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.mainDeployAlt + i, floatUnion.Byte[i]);}
  ulongUnion.val = settings.apogeeDelay;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.apogeeDelay + i, ulongUnion.Byte[i]);}
  ulongUnion.val = settings.rcdTime;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.rcdTime + i, ulongUnion.Byte[i]);}
  EEPROM.update(eeprom.silentMode, settings.silentMode);
  EEPROM.update(eeprom.magSwitchEnable, settings.magSwitchEnable);
  EEPROM.update(eeprom.inflightRecover, settings.inflightRecover);
  EEPROM.update(eeprom.gpsLogFile, settings.GPSlog);
  ulongUnion.val = settings.fireTime;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.fireTime + i, ulongUnion.Byte[i]);}
  EEPROM.update(eeprom.pyro1Func, settings.pyro1Func);
  EEPROM.update(eeprom.pyro2Func, settings.pyro2Func);
  EEPROM.update(eeprom.pyro3Func, settings.pyro3Func);
  EEPROM.update(eeprom.pyro4Func, settings.pyro4Func);
  EEPROM.update(eeprom.TXenable, settings.TXenable);
  EEPROM.update(eeprom.TXpwr, settings.TXpwr);
  floatUnion.val = settings.TXfreq;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.TXfreq + i, floatUnion.Byte[i]);}
  ulongUnion.val = settings.boosterSeparationDelay;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.boosterSeparationDelay + i, ulongUnion.Byte[i]);}
  ulongUnion.val = settings.sustainerFireDelay;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.sustainerFireDelay + i, ulongUnion.Byte[i]);}
  EEPROM.update(eeprom.airStart1Event, settings.airStart1Event);
  ulongUnion.val = settings.airStart1Delay;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.airStart1Delay + i, ulongUnion.Byte[i]);}
  EEPROM.update(eeprom.airStart2Event, settings.airStart2Event);
  ulongUnion.val = settings.airStart2Delay;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.airStart2Delay + i, ulongUnion.Byte[i]);}
  intUnion.val = settings.altThreshold;
  for(byte i=0; i<2; i++){EEPROM.update(eeprom.altThreshold + i, intUnion.Byte[i]);}
  intUnion.val = settings.maxAngle;
  for(byte i=0; i<2; i++){EEPROM.update(eeprom.maxAngle + i, intUnion.Byte[i]);}
  EEPROM.update(eeprom.stableRotn, settings.stableRotn);
  EEPROM.update(eeprom.stableVert, settings.stableVert);
  
  //configure pyro outupts
  pyro1.func = settings.pyro1Func; pyro1.contPin = pins.pyro1Cont; pyro1.firePin = pins.pyro1Fire; pyro1.fireStatus = false; pyro1.fireStart = 0UL;
  pyro2.func = settings.pyro2Func; pyro2.contPin = pins.pyro2Cont; pyro2.firePin = pins.pyro2Fire; pyro2.fireStatus = false; pyro2.fireStart = 0UL;
  pyro3.func = settings.pyro3Func; pyro3.contPin = pins.pyro3Cont; pyro3.firePin = pins.pyro3Fire; pyro3.fireStatus = false; pyro3.fireStart = 0UL;
  pyro4.func = settings.pyro4Func; pyro4.contPin = pins.pyro4Cont; pyro4.firePin = pins.pyro4Fire; pyro4.fireStatus = false; pyro4.fireStart = 0UL;

  if(settings.testMode){
  Serial.print(F("Flight Profile: "));  Serial.println(settings.fltProfile);
  Serial.print(F("Pyro 4 Function: ")); Serial.println(pyro4.func);
  Serial.print(F("Pyro 3 Function: ")); Serial.println(pyro3.func);
  Serial.print(F("Pyro 2 Function: ")); Serial.println(pyro2.func);
  Serial.print(F("Pyro 1 Function: ")); Serial.println(pyro1.func);}
  
  //check for silent mode
  if(settings.testMode && settings.silentMode){pins.beep=pins.nullCont; Serial.println(F("Silent Mode Confirmed"));}

  //setup the radio
  if(settings.TXenable){
    //915MHz FHSS
    if (settings.FHSS){
      pktInterval.preLiftoff = 600000UL;
      if(settings.testMode){Serial.println("FHSS Active!");}}
    //915MHz dedicated frequency (no FHSS)
    if (settings.TXfreq > 900.000F && !settings.FHSS){
      settings.TXpwr = 2;//minimum power due to FCC regulations
      if(settings.testMode){Serial.println(F("Dedicated ISM band frequency. Power limit 2mW"));}}
    //Set the radio output power & frequency
    setRadioFreq(settings.TXfreq);
    setRadioPWR(settings.TXpwr);//23 max setting; 20mW=13dBm, 30mW=15dBm, 50mW=17dBm, 100mW=20dBm
    if(settings.testMode){
      Serial.print("Radio Freq: ");Serial.println(settings.TXfreq, 3);
      Serial.print("Radio Power: ");Serial.println(settings.TXpwr);}}
  //if present, disconnect power or send the radio to sleep mode
  else{
    if(pins.radioEN != pins.nullCont){pinMode(pins.radioEN, OUTPUT);digitalWrite(pins.radioEN, LOW);}
    if(sensors.radio != 0){radioSleep();
      digitalWrite(pins.radioCS, HIGH);}
    if(settings.testMode){Serial.println(F("Telemetry OFF!"));}}
    
  //setup servos if enabled
  if(settings.stableRotn || settings.stableVert){

    if(settings.testMode){Serial.println(F("ACTIVE STABILIZATION ENABLED"));}

    //enable serial plotting
    GPSecho = false; radioDebug = false;

    //attach the servos
    canardYaw1.attach(pins.servo1);
    canardYaw2.attach(pins.servo2);
    canardPitch3.attach(pins.servo3);
    canardPitch4.attach(pins.servo4);

    //read the trim settings from EEPROM
    servo1trim = (int8_t)EEPROM.read(eeprom.servo1trim);
    servo2trim = (int8_t)EEPROM.read(eeprom.servo2trim);
    servo3trim = (int8_t)EEPROM.read(eeprom.servo3trim);
    servo4trim = (int8_t)EEPROM.read(eeprom.servo4trim);
    if(settings.testMode){
     Serial.print(F("Servo1 Trim: "));Serial.println(servo1trim);
     Serial.print(F("Servo2 Trim: "));Serial.println(servo2trim);
     Serial.print(F("Servo3 Trim: "));Serial.println(servo3trim);
     Serial.print(F("Servo4 Trim: "));Serial.println(servo4trim);}
     
    //Test canards
    canardYaw1.write(120-servo1trim);
    canardYaw2.write(120-servo2trim);
    canardPitch3.write(120-servo3trim);
    canardPitch4.write(120-servo4trim);
    delay(1000);
    canardYaw1.write(60-servo1trim);
    canardYaw2.write(60-servo2trim);
    canardPitch3.write(60-servo3trim);
    canardPitch4.write(60-servo4trim);
    delay(1000);
    canardYaw1.write(90-servo1trim);
    canardYaw2.write(90-servo2trim);
    canardPitch3.write(90-servo3trim);
    canardPitch4.write(90-servo4trim);}

  //otherwise disable the servos and ensure that stray voltages do not cause any attached servos to move
  else{
    pinMode(pins.servo1, OUTPUT);
    pinMode(pins.servo2, OUTPUT);
    pinMode(pins.servo3, OUTPUT);
    pinMode(pins.servo4, OUTPUT);
    pinMode(pins.servo5, OUTPUT);
    pinMode(pins.servo6, OUTPUT);
    pinMode(pins.servo7, OUTPUT);
    pinMode(pins.servo8, OUTPUT);

    digitalWrite(pins.servo1, LOW);
    digitalWrite(pins.servo2, LOW);
    digitalWrite(pins.servo3, LOW);
    digitalWrite(pins.servo4, LOW);
    digitalWrite(pins.servo5, LOW);
    digitalWrite(pins.servo6, LOW);
    digitalWrite(pins.servo7, LOW);
    digitalWrite(pins.servo8, LOW);}
  
  //signal if in test-mode
  if (settings.testMode){

    Serial.println(F("Signaling Test Mode"));
    beep_counter = 0;
    beep_delay = long_beep_delay;

    //initiate the 7 beeps to signal test mode
    while(beep_counter < 8){
      
      fltTime.tmClock = micros();

      //Look for the user to release the button
      if(digitalRead(pins.testRead) == HIGH){settings.testMode = false;delay(50);}

      //Look for the user to put it into accelerometer calibration mode or barometer calibration mode
      if(digitalRead(pins.testRead) == LOW && !settings.testMode){
        delay(50);//necessary because sometimes when the button is released it bounces back
        if(digitalRead(pins.testRead) == LOW){
          settings.calibrationMode = true;
          settings.testMode = true;
          beep_counter = 8;
          digitalWrite(pins.beep, LOW);
          beep = false;}}

      //Look for the user to enter barometer calibration mode by entering into the serial monitor
      if(Serial.available() > 0){
        userInput = Serial.read();
        if(userInput=='b'){baroCalibrate();}
        if(userInput=='c'){setCanardTrim();}
        while(Serial.available() > 0){userInput = Serial.read();}}

      //starts the beep
      if (!beep && fltTime.tmClock - timeLastBeep > beep_delay){
          digitalWrite(pins.beep, HIGH);
          timeBeepStart = fltTime.tmClock;
          beep = true;
          beep_counter++;}
      
      //stops the beep
      if(beep && (fltTime.tmClock - timeBeepStart > 500000UL)){
        digitalWrite(pins.beep, LOW);
        timeBeepStart = 0UL;
        timeLastBeep = fltTime.tmClock;
        beep = false;}
      }//end while

    //Reset variables
    beep_counter = 0;
    timeBeepStart = 0UL;
    timeLastBeep = 0UL;
    fltTime.tmClock = 0UL;
    if(!settings.calibrationMode){settings.testMode = true;}}//end testMode

  //calibration mode
  if(settings.calibrationMode){accelCalibrate();}//end calibration mode

  //Calibrate the magnetometer
  if(magCalibrateMode == 1){magCalibrate();}
  //set the trigger for the magnetic switch
  magTrigger = 3000;

  //read the bias from EEPROM  
  calUnion.calByte[0]=EEPROM.read(eeprom.accelBiasX); calUnion.calByte[1]=EEPROM.read(eeprom.accelBiasX+1);
  accel.biasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.accelBiasY); calUnion.calByte[1]=EEPROM.read(eeprom.accelBiasY+1);
  accel.biasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.accelBiasZ); calUnion.calByte[1]=EEPROM.read(eeprom.accelBiasZ+1);
  accel.biasZ = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.highGbiasX); calUnion.calByte[1]=EEPROM.read(eeprom.highGbiasX+1);
  highG.biasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.highGbiasY); calUnion.calByte[1]=EEPROM.read(eeprom.highGbiasY+1);
  highG.biasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.highGbiasZ); calUnion.calByte[1]=EEPROM.read(eeprom.highGbiasZ+1);
  highG.biasZ = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.magBiasX); calUnion.calByte[1]=EEPROM.read(eeprom.magBiasX+1);
  mag.biasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.magBiasY); calUnion.calByte[1]=EEPROM.read(eeprom.magBiasY+1);
  mag.biasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(eeprom.magBiasZ); calUnion.calByte[1]=EEPROM.read(eeprom.magBiasZ+1);
  mag.biasZ = calUnion.calValue;
  for(byte i = 0; i < 4; i++){floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.baroPressOffset +i);}
  baro.pressOffset = floatUnion.val;
  for(byte i = 0; i < 4; i++){floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.baroTempOffset +i);}
  baro.tempOffset = floatUnion.val;
  
  //If the barometric pressure sensor calibration values have never been written, then set the offsets to zero
  byte unusedBytes = 0;
  for(byte i = 0; i < 4; i++){if( (byte)EEPROM.read(eeprom.baroPressOffset+i) == (byte)255){unusedBytes++;}}
  if(unusedBytes >= 4){
    if(settings.testMode){Serial.println(F("Warning: Barometer is not calibrated!"));}
    baro.pressOffset = 0.0F;
    baro.tempOffset = 0.0F;}

  //If the accelerometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for(byte i = 0; i < 2; i++){if( (byte)EEPROM.read(eeprom.accelBiasX+i) == (byte)255){unusedBytes++;}}
  if(unusedBytes >= 2){
    if(settings.testMode){Serial.println(F("Warning: Accelerometer is not calibrated!"));}
    accel.biasX = accel.biasY = accel.biasZ = 0;
    accel.dirX = accel.dirY = accel.dirZ = gyro.dirX = gyro.dirY = gyro.dirZ = 1;
    accel.orientX = 'X'; accel.orientY = 'Y'; accel.orientZ = 'Z';
    gyro.orientX = 'X'; gyro.orientY = 'Y'; gyro.orientZ = 'Z';}

  //If the High-G accelerometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for(byte i = 0; i < 2; i++){if( (byte)EEPROM.read(eeprom.accelBiasX+i) == (byte)255){unusedBytes++;}}
  if(unusedBytes >= 2 && sensors.highG != 0){
    if(settings.testMode){Serial.println(F("Warning: High-G Accelerometer is not calibrated!"));}
    highG.biasX = highG.biasY = highG.biasZ = 0;}

  //If the magnetometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for(byte i = 0; i < 2; i++){if( (byte)EEPROM.read(eeprom.magBiasX+i) == (byte)255){unusedBytes++;}}
  if(unusedBytes >= 2){
    if(settings.testMode){Serial.println(F("Warning: Magnetometer is not calibrated!"));}
    mag.biasX = mag.biasY = mag.biasZ = 0;}
    
  //output the read values
  if(settings.testMode){
    Serial.println(F("Calibrataion values read from EEPROM:"));
    Serial.print(F("accel.biasX: "));Serial.println(accel.biasX);
    Serial.print(F("accel.biasY: "));Serial.println(accel.biasY);
    Serial.print(F("accel.biasZ: "));Serial.println(accel.biasZ);
    Serial.print(F("highG.biasX: "));Serial.println(highG.biasX);
    Serial.print(F("highG.biasY: "));Serial.println(highG.biasY);
    Serial.print(F("highG.biasZ: "));Serial.println(highG.biasZ);
    Serial.print(F("mag.biasX: "));Serial.println(mag.biasX);
    Serial.print(F("mag.biasY: "));Serial.println(mag.biasY);
    Serial.print(F("mag.biasZ: "));Serial.println(mag.biasZ);
    Serial.print(F("baro.tempOffset: "));Serial.println(baro.tempOffset, 1);
    Serial.print(F("baro.pressOffset: "));Serial.println(baro.pressOffset, 2);}

  //read the orientation variables from EEPROM
  readOrientation();
  
  //display orientation values from EEPROM
  if(settings.testMode){
    Serial.print(F("accel.X is pointed to real world: "));Serial.print((accel.dirX == 1) ? '+' : '-');Serial.println(accel.orientX);
    Serial.print(F("accel.Y is pointed to real world: "));Serial.print((accel.dirY == 1) ? '+' : '-');Serial.println(accel.orientY);
    Serial.print(F("accel.Z is pointed to real world: "));Serial.print((accel.dirZ == 1) ? '+' : '-');Serial.println(accel.orientZ);
    Serial.print(F("gyro.X is pointed to real world: "));Serial.print((gyro.dirX == 1) ? '+' : '-');Serial.println(gyro.orientX);
    Serial.print(F("gyro.Y is pointed to real world: "));Serial.print((gyro.dirY == 1) ? '+' : '-');Serial.println(gyro.orientY);
    Serial.print(F("gyro.Z is pointed to real world: "));Serial.print((gyro.dirZ == 1) ? '+' : '-');Serial.println(gyro.orientZ);
    Serial.print(F("mag.X is pointed to real world: "));Serial.print((mag.dirX == 1) ? '+' : '-');Serial.println(mag.orientX);
    Serial.print(F("mag.Y is pointed to real world: "));Serial.print((mag.dirY == 1) ? '+' : '-');Serial.println(mag.orientY);
    Serial.print(F("mag.Z is pointed to real world: "));Serial.print((mag.dirZ == 1) ? '+' : '-');Serial.println(mag.orientZ);
    Serial.print(F("highG.X is pointed to real world: "));Serial.print((highG.dirX == 1) ? '+' : '-');Serial.println(highG.orientX);
    Serial.print(F("highG.Y is pointed to real world: "));Serial.print((highG.dirY == 1) ? '+' : '-');Serial.println(highG.orientY);
    Serial.print(F("highG.Z is pointed to real world: "));Serial.print((highG.dirZ == 1) ? '+' : '-');Serial.println(highG.orientZ);}

  //if the ADS1115 is present then restart at the higher rate
  if(sensors.highG == 1){
    if(settings.testMode){Serial.println(F("Restarting ADS1115 at high rate"));}
    beginADS1115('F');
    //we need to multiply the gain again because we just reset it
    highG.gainX *= 9.80665;
    highG.gainY *= 9.80665;
    highG.gainZ *= 9.80665;}

  //Overrides for bench test mode   
  if(settings.testMode){
    if(settings.TXenable){
      settings.TXpwr = 2;
      setRadioPWR(settings.TXpwr);//lowest power setting
      Serial.print(F("Radio Power Reduced for Bench Test Mode: "));
      Serial.println(settings.TXpwr);}
    fltTime.detectLiftoffTime = 10000UL; //0.01s
    settings.setupTime = 3000UL; //3s startup time
    settings.apogeeDelay = 1000000UL; //1s apogee delay
    settings.rcdTime = 15000000UL; //15s record time
    if(settings.stableRotn || settings.stableVert){settings.rcdTime = 30000000UL;}//30s of record time for serial plotting
    gTrigger = (int)(1.5*g); //1.5G trigger
    baro.maxAlt = 11101/unitConvert;
    maxVelocity = 202/unitConvert;
    pktInterval.postFlight = 1000000UL;   
    thresholdVel = 15.5F;
    clearRailTime = 20000UL;
    settings.magSwitchEnable = false;}

  //Create and open the next file on the SD card
  createNextFileSD();

  //check continuity
  checkPyroContinuity();

  //report continuity results
  if(settings.testMode){
    Serial.print("Pyro4 Continuity: ");Serial.println((pyro4.contStatus) ? "Y" : "N");
    Serial.print("Pyro3 Continuity: ");Serial.println((pyro3.contStatus) ? "Y" : "N");
    Serial.print("Pyro2 Continuity: ");Serial.println((pyro2.contStatus) ? "Y" : "N");
    Serial.print("Pyro1 Continuity: ");Serial.println((pyro1.contStatus) ? "Y" : "N");
    Serial.print(F("Reporting continuity code: "));Serial.println(cont.beepCode);}
  
  //set the beep delay and preflight beep code
  beep_delay = long_beep_delay;
  beepCode = cont.beepCode;
  
  //if the magnetic switch is enabled, beep the continuity code until the magnet is sensed
  if(settings.magSwitchEnable){
    delay(250);
    uint8_t ii = 0;
    bool magDetect = false;
    //clear out the FIFO
    for(uint8_t i = 0; i < 10; i++){
      getMag();
      delay(100);}
    while(!magDetect){
      ii=0;
      if(cont.error){
        //signal the continuity error alarm
        for(uint8_t i = 0; i < 20; i++){
          digitalWrite(pins.beep, HIGH);
          delay(12);
          digitalWrite(pins.beep, LOW);
          delay(13);}
          delay(250);}
      while(!magDetect && ii < beepCode){
        digitalWrite(pins.beep, HIGH);
        delay(250);
        digitalWrite(pins.beep, LOW);
        delay(250);
        getMag();
        if(abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger){magDetect = true;}
        ii++;}//end inner loop
      ii=0;
      getMag();
      checkPyroContinuity();
      if(abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger){magDetect = true;}
      while(!magDetect && ii < 10){delay(100);ii++;}
      }//end outer loop
  }//end if magswitch

  if(settings.testMode){Serial.println(F("Hold Rocket Vertical"));}
  //wait for the rocket to be installed vertically
  digitalWrite(pins.beep, HIGH);
  delay(settings.setupTime);
  digitalWrite(pins.beep, LOW);
  delay(500);

  if(settings.testMode){Serial.println(F("Sampling Sensors"));}
  //sample the sensors for 3 seconds to determine the offsets and initial values
  gyro.biasX = 0;
  gyro.biasY = 0;
  gyro.biasZ = 0;
  getHighG();//clear out the buffer, useful only for the ADXL377 combo
  int16_t accelSamps = 0;
  int16_t gyroSamps = 0;
  int16_t highGsamps = 0;
  int16_t magSamps = 0;
  uint32_t sampTime = 3000000;
  uint32_t calibrationStart = micros();
  bool samplePrint = false;
  while(micros() - calibrationStart < sampTime){
    //accelerometer
    if(micros() - accel.timeLastSamp > accel.timeBtwnSamp){
      getAccel();
      accel.timeLastSamp = micros();
      accel.sumX0 += accel.x;
      accel.sumY0 += accel.y;
      accel.sumZ0 += accel.z;
      accelSamps++;
      samplePrint = true;}
    //gyroscope
    if(micros() - gyro.timeLastSamp > gyro.timeBtwnSamp){
      getGyro();
      gyro.timeLastSamp = micros();
      gyro.sumX0 += gyro.rawX;
      gyro.sumY0 += gyro.rawY;
      gyro.sumZ0 += gyro.rawZ;
      gyroSamps++;}
    //high-G accelerometer
    if(sensors.highG != 0 && micros() - highG.timeLastSamp > highG.timeBtwnSamp){
      getHighG();
      highG.timeLastSamp = micros();
      highG.sumX0 += highG.x;
      highG.sumY0 += highG.y;
      highG.sumZ0 += highG.z;
      highGsamps++;}
    //magnetometer
    if(micros() - mag.timeLastSamp > mag.timeBtwnSamp){
      getMag();
      mag.timeLastSamp = micros();
      mag.sumX0 += mag.x;
      mag.sumY0 += mag.y;
      mag.sumZ0 += mag.z;
      magSamps++;}
    if(settings.testMode && accelSamps%100 == 0 && samplePrint){
      Serial.print("Gyro: ");Serial.print(gyro.rawX);Serial.print(',');Serial.print(gyro.rawY);Serial.print(',');Serial.println(gyro.rawZ);
      Serial.print("Accel: ");Serial.print(accel.x);Serial.print(',');Serial.print(accel.y);Serial.print(',');Serial.println(accel.z);
      Serial.print("HighG: ");Serial.print(highG.x);Serial.print(',');Serial.print(highG.y);Serial.print(',');Serial.println(highG.z);
      Serial.print("Mag: ");Serial.print(mag.x);Serial.print(',');Serial.print(mag.y);Serial.print(',');Serial.println(mag.z);
      samplePrint = false;}
  }//end sample period

  //Compute the average of the sample period
  Serial.print("gyro samples: "); Serial.println(gyroSamps);
  Serial.print("accel samples: "); Serial.println(accelSamps);
  Serial.print("highG samples: "); Serial.println(highGsamps);
  Serial.print("mag samples: "); Serial.println(magSamps);
  gyro.biasX = (int)round(gyro.sumX0 / gyroSamps);
  gyro.biasY = (int)round(gyro.sumY0 / gyroSamps);
  gyro.biasZ = (int)round(gyro.sumZ0 / gyroSamps);
  accel.x0 = (int)round(accel.sumX0 / accelSamps);
  accel.y0 = (int)round(accel.sumY0 / accelSamps);
  accel.z0 = (int)round(accel.sumZ0 / accelSamps);
  highG.x0 = (int)round(highG.sumX0 / highGsamps);
  highG.y0 = (int)round(highG.sumY0 / highGsamps);
  highG.z0 = (int)round(highG.sumZ0 / highGsamps);
  mag.x0 = (int)round(mag.sumX0 / magSamps);
  mag.y0 = (int)round(mag.sumY0 / magSamps);
  mag.z0 = (int)round(mag.sumZ0 / magSamps);
  //print out the results of the initial calibration sequence
  if(settings.testMode){
    Serial.println(F("Sampling complete"));
    Serial.print(F("Gyro Offsets: "));
    Serial.print(gyro.biasX);Serial.print(F(", "));
    Serial.print(gyro.biasY);Serial.print(F(", "));
    Serial.println(gyro.biasZ);
    Serial.print(F("Accel X0,Y0,Z0: "));
    Serial.print(accel.x0);Serial.print(F(", "));
    Serial.print(accel.y0);Serial.print(F(", "));
    Serial.println(accel.z0);
    Serial.print(F("HighG X0,Y0,Z0: "));
    Serial.print(highG.x0);Serial.print(F(", "));
    Serial.print(highG.y0);Serial.print(F(", "));
    Serial.println(highG.z0);
    Serial.print(F("Mag X0,Y0,Z0: "));
    Serial.print(mag.x0);Serial.print(F(", "));
    Serial.print(mag.y0);Serial.print(F(", "));
    Serial.println(mag.z0);}

  //Calibrate the analog accelerometer to the digital one
  float A2D = highG.gainZ / accel.gainZ;//.64599
  if(settings.testMode){
    Serial.println(F("Re-calibrating high-G accelerometer..."));
    Serial.print(F("HighG.Z0: "));Serial.println(highG.z0);}
  //Z0 + correction = correctZ0
  //correctZ0 = accel.z0/A2D
  //correction = correctZ0 - Z0
  //corrected bias = original bias - directionX *(highG.z0 - accel.z0/A2D)
  if(highG.orientX == 'Z'){
    if(settings.testMode){Serial.print("Old Bias: ");Serial.println(highG.biasX);}
    highG.biasX -= highG.dirX*(highG.z0 - (int)((float)accel.z0 / (float)A2D));
    if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasX);}}
  if(highG.orientY == 'Z'){
    if(settings.testMode){Serial.print("Old Bias: ");Serial.println(highG.biasY);}
    highG.biasY -= highG.dirY*(highG.z0 - (int)((float)accel.z0 / (float)A2D));
    if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasY);}}
  if(highG.orientZ == 'Z'){
    if(settings.testMode){Serial.print("Old Bias: ");Serial.println(highG.biasZ);}
    highG.biasZ -= highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));
    if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasZ);}}
  //highG.biasX = highGx0 - (int)((float)accel.x0 / (float)A2D) - 27;//old formula is kept for reference
  
  //Compute the acceleromter based rotation angle
  const float rad2deg = 57.29577951308; //degrees per radian
  if (accel.y0 >= 0) {yawY0 = asinf(min(1, (float)accel.y0 / (float)g)) * rad2deg;}
  else {yawY0 = asinf(max(-1, (float)accel.y0 / (float)g)) * rad2deg;}

  if (accel.x0 >= 0) {pitchX0 = asinf(min(1, (float)accel.x0 / (float)g)) * rad2deg;}
  else {pitchX0 = asinf(max(-1, (float)accel.x0 / (float)g)) * rad2deg;}

  //update quaternion rotation
  getQuatRotn(pitchX0*1000000/gyro.gainZ, yawY0*1000000/gyro.gainZ, 0, gyro.gainZ);
  //use DCM2D if we are deploying control surfaces
  if(settings.stableRotn || settings.stableVert){getDCM2DRotn(pitchX0*1000000/gyro.gainZ, yawY0*1000000/gyro.gainZ, 0, gyro.gainZ);}
  //Output the initial rotation conditions reltative to the Earth
  if(settings.testMode){
    Serial.println(F("Rotation Computation Complete"));
    Serial.print(F("Yaw: "));Serial.println(yawY0, 2);
    Serial.print(F("Pitch: "));Serial.println(pitchX0, 2);
    Serial.print(F("Off Vertical: "));Serial.println(((float)offVert)*.1, 1);}

  //Read the battery voltage
  voltReading = analogRead(pins.batt);
  voltage = (float)(voltReading)*3.3*3.2*adcConvert;

  //Read the battery voltage on old hardware if present
  if(pins.batt == pins.pyro4Cont){
    if(settings.pyro1Func == 'M'){pins.batt = pins.pyro1Cont;}
    if(settings.pyro2Func == 'M'){pins.batt = pins.pyro2Cont;}
    if(settings.pyro3Func == 'M'){pins.batt = pins.pyro3Cont;}
    voltReading = analogRead(pins.batt);
    voltage = (float)(voltReading)*3.3*2.72*adcConvert;}
  
  //Read main deploy setting into its beep array
  if(settings.reportStyle == 'P'){
    parseBeep(long(10*int(settings.mainDeployAlt*(unitConvert/10))), maxVelDigits, 4);
    if(settings.testMode){Serial.print(F("Reporting Main Deploy Settings: "));Serial.println((int)(settings.mainDeployAlt*unitConvert));}
    //Beep out the main deployment altitude setting
    while (maxVelDigits[velDigits-1]==0){velDigits--;}  
    for(byte i = velDigits; i > 0; i--){
      delay(800);
      for(byte j = maxVelDigits[i-1]; j > 0; j--){
        digitalWrite(pins.beep, HIGH);
        delay(100);
        digitalWrite(pins.beep, LOW);
        delay(100);}}
    velDigits = 4;
    delay(2000);}

  //check for initial MCU programming
  boolean initPgm = true;
  for(byte i = 0; i < altDigits; i++){if(EEPROM.read(eeprom.maxFltAlt + i) != (byte)255){initPgm = false;}}
  //Write initial values into EEPROM
  if(initPgm){
    if(settings.testMode){Serial.println(F("Initial Use Detected: Writing Initial EEPROM values for prior flight"));}
    for(byte j = 0; j < 6; j++){EEPROM.update(eeprom.maxFltAlt + j,j);}}

  //Beep out the last flight's altitude
  if(settings.reportStyle == 'P'){
    if(settings.testMode){Serial.print(F("Reporting last flight: "));}
    for(byte i=0;i<6;i++){maxAltDigits[i]=EEPROM.read(eeprom.maxFltAlt+i);}
    while (maxAltDigits[altDigits-1]==0 && altDigits > 0){altDigits--;}  
    for(byte i = altDigits; i > 0; i--){
      delay(800);
      if(settings.testMode){
        if(maxAltDigits[i-1] < 10){Serial.print(maxAltDigits[i-1]);}
        else{Serial.print('0');}}
      for(byte j = maxAltDigits[i-1]; j > 0; j--){
        digitalWrite(pins.beep, HIGH);
        delay(100);
        digitalWrite(pins.beep, LOW);
        delay(100);}}
    altDigits = 6;
    delay(2000);}

  //Beep out the battery voltage
  if(settings.reportStyle == 'P'){
    if(settings.testMode){Serial.println(" "); Serial.print(F("Reporting Battery Voltage: "));Serial.println(voltage, 1);}
    parseBeep((int)(voltage*10), voltageDigits, 2);
    for(byte i = 0; i < 2; i++){
      delay(800);
      for(byte j = voltageDigits[1-i]; j > 0; j--){
        digitalWrite(pins.beep, HIGH);
        delay(100);
        digitalWrite(pins.beep, LOW);
        delay(100);}}
    delay(2000);}

  //set the debug ouputs
  switch (settings.serialDebug){
    case 0: GPSecho = false; radioDebug = false; break;
    case 1: GPSecho = true; radioDebug = false; break;
    case 2: GPSecho = false; radioDebug = true; break;
    case 3: GPSecho = true; radioDebug = true; break;}
    
  if(settings.testMode){
    Serial.println(F("Setup Complete.  Awaiting simulated launch."));
    Serial.print(F("Beginning Serial Output and Continuity Reporting: "));
    Serial.println(beepCode);
    delay(3000);}

  //initialize the radio timing
  radioTimer.begin(timerBuildPkt, pktInterval.preLiftoff);
  if(settings.fltProfile == 'B'){radio.event = Booster_Preflight;}
  
}//end setup

int cyclesBtwn = 0;
uint32_t sampleTime = 0UL;
uint32_t sampleStart = 0UL;
uint32_t sampleTimeCheck = 0UL;

void loop(void){

  //debug
  if(settings.testMode){sampleStart = micros();}
  
  //Check if an accelerometer sample is needed and set timestamp
  fltTime.tmClock = sampleTimeCheck = micros();
  if(sampleTimeCheck - accel.timeLastSamp > accel.timeBtwnSamp){
    getAccel();
    accel.timeLastSamp += accel.timeBtwnSamp;
    while(sampleTimeCheck - accel.timeLastSamp > accel.timeBtwnSamp){accel.timeLastSamp += accel.timeBtwnSamp;}}

  //Check if an gyroscope sample is needed
  sampleTimeCheck = micros();
  if(sampleTimeCheck - gyro.timeLastSamp > gyro.timeBtwnSamp){
    getGyro();
    gyro.timeLastSamp += gyro.timeBtwnSamp;
    while(sampleTimeCheck - gyro.timeLastSamp > gyro.timeBtwnSamp){gyro.timeLastSamp += gyro.timeBtwnSamp;}}

  //Check if a high-G accelerometer sample is needed
  sampleTimeCheck = micros();
  if(sampleTimeCheck - highG.timeLastSamp > highG.timeBtwnSamp){
    getHighG();
    highG.timeLastSamp += highG.timeBtwnSamp;
    while(sampleTimeCheck - highG.timeLastSamp > highG.timeBtwnSamp){highG.timeLastSamp += highG.timeBtwnSamp;}}

  //Check if a barometer sample is needed
  sampleTimeCheck = micros();
  if(sampleTimeCheck - baro.timeLastSamp > baro.timeBtwnSamp){
    getBaro();
    //barometers work in single-shot mode so we can't use time-block sampling like the other sensors
    baro.timeLastSamp = sampleTimeCheck;}

  //Check if a magnetometer sample is needed
  sampleTimeCheck = micros();
  if(sampleTimeCheck - mag.timeLastSamp > mag.timeBtwnSamp){
    getMag();
    mag.timeLastSamp += mag.timeBtwnSamp;
    while(sampleTimeCheck - mag.timeLastSamp > mag.timeBtwnSamp){mag.timeLastSamp += mag.timeBtwnSamp;}}
    
  //Sample continuity
  checkPyroContinuity();

  //debug
  if(settings.testMode){sampleTime = micros() - sampleStart;}
  
  //process barometric samples
  //See if a new altitude reading is available
  if(baro.newSamp){processBaroSamp();}

  //look for a shutdown command and if seen, stop all progress for a hard reset
  if (events.preLiftoff && settings.magSwitchEnable){
    getMag();
    n=0;
    if(abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger){n=1;}
    while(n==1){digitalWrite(pins.beep, HIGH);delay(1000);}}

  //detect liftoff
  if (!events.liftoff && accel.z > gTrigger && !events.touchdown && !events.timeOut) {
    if(settings.testMode){Serial.println(' ');Serial.println(F("Simulated Liftoff Detected!"));}
    fltTime.padTime = fltTime.tmClockPrev = fltTime.tmClock;
    events.preLiftoff = false;
    events.liftoff = true;
    events.inFlight = true;
    fltTime.liftoff = fltTime.tmClock;
    radio.event = Liftoff;
    radio.alt = 0;
    radioTimer.begin(timerBuildPkt, pktInterval.inflight/pktInterval.samplesPerPkt);
    noInterrupts();
    buildPkt = true;
    interrupts();
    gnss.liftoff.hour = GPS.time.hour();
    gnss.liftoff.minute = GPS.time.minute();
    gnss.liftoff.second = GPS.time.second();
    gnss.liftoff.mili = GPS.time.centisecond();
    gnss.maxAlt = 0.0F;
    //store base alt in EEPROM
    floatUnion.val = baro.baseAlt;
    if(settings.inflightRecover != 0){
      for(byte i = 0; i<4; i++){EEPROM.update(eeprom.baseAlt + i, floatUnion.Byte[i]);}
      EEPROM.update(eeprom.lastEvent, radio.event);}}

  if (events.liftoff) {

    //capture the cycles between samples
    if(settings.testMode){cyclesBtwn++;}

    //set cycle timestamp
    fltTime.dt = fltTime.tmClock - fltTime.tmClockPrev;
    fltTime.timeCurrent += fltTime.dt;
    fltTime.tmClockPrev = fltTime.tmClock;
    
    //Update the moving average which greatly improves accuracy with minimal latency
    if(highG.newSamp){
      highGsum -= highGfilter[filterPosn];
      highGfilter[filterPosn] = highG.z;
      highGsum += highGfilter[filterPosn];
      filterPosn++;
      if(filterPosn >= sizeHighGfilter){
        filterPosn = 0; 
        if(!filterFull){filterFull = true;}}
      if(!filterFull){highGsmooth = highGsum / filterPosn;}
      else{highGsmooth = highGsum / sizeHighGfilter;}}

    //Compute the current g-load. Use the high-G accelerometer if the IMU is pegged and a high-G accelerometer is present
    if(abs(accel.z) < accel.ADCmax){accelNow = (float)(accel.z - g) * accel.gainZ;}
    else if(sensors.highG != 0){accelNow = (float)(highG.z - high1G) *  highG.gainZ;}
    else{accelNow = (float)(accel.z - g) * accel.gainZ;}

    //Integrate velocity and altitude data prior to apogee
    if(!events.apogeeFire){

      //Capture the max acceleration
      if(accelNow > maxG){maxG = accelNow;}

      //calculate the new acceleration based velocity
      //this makes the apogee event mach immune
      accelVel += accelNow * (float)fltTime.dt * mlnth;
      
      //calculate the new acceleration based altitude
      accelAlt += accelVel * (float)fltTime.dt * mlnth;
      
      //calculate the new sensor fusion based velocity
      fusionVel += accelNow * (float)fltTime.dt * mlnth;
      if(baro.newSamp && baro.Vel < 300 && baro.Vel < baro.maxVel && accelNow < 0.2F && fusionVel < 300.0F && baro.Alt < 13000){
        fusionVel *= 0.99F;
        fusionVel += 0.01F * baro.Vel;}
      radio.vel = (int16_t)fusionVel;
      
      //update maximum velocity if it exceeds the previous value
      if(fusionVel > maxVelocity){maxVelocity = fusionVel;}
    
      //calculate the new sensor fusion based altitude
      fusionAlt += fusionVel * (float)fltTime.dt * mlnth;
      if(baro.newSamp &&  baro.Alt < 9000){
        fusionAlt *= 0.95;
        fusionAlt += 0.05 * baro.Alt;}
      if(!altOK && (fusionAlt > settings.altThreshold || settings.testMode)){altOK = true;}
      radio.alt = (int16_t)fusionAlt;

    }//end if !apogee
    
    //caluclate the partial rotation
    dx += gyro.x * fltTime.dt;
    dy += gyro.y * fltTime.dt;
    dz += gyro.z * fltTime.dt; 

    //if active stablization is on, then use the more stable rotation algorithm
    if(settings.stableRotn || settings.stableVert){
      getDCM2DRotn(dx, dy, dz, gyro.gainZ); 
      dx = dy = dz = 0L;}
    
    //update the quaternion rotation if we are not using active stabilization
    else if(fltTime.timeCurrent - lastRotn > rotnRate){
      getQuatRotn(dx, dy, dz, gyro.gainZ);
      dx = dy = dz = 0L;
      lastRotn = fltTime.timeCurrent;}
 
    //run event logic
    checkEvents();

    //update the canards throws if stabilization or flyback is enabled
  if(settings.stableRotn || settings.stableVert || settings.flyBack){
      uint32_t controlTime = micros();
      if (controlTime - timeLastControl >= controlInterval) {
        //if active stabilization is activated, set the canards
        if((settings.stableVert || settings.stableRotn) && !events.apogee && events.boosterBurnout){setCanards();}
        //if RTB is on and we are post apogee, then set canards to return to launch point
        if(settings.flyBack && events.apogeeFire && !events.mainDeploy){setRTB();}        
        //update control timer
        timeLastControl = controlTime;}}
    
    //Read the battery voltage
    if((fltTime.timeCurrent - lastVolt > voltRate) || pyroFire){
      voltReading = analogRead(pins.batt);
      voltage = (float)(voltReading)*3.3*3.2*adcConvert;
      if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}//on old units we used to pull the battery voltage through one of the pyro channels which had diodes
      writeVolt = true;
      lastVolt = fltTime.timeCurrent;}
        
    //Write the data to a string if we have a new sample
    if(accel.newSamp || gyro.newSamp || highG.newSamp || mag.newSamp || baro.newSamp || baro.newTemp || gnss.SDwrite || writeVolt || SDradioTX){
      writeSDflightData();}
        
    //Close file at Touchdown or Timeout
    if (events.timeOut || events.touchdown) {
      //write the final data to SD card and close
      writeSDfooter();      
      events.liftoff = false;
      //shutoff all pyro outputs
      digitalWrite(pyro1.firePin, LOW);
      digitalWrite(pyro2.firePin, LOW);
      digitalWrite(pyro3.firePin, LOW);
      digitalWrite(pyro4.firePin, LOW);
      //if FHSS, start the postFlight synch packet timer
      if(settings.FHSS){
        syncTimer.begin(timerSyncPkt, pktInterval.postFlight);
        delayMicroseconds(300000UL);}
      //Set the radio transmitter to post-flight data rate
      radioTimer.begin(timerBuildPkt, pktInterval.postFlight);
      //Read max altitude into its beep array
      parseBeep(long(baro.maxAlt*unitConvert), maxAltDigits, 6);
      //Read max velocity into its beep array
      parseBeep(long(maxVelocity*unitConvert), maxVelDigits, 4);
      //set the beeper controls to audibly beep out the maximum altitude and speed
      while (maxAltDigits[altDigits-1]==0){altDigits--;}
      while (maxVelDigits[velDigits-1]==0){velDigits--;}  
      beepPosn=altDigits;
      beepCode=maxAltDigits[beepPosn-1];
      beepAlt = true;
      beepCont = false;
      beepAlarm = false;
      //store the maximum altitude in EEPROM
      if(!settings.testMode){for(byte i=0;i<6;i++){EEPROM.update(eeprom.maxFltAlt+i,maxAltDigits[i]);}}
      //set the final radio variables
      radio.maxAlt = (int16_t)baro.maxAlt;
      radio.maxVel = (int16_t)maxVelocity;
      radio.maxG = (int16_t)((maxG / 0.980665));
      radio.maxGPSalt = (int16_t)gnss.maxAlt;
      //write out the SD card timing variables
      if(settings.testMode){
        Serial.print(F("Max SD Write Time: "));Serial.println(maxWriteTime);
        Serial.print(F("Write Threshold: "));Serial.print(writeThreshold);Serial.print(F(", Count: "));Serial.println(writeThreshCount);}
    }//end of timeout/touchdown protocols    
    
  }//end of liftoff flag

  //Radio packet handling
  noInterrupts();
  //use the interrupt to build the packets at regular intervals
  bool pktFlag = buildPkt;
  interrupts();
  if(settings.TXenable && pktFlag){buildTelmetryPkt();}
  //send the packet when its ready and the radio is available
  if(settings.TXenable && sendPkt && !TX && sensors.statusRadio){sendTelemetryPkt();}

  //Radio Synchronization packet when 915MHz FHSS is used
  if(settings.FHSS && (events.touchdown || events.timeOut)){
    noInterrupts();
    syncFreq = syncFlag;
    interrupts();}

  //Send the sych packet if needed
  if(syncFreq && !TX){syncPkt();}

  //Radio clear interrupts
  noInterrupts();
  bool irqFlag = clearTX;
  interrupts();
  if(irqFlag){clearTXdone();}
  
  //pre-flight continuity alarm beep
  if(events.preLiftoff && settings.reportStyle != 'M' && beepAlarm && !beep && fltTime.tmClock - timeLastBeep > beep_delay){
    digitalWrite(pins.beep, HIGH);
    beep = true;
    beep_len = alarmBeepLen;
    beep_delay = alarmBeepDelay;
    timeBeepStart = fltTime.tmClock;
    beep_counter++;
    //if the counter reaches the beepCode, then switch to error pyro channel
    if(beep_counter == beepCode){
      beep_counter = 0;
      //cycle to the error pyro channel
      beepCont = true;
      beepAlarm = false;
      beepCode = cont.beepCode;
      beep_delay = long_beep_delay;}}

  //pre-flight continuity beep
  if(events.preLiftoff && beepCont && !beep && fltTime.tmClock - timeLastBeep > beep_delay){
    digitalWrite(pins.beep, HIGH);
    beep = true;
    beep_len = medBeepLen;
    beep_delay = short_beep_delay;
    timeBeepStart = fltTime.tmClock;
    beep_counter++;
    //if the counter reaches the beepCode, then pause for a long delay
    if(beep_counter == beepCode){
      beep_counter = 0;
      beep_delay = long_beep_delay;
      //cycle to the alarm if there is a continuity error
      if(cont.error && settings.reportStyle != 'M'){
        beepAlarm = true; 
        beepCont = false;
        beepCode = 50;
        beep_delay = long_beep_delay;}}}
    
  //post-flight max velocity beeping
  if(fileClose && beepVel && !beep && fltTime.tmClock - timeLastBeep > beep_delay){
    if(!settings.silentMode){digitalWrite(pins.beep, HIGH);}
    beep = true;
    beep_len = medBeepLen;
    timeBeepStart = fltTime.tmClock;
    //decrement the current beep
    beep_counter++;
    //if the beep has hit the current digit, then move to the next digit
    if(beep_counter == beepCode){
      beep_counter = 0;
      beep_delay = long_beep_delay;
      beepCode = maxVelDigits[beepPosn-1];
      if(beepPosn==velDigits){beep_delay = 3000000UL;}
      beepPosn--;
      //if we are at the end of the velocity array, switch to the altitude array
      if(beepPosn == 0){
        beepPosn = altDigits;
        //switch to altitude reporting
        beepVel = false;
        beepAlt = true;}}
     //move to the next velocity digit
     else{beep_delay = short_beep_delay;}
     beep = true;}
    
  //post-flight max altitude and max velocity beeping
  if(fileClose && beepAlt && !beep && fltTime.tmClock - timeLastBeep > beep_delay){
    digitalWrite(pins.beep, HIGH);
    beep = true;
    beep_len = medBeepLen;
    timeBeepStart = fltTime.tmClock;
    //decrement the current beep
    beep_counter++;
    //if the beep has hit the current digit, then move to the next digit
    if(beep_counter == beepCode){
      beep_counter = 0;
      beep_delay = long_beep_delay;
      beepCode = maxAltDigits[beepPosn-1];
      if(beepPosn==altDigits){beep_delay = 3000000UL;}
      beepPosn--;
      //if we are at the end of the velocity array, switch to the altitude array
      if(beepPosn == 0){
        beepPosn = velDigits;
        //switch to velocity reporting
        beepVel = true;
        beepAlt = false;}}
     //move to the next velocity digit
     else{beep_delay = short_beep_delay;}
     beep = true;}

  //Code to stop the beep
  if (beep && (fltTime.tmClock - timeBeepStart > beep_len)) {
    digitalWrite(pins.beep, LOW);
    beep = false;
    timeBeepStart = 0UL;
    timeLastBeep = fltTime.tmClock;}
    
  //GPS Code
  //restore defaults if we go a long time without a lock
  /*if(!configGPSdefaults && micros() - timeLastGPS > 10000000UL){
    restoreGPSdefaults(settings.testMode);
    configGPSdefaults = true; 
    gpsFix = 0;
    fixCount = 0;
    configGPSflight = false;}*/
  //5 seconds after touchdown put GPS into Power Save Mode (PSM)
  if(!gnss.configPwrSave && (events.touchdown || events.timeOut) && micros() - fltTime.touchdown > 5000000UL){GNSSpowerSave();gnss.configPwrSave = true;}
  //Read from serial
  char serialBuffer[512];
  uint16_t serialPosn = 0;
  bool msgRX = false;
  if(HWSERIAL->available() > 0){msgRX = true;}
  while(HWSERIAL->available() > 0){
    char c = HWSERIAL->read();
    GPS.encode(c);
    if(settings.GPSlog && !fileClose){updateGPSlogSD(c);}
    if(settings.testMode && GPSecho &&!events.liftoff){serialBuffer[serialPosn] = c; serialPosn++;}}
  if(settings.testMode && GPSecho && !events.liftoff && msgRX){serialBuffer[serialPosn] = '\0'; Serial.print(serialBuffer); msgRX = false; serialPosn = 0;}
  radio.satNum = GPS.satellites.value();
  if(micros() - gnss.timeLastFix > 2000000UL && !events.postFlight){gnss.fix = 0;}
  //process new GNSS position update
  if (GPS.location.isUpdated() || GPS.altitude.isUpdated()) {
        gnss.timeLastFix = micros();
        gnss.fixCount++;
        gnss.fix = 1;
        gnss.SDwrite = true;
        gnss.alt = (GPS.altitude.meters() - baro.baseAlt);
        radio.GPSalt = (int16_t)gnss.alt;
        //capture max GPS alt
        if(radio.GPSalt > gnss.maxAlt){gnss.maxAlt = radio.GPSalt;}
        gnss.latitude = GPS.location.lat();
        gnss.longitude = GPS.location.lng();
        //configure the GPS if the fix is stable
        if(!gnss.configFlight && gnss.fixCount > 40){
          GNSSconfig();
          gnss.fix = 0;
          gnss.configFlight = true; 
          gnss.configDefaults = false;}
        //capture GPS vertical descent velocity with a moving average of 5 samples
        gnss.altPosn++;
        if(gnss.altPosn >= (byte)(sizeof(gnss.altBuff)/sizeof(gnss.altBuff[0]))){gnss.altPosn = 0;}
        gnss.vel = (GPS.altitude.meters() - gnss.altBuff[gnss.altPosn])/((micros() - gnss.timeBuff[gnss.altPosn])*mlnth);
        gnss.altBuff[gnss.altPosn] = GPS.altitude.meters();
        gnss.timeBuff[gnss.altPosn] = micros();
        //update sensor fusion velocity if descending
        if(events.apogee){fusionVel = 0.9 * gnss.vel + 0.1 * baro.Vel;}
        //capture the GPS takeoff position and correct base altitude
        if(events.preLiftoff){
          if(GPS.altitude.meters() != 0){
            //Correct sea level pressure with running average of 5 samples
            //GPS altitude running average
            gnss.bufferPosn++;
            if(gnss.bufferPosn >= (byte)(sizeof(gnss.avgAlt)/sizeof(gnss.avgAlt[0]))){gnss.bufferPosn = 0;gnss.bufferFull = true;}
            gnss.altSum = gnss.altSum + GPS.altitude.meters() - gnss.avgAlt[gnss.bufferPosn];
            gnss.avgAlt[gnss.bufferPosn] = GPS.altitude.meters();
            gnss.baseAlt = gnss.altSum/(float)(sizeof(gnss.avgAlt)/sizeof(gnss.avgAlt[0]));
            //barometric pressure running average
            pressurePosn++;
            if(pressurePosn >= (byte)(sizeof(pressureAvg5)/sizeof(pressureAvg5[0]))){pressurePosn = 0;}
            pressureSum = pressureSum + baro.pressure - pressureAvg5[pressurePosn];
            pressureAvg5[pressurePosn] = baro.pressure;
            pressureAvg = pressureSum/(float)(sizeof(pressureAvg5)/sizeof(pressureAvg5[0]));
            //sea level correction
            if(gnss.bufferFull){baro.seaLevelPressure = pressureAvg / powf((44330 - gnss.baseAlt)/44330, 5.254861);}}
          gnss.liftoff.latitude = GPS.location.lat();
          gnss.liftoff.longitude = GPS.location.lng();
          gnss.liftoff.year = GPS.date.year();
          gnss.liftoff.month = GPS.date.month();
          gnss.liftoff.day = GPS.date.day();}
        //capture the last GPS position
        if(events.mainDeploy || !events.touchdown || !events.timeOut){
          gnss.touchdown.latitude = GPS.location.lat();
          gnss.touchdown.longitude = GPS.location.lng();
          gnss.touchdown.alt = GPS.altitude.meters();}}

}//end void main loop
  
void parseBeep(long value, byte array[], byte arrayLen){
  bool flag = false;
  for (byte i = arrayLen; i >= 1; i--){
       array[i-1] = byte(value/powf(10,i-1));
       value -= array[i-1]*powf(10,i-1);
       if (!flag && array[i-1] > 0){flag = true;}
       if (flag && array[i-1] == 0){array[i-1] = 10;}}}//end void

void checkPyroContinuity(){

  //check continuity
  pyro1.contStatus = ((digitalRead(pyro1.contPin) == HIGH) ? true : false);
  pyro2.contStatus = ((digitalRead(pyro2.contPin) == HIGH) ? true : false);
  pyro3.contStatus = ((digitalRead(pyro3.contPin) == HIGH) ? true : false);
  pyro4.contStatus = ((digitalRead(pyro4.contPin) == HIGH) ? true : false);

  if(!events.liftoff){
    if(pyro1.contStatus){
      if(pyro1.func == 'M'){cont.main = true;}
      if(pyro1.func == 'A'){cont.apogee = true;}
      if(pyro1.func == 'I'){cont.upperStage = true;}
      if(pyro1.func == 'B'){cont.boosterSep = true;}
      if(pyro1.func == '1'){cont.airStart1 = true;}
      if(pyro1.func == '2'){cont.airStart2 = true;}
      if(pyro1.func == 'N'){cont.noFunc = true;}}
    if(pyro2.contStatus){
      if(pyro2.func == 'M'){cont.main = true;}
      if(pyro2.func == 'A'){cont.apogee = true;}
      if(pyro2.func == 'I'){cont.upperStage = true;}
      if(pyro2.func == 'B'){cont.boosterSep = true;}
      if(pyro2.func == '1'){cont.airStart1 = true;}
      if(pyro2.func == '2'){cont.airStart2 = true;}
      if(pyro2.func == 'N'){cont.noFunc = true;}}
    if(pyro3.contStatus){
      if(pyro3.func == 'M'){cont.main = true;}
      if(pyro3.func == 'A'){cont.apogee = true;}
      if(pyro3.func == 'I'){cont.upperStage = true;}
      if(pyro3.func == 'B'){cont.boosterSep = true;}
      if(pyro3.func == '1'){cont.airStart1 = true;}
      if(pyro3.func == '2'){cont.airStart2 = true;}
      if(pyro3.func == 'N'){cont.noFunc = true;}}
   if(pyro4.contStatus){
      if(pyro4.func == 'M'){cont.main = true;}
      if(pyro4.func == 'A'){cont.apogee = true;}
      if(pyro4.func == 'I'){cont.upperStage = true;}
      if(pyro4.func == 'B'){cont.boosterSep = true;}
      if(pyro4.func == '1'){cont.airStart1 = true;}
      if(pyro4.func == '2'){cont.airStart2 = true;}
      if(pyro4.func == 'N'){cont.noFunc = true;}}

  //if the flight profile is complex, but there is no continuity on BOTH of the extra pyros, then reset to a single stage flight
  if((settings.fltProfile == '2' || settings.fltProfile == 'A') && !cont.upperStage && !cont.boosterSep && !cont.airStart1 && !cont.airStart2){
    if(settings.testMode){Serial.println(F("Complex Pyros Not Detected! Flight Profile set to single stage"));}
    settings.fltProfile = 'S';
    if(pyro1.func == 'I' || pyro1.func == 'B' || pyro1.func == '1' || pyro1.func == '2'){pyro1.func = 'N';}
    if(pyro2.func == 'I' || pyro2.func == 'B' || pyro2.func == '1' || pyro2.func == '2'){pyro2.func = 'N';}
    if(pyro3.func == 'I' || pyro3.func == 'B' || pyro3.func == '1' || pyro3.func == '2'){pyro3.func = 'N';}
    if(pyro4.func == 'I' || pyro4.func == 'B' || pyro4.func == '1' || pyro4.func == '2'){pyro4.func = 'N';}}

  //Look for continuity problems
  if(!pyro1.contStatus && pyro1.func != 'N'){cont.error = true; cont.reportCode = cont.beepCode = 1;}
  if(!pyro2.contStatus && pyro2.func != 'N'){cont.error = true; cont.reportCode = cont.beepCode = 2;}
  if(!pyro3.contStatus && pyro3.func != 'N'){cont.error = true; cont.reportCode = cont.beepCode = 3;}
  if(!pyro4.contStatus && pyro4.func != 'N'){cont.error = true; cont.reportCode = cont.beepCode = 4;}
  if(!pyro1.contStatus && !pyro2.contStatus && !pyro3.contStatus && !pyro4.contStatus){cont.reportCode = 0; cont.beepCode = 1;}
  
  //Report single-stage pre-flight status
  if (!cont.error && (settings.fltProfile == 'S' || settings.fltProfile == 'B')){
    if (cont.main && cont.apogee) {cont.beepCode = 3; cont.reportCode = 9;}
    else if (cont.main){cont.beepCode = 2; cont.reportCode = 8;}
    else if (cont.apogee) {cont.beepCode = 1; cont.reportCode = 7;}
    postFlightCode = 1;}

  //Report two-stage pre-flight status
  if (!cont.error && settings.fltProfile == '2'){
    if (cont.boosterSep && cont.upperStage && cont.apogee && cont.main) {cont.beepCode = 4; cont.reportCode = 6;}
    else if(cont.upperStage && cont.apogee && cont.main){cont.beepCode = 3; cont.reportCode = 5;}}

  //Report airstart pre-flight status
  if (!cont.error && settings.fltProfile == 'A'){
    if (cont.airStart1 && cont.airStart2 && cont.apogee && cont.main) {cont.beepCode = 4; cont.reportCode = 6;}
    else if(cont.airStart1 && cont.apogee && cont.main) {cont.beepCode = 3; cont.reportCode = 5;}}

  //Change if Marsa reporting style is desired
  if(settings.reportStyle == 'M'){
    cont.beepCode = 2;
    if(cont.error){cont.beepCode = 1;}}
  }//end if liftoff
}//end check continuity

void firePyros(char event){
   pyroFire = true;
   if(pyro1.func == event){pins.firePin = pyro1.firePin; digitalWrite(pyro1.firePin, HIGH); pyro1.fireStatus = true; pyro1.fireStart = fltTime.timeCurrent;}
   if(pyro2.func == event){pins.firePin = pyro2.firePin; digitalWrite(pyro2.firePin, HIGH); pyro2.fireStatus = true; pyro2.fireStart = fltTime.timeCurrent;}
   if(pyro3.func == event){pins.firePin = pyro3.firePin; digitalWrite(pyro3.firePin, HIGH); pyro3.fireStatus = true; pyro3.fireStart = fltTime.timeCurrent;}
   if(pyro4.func == event){pins.firePin = pyro4.firePin; digitalWrite(pyro4.firePin, HIGH); pyro4.fireStatus = true; pyro4.fireStart = fltTime.timeCurrent;}}

void pulsePyro(){
  static boolean pyro1fire = false;
  static boolean pyro2fire = false;
  static boolean pyro3fire = false;
  static boolean pyro4fire = false;

  if(pyro1.fireStatus){
    if(pyro1fire){digitalWrite(pyro1.firePin, LOW); pyro1fire = false;}
    else{digitalWrite(pyro1.firePin, HIGH); pyro1fire = true;}}
  if(pyro2.fireStatus){
    if(pyro2fire){digitalWrite(pyro2.firePin, LOW); pyro2fire = false;}
    else{digitalWrite(pyro2.firePin, HIGH); pyro2fire = true;}}
  if(pyro3.fireStatus){
    if(pyro3fire){digitalWrite(pyro3.firePin, LOW); pyro3fire = false;}
    else{digitalWrite(pyro3.firePin, HIGH); pyro3fire = true;}}
  if(pyro4.fireStatus){
    if(pyro4fire){digitalWrite(pyro4.firePin, LOW); pyro4fire = false;}
    else{digitalWrite(pyro4.firePin, HIGH); pyro4fire = true;}}
}

void readEEPROMsettings(){

  //read user settings
  settings.fltProfile = (char)EEPROM.read(eeprom.fltProfile);
  settings.units = (char)EEPROM.read(eeprom.units);
  if(settings.units == 'M'){unitConvert = 1.0F;}
  settings.reportStyle = (char)EEPROM.read(eeprom.reportStyle);
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.setupTime + i);}
  settings.setupTime = ulongUnion.val;
  for(byte i=0; i<4; i++){floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.mainDeployAlt + i);}
  settings.mainDeployAlt = floatUnion.val;
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.apogeeDelay + i);}
  settings.apogeeDelay = ulongUnion.val;
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.rcdTime + i);}
  settings.rcdTime = ulongUnion.val;
  settings.silentMode = (boolean)EEPROM.read(eeprom.silentMode);
  settings.magSwitchEnable = (boolean)EEPROM.read(eeprom.magSwitchEnable);
  settings.inflightRecover = (byte)EEPROM.read(eeprom.inflightRecover);
  settings.GPSlog = (boolean)EEPROM.read(eeprom.gpsLogFile);
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.fireTime + i);}
  settings.fireTime = ulongUnion.val; 
  settings.pyro1Func = (char)EEPROM.read(eeprom.pyro1Func); 
  settings.pyro2Func = (char)EEPROM.read(eeprom.pyro2Func);
  settings.pyro3Func = (char)EEPROM.read(eeprom.pyro3Func);
  settings.pyro4Func = (char)EEPROM.read(eeprom.pyro4Func);
  settings.TXenable = (boolean)EEPROM.read(eeprom.TXenable);
  settings.TXpwr = (byte)EEPROM.read(eeprom.TXpwr);
  for(byte i=0; i<4; i++){floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.TXfreq + i);}
  settings.TXfreq = floatUnion.val; 
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.boosterSeparationDelay + i);}
  settings.FHSS = (boolean)EEPROM.read(eeprom.FHSS);
  settings.boosterSeparationDelay = ulongUnion.val; 
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.sustainerFireDelay + i);}
  settings.sustainerFireDelay = ulongUnion.val; 
  settings.airStart1Event = (char)EEPROM.read(eeprom.airStart1Event);
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.airStart1Delay + i);}
  settings.airStart1Delay = ulongUnion.val; 
  settings.airStart2Event = (char)EEPROM.read(eeprom.airStart2Event);
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.airStart2Delay + i);}
  settings.airStart2Delay = ulongUnion.val;
  for(byte i=0; i<2; i++){intUnion.Byte[i] = (byte)EEPROM.read(eeprom.altThreshold + i);}
  settings.altThreshold = intUnion.val;
  for(byte i=0; i<2; i++){intUnion.Byte[i] = (byte)EEPROM.read(eeprom.maxAngle + i);}
  settings.maxAngle = intUnion.val;
  settings.stableRotn = (boolean)EEPROM.read(eeprom.stableRotn);
  settings.stableVert = (boolean)EEPROM.read(eeprom.stableVert);}

void processBaroSamp(){

  //----------------------------
  //process preliftoff variables
  //----------------------------
  static float sumBaseAlt = 0.0F;
  static byte baseAltPosn = 0;
  static float baseAltBuff[30]  = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  if(events.preLiftoff || events.touchdown || events.timeOut){
    sumBaseAlt -= baseAltBuff[baseAltPosn];
    sumBaseAlt += baro.rawAlt;
    baseAltBuff[baseAltPosn] = baro.rawAlt;
    baseAltPosn++;
    const byte sizeBaseAltBuff = sizeof(baseAltBuff) / sizeof(baseAltBuff[0]);
    if(baseAltPosn >= sizeBaseAltBuff){baseAltPosn = 0;}        
    baro.baseAlt = sumBaseAlt / sizeBaseAltBuff;
    radio.baseAlt = (int16_t)baro.baseAlt;
    baro.newSamp = false;}
  
  //--------------------------
  //Update in-flight variables
  //--------------------------
  baro.Alt = baro.rawAlt - baro.baseAlt;
  //update the maximum altitude reporting variable
  if(baro.Alt > baro.maxAlt && !events.apogee){baro.maxAlt = baro.Alt;}
  
  //----------------------------
  //Smoothed barometric altitude
  //----------------------------
  static float rawAltSum = 0.0;
  static float rawAltBuff[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static byte rawAltPosn = 0;
  //Update the rolling sum
  rawAltSum -= rawAltBuff[rawAltPosn];
  rawAltBuff[rawAltPosn] = baro.Alt;
  rawAltSum += rawAltBuff[rawAltPosn];
  //update the position counter
  rawAltPosn++;
  static const float sizeRawAltBuff = sizeof(rawAltBuff)/sizeof(rawAltBuff[0]);
  if(rawAltPosn >= (byte)sizeRawAltBuff){rawAltPosn = 0;}
  //calculate the smoothed barometric altitude
  baro.smoothAlt = rawAltSum / sizeRawAltBuff;
  
  //---------------------------
  //barometric derived velocity
  //---------------------------
  static byte altAvgPosn = 0;
  static int baroVelPosn = 0;
  static float altAvgBuff[30] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static unsigned long baroTimeBuff[30] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL};
  //if we are past apogee, use the maximum difference between samples in the buffer, otherwise set the difference to 10 samples
  if(events.apogee){baroVelPosn = altAvgPosn;}
  else{
    baroVelPosn = altAvgPosn - 10;
    if(baroVelPosn < 0){baroVelPosn = (int)((sizeof(altAvgBuff)/sizeof(altAvgBuff[0])) - (10 - altAvgPosn));}}
  //calculate the barometric derived velocity from the moving averages
  baro.Vel = (baro.smoothAlt - altAvgBuff[baroVelPosn])/((float)(baro.timeLastSamp - baroTimeBuff[baroVelPosn])*mlnth);
  //update variables
  if(baro.Vel > baro.maxVel){baro.maxVel = baro.Vel;}
  radio.vel = baro.Vel;
  radio.alt = baro.smoothAlt;
  altAvgBuff[altAvgPosn] = baro.smoothAlt;
  baroTimeBuff[altAvgPosn] = baro.timeLastSamp;
  altAvgPosn++;
  if(altAvgPosn >= (byte)(sizeof(altAvgBuff)/sizeof(altAvgBuff[0]))){altAvgPosn = 0;}
  if(events.apogeeFire || settings.testMode){fusionVel = baro.Vel; fusionAlt = baro.smoothAlt;}
  
  //----------------------------
  //Barometric touchdown trigger
  //----------------------------
  (events.mainDeploy && fabs(baro.Vel) < 1.0F ) ? baroTouchdown++ : baroTouchdown = 0;}

void clearIRQ(){clearTX = true;}

void timerBuildPkt(){buildPkt = true;}

void timerSyncPkt(){syncFlag = true;}