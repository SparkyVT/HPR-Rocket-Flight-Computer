//High-Power Rocketry Flight Computer (TeensyFlight)
//Original sketch by Bryan Sparkman, TRA #12111, NAR #85720, L3
//Built for Teensy 3.2, 3.5, 3.6, 4.0, and 4.1
//Code Line Count: 9356 lines of code = 2435 MainFile + 366 Bus_Mgmt + 2194 SensorDrivers + 888 Calibration + 625 SpeedTrig + 505 Inflight_Recover + 637 SD + 398 Rotation + 698 Telemetry + 327 Event_Logic + 283 GPSconfig      
//--------FEATURES----------
//Dual-deploy flight computer capable to over 100,000ft 
//Two-stage & airstart capable with tilt-sensing safety features
//Live telemetry over 433MHz or 915MHz LoRa (433MHz: USA amateur 70cm band, EUR licencse free) (915MHz: USA FHSS licence free or USA amateur license use non-FHSS) 
//4 programmable high-current pyro outputs with continuity checks
//Captures high-rate data at approximately 50,000 samples per second recorded to SD card
//--1000Hz 3-axis digital 16G and 100G accelerometer data logging
//--1000Hz 3-axis digital 2000dps gyroscope data logging
//--1000Hz of flight events & continuity data logging
//--1000Hz of sensor-fuzed speed & altitude
//--100Hz of roll, pitch, yaw rotation
//--40Hz of of magnetic data logging and magnetic roll
//--30Hz-100Hz of digital barometric data logging (Altitude, pressure, temperature)
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
//--User selectable frequency & power settings for telemetry
//--8 configurable servo outputs (4 powered, 4 un-powered)
//--User selectable inflight brownout recovery
//--User selectable active stabilization for roll, pitch, and yaw correction
//--User selectable return-to-pad controlled recovery
//Tilt-sensing lockout for ignition of second stages and/or airstarts
//Mach immune, sensor-fusion based apogee event
//Barometric based main deploy event
//Audible pre-flight continuity report
//Audible Post-flight max altitude & speed report
//Mount in any orientation, automatic orientation detection during calibration
//Bench-test mode activated w/ tactile button; user configurable status messages over USB Serial
//Built-in self-calibration mode 
//Report in SI or Metric units
//Compatible with Teensy 3.2, 3.5, 3.6, 4.0, 4.1
//--Compatible with many different sensors over I2C or SPI
//--Configurable GPIO pins and hardware I2C/SPI bus options
//--GPS can be wired to any available HW Serial port (Ublox M6, M7, M8, M9 supported)
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
//V4_5_0 eliminates RadioHead library due to interferences with the PWMServo library, uses an interval timer to control the radio packet timing, fixes bugs in the telemetry timestamps
//-------FUTURE UPGRADES----------
//Active Stabilization (started)
//Return-to-Base capability (started)
//3D position physics model
//Remote Arm & Shutdown Commands over LoRa
//Ground Station Bluetooth Datalink to smartphone
//Smartphone App
//------TO DO LIST------
//Bench test on inflight recovery routines
//Flight test airstart code
//stuck-in-a-loop detection and breakout
//-------CODE START--------
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <PWMServo.h>
#include <SPI.h>
//Teensy 3.X
#if defined (__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MK20DX256__)
  //i2c_t3 is needed to enable the different pin options of bus0
  #include <i2c_t3.h>
  //NOTE: typedef declarations must be in the main file and not the tabs
  typedef i2c_t3 TwoWire;
#endif
//Teensy 4.X
#if defined (__IMXRT1062__)
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
IntervalTimer radioTimer;

//GLOBAL VARIABLES
//-----------------------------------------
//Set code version
//-----------------------------------------
const float codeVersion = 4.5;
//-----------------------------------------
//EEPROM allocation
//-----------------------------------------
struct{
    int maxFltAlt = 0;//00-05: maximum altitude of last flight
    int accelBiasX = 6;//06-11: accel.bias(X,Y,Z)
    int accelBiasY = 8;
    int accelBiasZ = 10;
    int highGbiasX = 12;//12-17: highG.bias(X,Y,Z)
    int highGbiasY = 14;
    int highGbiasZ = 16;
    int magBiasX = 18;//18-23: mag.bias(X,Y,Z)
    int magBiasY = 20;
    int magBiasZ = 22;
    int accelXsign = 24;//24-29: accelerometer to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    int accelXptr = 25;
    int accelYsign= 26;
    int accelYptr = 27;
    int accelZsign = 28;
    int accelZptr = 29;
    int gyroXsign = 30;//30-35: gyro to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    int gyroXptr = 31;
    int gyroYsign= 32;
    int gyroYptr = 33;
    int gyroZsign = 34;
    int gyroZptr = 35;
    int magXsign = 36;//36-41: magnetometer to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    int magXptr = 37;
    int magYsign= 38;
    int magYptr = 39;
    int magZsign = 40;
    int magZptr = 41;
    int highGxSign = 42;//42-47: highG to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    int highGxPtr = 43;
    int highGySign = 44;
    int highGyPtr = 45;
    int highGzSign = 46;
    int highGzPtr = 47;
    int accel2boardXaxis = 48;//48-53: accelerometer to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
    int accel2boardXsign = 49;
    int accel2boardYaxis = 50;
    int accel2boardYsign = 51;
    int accel2boardZaxis = 52;
    int accel2boardZsign = 53;
    int gyro2boardXaxis = 54;//54-59: gyro to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
    int gyro2boardXsign = 55;
    int gyro2boardYaxis = 56;
    int gyro2boardYsign = 57;
    int gyro2boardZaxis = 58;
    int gyro2boardZsign = 59;
    int mag2boardXaxis = 60;//60-65: magnetometer to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
    int mag2boardXsign = 61;
    int mag2boardYaxis = 62;
    int mag2boardYsign = 63;
    int mag2boardZaxis = 64;
    int mag2boardZsign = 65;
    int highG2boardXaxis = 66;//66-71: high-G accelerometer to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
    int highG2boardXsign = 67;
    int highG2boardYaxis = 68;
    int highG2boardYsign = 69;
    int highG2boardZaxis = 70;
    int highG2boardZsign = 71;
    int accelBusType = 72;//72-93: device bus info
    int accelBusNum = 73;
    int gyroBusType = 74;
    int gyroBusNum = 75;
    int magBusType = 76;
    int magBusNum = 77;
    int highGBusType = 78;
    int highGBusNum = 79;
    int baroPressOffset = 80;//80-83 : barometer pressure offset, stays in this position so I don't have to recalibrate all my flight computers
    int baroTempOffset = 84;//84-87  : barometer temperature offset, stays in this position so I don't have to recalibrate all my flight computers
    int baroBusType = 88;
    int baroBusNum = 89;
    int radioBusType = 90;
    int radioBusNum = 91;
    int sdBusType = 92;
    int sdBusNum = 93;
    int gpsBusType = 94;
    int gpsBusNum = 95;
    int pyro1ContPin = 96;//96     : Pyro 1 Continuity Pin
    int pyro1FirePin = 97;//97     : Pyro 1 Fire Pin
    int pyro2ContPin = 98;//98     : Pyro 2 Continuity Pin
    int pyro2FirePin = 99;//98     : Pyro 2 Fire Pin
    int pyro3ContPin = 100;//100   : Pyro 3 Continuity Pin
    int pyro3FirePin = 101;//101   : Pyro 3 Fire Pin
    int pyro4ContPin = 102;//102   : Pyro 4 Continuity Pin
    int pyro4FirePin = 103;//103   : Pyro 4 Fire Pin
    int nullPin = 104;//104        : Null Continuity / Fire Pin
    int beepPin = 105;//105        : Beeper Pin
    int battReadPin = 106;//106    : Battery Read Pin
    int testModeGndPin = 107;//107 : Test Mode Button Gnd Pin
    int testModeRdPin= 108;//108   : Test Mode Read Pin
    int radioCSpin = 109;//109     : radio CS pin
    int radioIRQpin = 110;//110    : radio interrupt pin
    int radioRstPin = 111;//111    : radio reset pin
    int radioEnPin = 112;//112     : radio enable pin (optional w/ Adafruit breakout only)
    int accelCSpin = 113;
    int gyroCSpin = 114;
    int magCSpin = 115;
    int highGcsPin = 116;
    int sdCSpin = 117;
    int baroCSpin = 118;
    int servo1pin = 119;//119   : servo 1 control pin
    int servo2pin = 120;//120   : servo 2 control pin
    int servo3pin = 121;//121   : servo 3 control pin
    int servo4pin = 122;//122   : servo 4 control pin
    int servo5pin = 123;//123   : servo 5 control pin
    int servo6pin = 124;//124   : servo 6 control pin
    int servo7pin = 125;//125   : servo 7 control pin
    int servo8pin = 126;//126   : servo 8 control pin
    int accelID = 127;//127   : Sensor ID accel    
    int gyroID = 128;//128   : Sensor ID gyro
    int highGID = 129;//129   : Sensor ID highG
    int baroID = 130;//130   : Sensor ID baro
    int radioID = 131;//131   : Sensor ID Radio
    int GPSID = 132;//132   : Sensor ID GPS
    int magID = 133;
    int sdID = 134;
    int HWversion = 135;
    int HWsubVersion = 136;
    int HWunitNum = 137;
    int callSign = 138;//138-143: Ham Radio Callsign
    //-------------------------------------------------------------
    //flight settings stored in EEPROM to facilitate rapid recovery
    //-------------------------------------------------------------
    int rocketName = 144; //Maximum of 20 characters
    int fltProfile = 164;
    int units = 165;
    int reportStyle = 166;
    int setupTime = 167;//4
    int gTrigger = 171;//2
    int detectLiftoffTime = 173;//4
    int mainDeployAlt = 177;//2
    int rcdTime = 179;//4
    int apogeeDelay = 183;
    int silentMode = 184;
    int magSwitchEnable = 185;
    int inflightRecover = 186;
    int fireTime = 187;//4
    int pyro4Func = 191;
    int pyro3Func = 192;
    int pyro2Func = 193;
    int pyro1Func = 194;
    int TXenable = 195;
    int TXpwr = 196;
    int TXfreq = 197;//4
    int FHSS = 201;
    int boosterSeparationDelay = 202;//4
    int sustainerFireDelay = 206;//4
    int airStart1Event = 210;
    int airStart1Delay = 211;//4
    int airStart2Event = 215;
    int airStart2Delay = 216;//2
    int altThreshold = 218;//2
    int maxAngle = 220;
    int stableRotn = 221;
    int stableVert = 222;
    int flyback = 223;
    int serialDebug = 224;
    int lastFile = 225;//2
    int baseAlt = 227;//4
    int lastEvent = 231;
    //-------------------------------------------------------------
    //active stabilization parameters
    //-------------------------------------------------------------
    int servo1trim = 232;
    int servo2trim = 233;
    int servo3trim = 234;
    int servo4trim = 235;
    int servo5trim = 236;
    int servo6trim = 237;
    int servo7trim = 238;
    int servo8trim = 239;
    int Kp = 240;//4
    int Ki = 244;//4
    int Kd = 248;//4
} eeprom;
//-----------------------------------------
//Sensor device data
//-----------------------------------------
struct{
  byte accel = 0;
  byte mag = 0;
  byte gyro = 0;
  byte highG = 0;
  byte baro = 0;
  byte radio = 0;
  byte GPS = 0;
  char accelBusType;
  byte accelBusNum;
  char gyroBusType;
  byte gyroBusNum;
  char magBusType;
  byte magBusNum;
  char baroBusType;
  byte baroBusNum;
  char highGBusType;
  byte highGBusNum;
  char radioBusType;
  byte radioBusNum;
  char gpsBusType;
  byte gpsBusNum;
  char sdBusType;
  byte sdBusNum;
  boolean status_LSM303 = false;
  boolean status_LSM9DS1 = false;
  boolean status_LSM6DS33 = false;
  boolean status_L3GD20H = false;
  boolean status_LIS3MDL = false;
  boolean status_MS5611 = false;
  boolean status_MS5607 = false;
  boolean status_MPL3115A2 = false;
  boolean status_BMP180 = false;
  boolean status_BMP280 = false;
  boolean status_BMP388 = false;
  boolean status_H3LIS331DL = false;
  boolean status_ADS1115 = false;
  boolean status_ADXL377 = false;
  boolean status_RFM96W = false;
  boolean status_NoHighGAccel = false;
  boolean pyroPWM = false;
} sensors;
//-----------------------------------------
//Sensor variables
//-----------------------------------------
byte rawData[12];
typedef struct{
  byte addr;
  int16_t ADCmax;
  float gainX;
  float gainY;
  float gainZ;
  int16_t rawX;
  int16_t rawY;
  int16_t rawZ;
  int8_t dirX;
  int8_t dirY;
  int8_t dirZ;
  char orientX;
  char orientY;
  char orientZ;
  int16_t *ptrX;
  int16_t *ptrY;
  int16_t *ptrZ;
  int8_t *ptrXsign;
  int8_t *ptrYsign;
  int8_t *ptrZsign;
  int32_t sumX0 = 0L;
  int32_t sumY0 = 0L;
  int32_t sumZ0 = 0L;
  int16_t x0;
  int16_t y0;
  int16_t z0;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t biasX;
  int16_t biasY;
  int16_t biasZ;
  uint32_t timeLastSamp = 0UL;
  uint32_t timeBtwnSamp = 10000UL;
  boolean newSamp = false;
} sensorData;
sensorData accel;
sensorData mag;
sensorData gyro; 
sensorData highG;
//-----------------------------------------
//Set defaults for user defined variables
//-----------------------------------------
struct {
  boolean testMode = false;
  boolean calibrationMode = false;
  //--------flight settings----------------
  char rocketName[20] = ""; //Maximum of 20 characters
  char HWid[7] = "";
  char fltProfile = 'S';
  char units = 'S';
  char reportStyle = 'P';
  unsigned long setupTime = 5000000UL;
  int gTrigger = 3415; //2.5G trigger
  unsigned long detectLiftoffTime = 500000UL; //0.5s
  float mainDeployAlt = 153;//Up to 458m for main deploy
  unsigned long rcdTime = 900000000UL; //15min
  unsigned long apogeeDelay = 1000000UL; //1.0s apogee delay
  boolean silentMode = false; //true turns off beeper
  boolean magSwitchEnable = false;
  byte inflightRecover = 0;
  byte serialDebug = 1;
  //--------pyro settings----------------
  unsigned long fireTime = 500000UL;//0.5s
  char pyro4Func = 'M';
  char pyro3Func = 'A';
  char pyro2Func = 'N';
  char pyro1Func = 'N';
  //--------telemetry settings----------------
  char callSign[7]= ""; 
  boolean TXenable = true;//false turns off radio transmissions
  byte TXpwr = 13;
  float TXfreq = 433.250;
  boolean FHSS = false;
  //--------2 stage settings----------------
  unsigned long boosterSeparationDelay = 500000UL; //0.5s
  unsigned long sustainerFireDelay = 1000000UL; //1.0s
  //--------airstart settings----------------
  char airStart1Event = 'B';
  unsigned long airStart1Delay = 500000UL;
  char airStart2Event = 'B';
  unsigned long airStart2Delay = 500000UL;
  //-------safety thresholds----------------
  int altThreshold = 120; //120m = 400ft
  int maxAngle = 45; //degrees
  //-------active stabilization----------------
  boolean stableRotn = false;
  boolean stableVert = false;  
  //----------flyback option--------------------
  boolean flyBack = false;
} settings;
//-----------------------------------------
//GPIO pin mapping
//-----------------------------------------
struct{
  byte nullCont;
  byte nullFire;
  byte beep;
  byte testGnd;
  byte testRead;
  byte mag;
  byte batt;
  byte pyro1Cont;
  byte pyro1Fire;
  byte pyro2Cont;
  byte pyro2Fire;
  byte pyro3Cont;
  byte pyro3Fire;
  byte pyro4Cont;
  byte pyro4Fire;
  byte radioCS;
  byte radioIRQ;
  byte radioRST;
  byte radioEN;
  byte servo1;
  byte servo2;
  byte servo3;
  byte servo4;
  byte servo5;
  byte servo6;
  byte servo7;
  byte servo8;
  byte accelCS;
  byte gyroCS;
  byte magCS;
  byte highG_CS;
  byte baroCS;
  byte SD_CS;
} pins;
byte firePin = 0;
//-----------------------------------------
//pyro variables
//-----------------------------------------
typedef struct{
  char func;
  boolean fireStatus = false;
  boolean contStatus = false;
  unsigned long fireStart;
  byte contPin;
  byte firePin;
} pyroMap;
pyroMap pyro1;
pyroMap pyro2;
pyroMap pyro3;
pyroMap pyro4;
//-----------------------------------------
//radio variables
//-----------------------------------------
byte dataPacket[77];
boolean liftoffSync = false;
unsigned long TXstartTime;
boolean syncFreq = false;
byte pktPosn=0;
uint16_t sampNum = 0;
byte packetSamples = 4;
boolean gpsTransmit = false;
unsigned long RIpreLiftoff = 1000000UL;
unsigned long RIinflight = 50000UL;
unsigned long RIpostFlight = 10000000UL;
unsigned long RIsyncOffset = 0UL;
unsigned long radioInterval = 0UL;;
boolean TX = false;
boolean SDradioTX = false;
uint8_t radioMode;
volatile boolean clearTX = false;
volatile boolean sendPkt = false;
struct{
  int16_t packetnum = 0;
  uint8_t event = 0;
  uint16_t fltTime = 0;
  int16_t baseAlt = 0;
  int16_t alt;
  int16_t accel;
  int16_t vel = 0;
  int16_t roll;
  int16_t offVert;
  int16_t maxAlt;
  int16_t maxVel;
  int16_t maxGPSalt;
  int16_t maxG;
  int16_t GPSalt;
  int16_t satNum = 0;
} radio;
union {
   float GPScoord; 
   byte GPSbyte[4];
} GPSlongitude;
union {
   float GPScoord; 
   byte GPSbyte[4];
} GPSlatitude;
//-----------------------------------------
//flight events
//-----------------------------------------
typedef struct{
  boolean preLiftoff = true;
  boolean inFlight = false;
  boolean postFlight = false;
  boolean liftoff = false;
  boolean falseLiftoffCheck = true;
  boolean boosterBurnout = false;
  boolean boosterBurnoutCheck = false;
  boolean boosterSeparation = false;
  boolean sustainerFireCheck = false;
  boolean sustainerFire = false;
  boolean sustainerIgnition = false;
  boolean sustainerBurnout = false;
  boolean airStart1Check = false;
  boolean airStart1Fire = false;
  boolean airStart1Ignition = false;
  boolean airStart1BurnoutCheck = false;
  boolean airStart1Burnout = false;
  boolean airStart2Check = false;
  boolean airStart2Fire = false;
  boolean airStart2Ignition = false;
  boolean airStart2BurnoutCheck = false;
  boolean airStart2Burnout = false;
  boolean apogee = false;
  boolean apogeeFire = false;
  boolean apogeeSeparation = false;
  boolean mainDeploy = false;
  boolean touchdown = false;
  boolean timeOut = false;
  } eventList;
eventList events;
eventList resetEvents;
//-----------------------------------------
//Master timing variables
//-----------------------------------------
typedef struct{
  unsigned long liftoff = 0UL;
  unsigned long boosterBurnout = 0UL;
  unsigned long boosterSeparation = 0UL;
  unsigned long sustainerFireCheck = 0UL;
  unsigned long sustainerFire = 0UL;
  unsigned long sustainerIgnition = 0UL;
  unsigned long sustainerBurnout = 0UL;
  unsigned long airStart1Check = 0UL;
  unsigned long airStart1Fire = 0UL;
  unsigned long airStart1Ignition = 0UL;
  unsigned long airStart1BurnoutCheck = 0UL;
  unsigned long airStart1Burnout = 0UL;
  unsigned long airStart2Check = 0UL;
  unsigned long airStart2Fire = 0UL;
  unsigned long airStart2Ignition = 0UL;
  unsigned long airStart2BurnoutCheck = 0UL;
  unsigned long airStart2Burnout = 0UL;
  unsigned long apogee = 0UL;
  unsigned long apogeeFire = 0UL;
  unsigned long apogeeSeparation = 0UL;
  unsigned long mainDeploy = 0UL;
  unsigned long touchdown = 0UL;
  unsigned long padTime = 0UL;
  unsigned long tmClock = 0UL;
  unsigned long tmClockPrev = 0UL;
  unsigned long timeCurrent = 0UL;
  unsigned long dt = 0UL;
} timerList;
timerList fltTime;
timerList resetFltTime;
//-----------------------------------------
//continuity Booleans
//-----------------------------------------
struct{
  boolean apogee;
  boolean main;
  boolean boosterSep;
  boolean upperStage;
  boolean airStart1;
  boolean airStart2;
  boolean noFunc;
  boolean error;
  byte reportCode;
  byte beepCode;
  } cont;
//-----------------------------------------
//Non-Event Booleans
//-----------------------------------------
boolean rotnOK = true;
boolean altOK = false;
boolean beep = false;
boolean pyroFire = false;
boolean fileClose = false;
unsigned long boosterBurpTime = 1000000UL;
float unitConvert = 3.2808F;
//-----------------------------------------
//digital accelerometer variables
//-----------------------------------------
int g = 1366;
float accelNow;
float maxG = 0.0;
//-----------------------------------------
//High-G accelerometer variables
//-----------------------------------------
boolean startADXL377 = false;
int high1G = 63;
boolean filterFull = false;
long highGsum = 0L;
int highGfilter[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte sizeHighGfilter = 30;
byte filterPosn = 0;
float highGsmooth;
union {
   int16_t calValue; 
   byte calByte[2];
} calUnion;
//-----------------------------------------
//Altitude & Baro Sensor variables
//-----------------------------------------
struct{
  float rawAlt = 0.0F;
  float Alt = 0.0F;
  float baseAlt = 10.0F;
  float maxAlt = 0.0F;
  float smoothAlt = 0.0F;
  float Vel = 0.0F;
  float maxVel = 0.0F;
  float pressure;
  float temperature;
  unsigned long timeLastSamp = 0UL;
  unsigned long timeBtwnSamp = 50000UL;
  boolean newSamp = false;
  boolean newTemp = false;
  float seaLevelPressure = 1013.25;
  float tempOffset;
  float pressOffset;
} baro; 
//-----------------------------------------
//Baro Reporting Variables
//-----------------------------------------
byte baroTouchdown = 0;
byte touchdownTrigger = 5;
//-----------------------------------------
//Magnetometer Variables
//-----------------------------------------
int16_t magTrigger = 0;
byte magCalibrateMode = 0;
long magRoll = 0UL;
int magOffVert = 0;
int magPitch = 0;
int magYaw = 0;
//-----------------------------------------
//gyro & rotation variables
//-----------------------------------------
long dx = 0L;
long dy = 0L;
long dz = 0L;
float yawY0;
float pitchX0;
int pitchX;
int yawY;
long rollZ = 0;
const float degRad = 57.29577951308; //degrees per radian
const float mlnth = 0.000001;
int offVert = 0;
boolean rotationFault = false;
//-----------------------------------------
//Quaternion Integration variables
//-----------------------------------------
float Quat[5] = {0.0, 1.0, 0.0, 0.0, 0.0};
float ddx;
float ddy;
float ddz;
unsigned long lastRotn = 0UL;
unsigned long rotnRate = 10000UL;//100 updates per second
//-----------------------------------------
//velocity calculation variables
//-----------------------------------------
float accelVel = 0.0F;
float accelAlt = 0.0F;
float maxVelocity = 0.0F;
float fusionVel = 0.0F;
float fusionAlt = 0.0F;
float thresholdVel = 44.3F;
//-----------------------------------------
//beeper variables
//-----------------------------------------
byte beep_counter = 0;
byte beepPosn = 0;
unsigned long beep_delay = 100000UL;
int beepCode = 0;
unsigned long beep_len = 100000UL;
unsigned long timeBeepStart;
unsigned long timeLastBeep;
boolean beepAlt = false;
boolean beepVel = false;
boolean beepCont = true;
boolean beepAlarm = false;
const unsigned long short_beep_delay = 100000UL;
const unsigned long long_beep_delay = 800000UL;
const unsigned long alarmBeepDelay = 10000UL;
const unsigned long alarmBeepLen = 10000UL;
const unsigned long medBeepLen = 100000UL;
const unsigned long shortBeepLen = 10000UL;
//-----------------------------------------
//SD card writing variables
//-----------------------------------------
uint32_t writeStart;
uint32_t writeTime;
uint32_t maxWriteTime;
uint32_t writeThreshold = 10000UL;
uint32_t writeThreshCount = 0UL;
int strPosn = 0;
boolean syncApogee = false;
boolean syncMains = false;
const byte decPts = 2;
const byte base = 10;
char dataString[1024];
byte maxAltDigits[6];
byte maxVelDigits[4];
byte voltageDigits[2];
byte altDigits = 6;
byte velDigits = 4;
byte n = 1;
float voltage = 0.0F;
boolean writeVolt = false;
unsigned long voltRate = 33333UL;
uint16_t voltReading;
unsigned long lastVolt = 0UL;
boolean reportCode = true;//true = report max altitude, false = report max velocity
byte postFlightCode = 0;
float adcConvert = 0.000015259;
const char cs = ',';
//-----------------------------------------
//GPS Variables
//-----------------------------------------
float maxGPSalt = 0.0;
float baseGPSalt = 0.0;
float GPSavgAlt[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float GPSaltBuff[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
uint32_t GPStimeBuff[5] = {0UL, 0UL, 0UL, 0UL, 0UL};
float GPSaltSum = 0.0;
byte GPSposn = 0;
byte GPSaltPosn = 0;
boolean GPSnewData = false;
boolean gpsWrite = false;
char liftoffLat = 'N';
char liftoffLon = 'W';
float liftoffLatitude = 0.0;
float liftoffLongitude = 0.0;
int liftoffYear = 0;
byte liftoffMonth = 0;
byte liftoffDay = 0;
byte liftoffHour = 0;
byte liftoffMin = 0;
float liftoffSec = 0.0;
long liftoffMili = 0L;
char touchdownLat = 'N';
char touchdownLon = 'W';
float touchdownLatitude = 0.0;
float touchdownLongitude = 0.0;
float touchdownAlt = 0;
byte touchdownHour = 0;
byte touchdownMin = 0;
float touchdownSec = 0.0;
long touchdownMili = 0L;
byte gpsFix = 0;
float gpsFloat;
float gpsInt;
byte gpsLat;
byte gpsLon;
float gpsLatitude;
float gpsLongitude;
float GPSvel = 0.0F;
boolean configGPSdefaults = true;
boolean configGPSflight = false;
unsigned long timeLastGPS = 0UL;
uint16_t fixCount = 0;
boolean GPSpsm = false;
char serialBuffer[512];
uint16_t serialPosn = 0;
boolean msgRX = false;
boolean GPSbufferFull = false;
float pressureAvg = 0;
float pressureAvg5[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float pressureSum = 0.0;
byte pressurePosn = 0;
//-----------------------------------------
//EEPROM useful unions
//-----------------------------------------
union{
    unsigned long val;
    byte Byte[4];
  } ulongUnion;
union{
    int val;
    byte Byte[2];
  } intUnion;
union{
    float val;
    byte Byte[4];
  } floatUnion;
//-----------------------------------------
//Active Stabilization
//-----------------------------------------
char userInput;
int8_t servo1trim = 0;
int8_t servo2trim = 0;
int8_t servo3trim = 0;
int8_t servo4trim = 0;
int8_t servo5trim = 0;
int8_t servo6trim = 0;
int8_t servo7trim = 0;
int8_t servo8trim = 0;
const uint32_t controlInterval = 50000UL;
uint32_t timeLastControl = 0UL;
float Kp = 0.0F;
float Ki = 0.0F;
float Kd = 0.0F;
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
//-----------------------------------------
//debug
//-----------------------------------------
boolean GPSecho = false;
boolean radioDebug = false;
boolean disp = false;
boolean TXdataFile = true;
//--------------------------------------------
//Radio event codes to eliminate magic numbers
//--------------------------------------------
enum{
  Preflight           = 0,
  Liftoff             = 1,
  Booster_Burnout     = 2,
  Apogee              = 3,
  Fire_Apogee         = 4,
  Separation          = 5,
  Fire_Mains          = 6,
  Under_Chute         = 7,
  Eject_Booster       = 8,
  Fire_Sustainer      = 9,
  Sustainer_Ignition  = 10,
  Sustainer_Burnout   = 11,
  Fire_Airstart1      = 12,
  Airstart1_Ignition  = 13,
  Airstart1_Burnout   = 14,
  Fire_Airstart2      = 15,
  Airstart2_Ignition  = 16,
  Airstart2_Burnout   = 17,
  NoFire_Rotn_Limit   = 18,
  NoFire_Alt_Limit    = 19,
  NoFire_RotnAlt_Limit= 20,
  Booster_Apogee      = 21,
  Fire_Booster_Apogee = 22,
  Booster_Separation  = 23,
  Fire_Booster_Mains  = 24,
  Booster_Under_Chute = 25,
  Time_Limit          = 26,
  Touchdown           = 27,
  Power_Loss_Restart  = 28,
  Booster_Touchdown   = 29,
  Booster_Preflight   = 30,
  Booster_Time_Limit  = 31,
  Booster_Power_Loss  = 32};
//--------------------------------------------
//Radio operating modes (RegOpMode)
//--------------------------------------------
enum {
  SleepMode     =  0x00,
  StandbyMode   =  0x01,
  TXmode        =  0x03,
  RXmode        =  0x05,
  CADmode       =  0x07,
  LoRaMode      =  0x80, 
  LowFrqMode    =  0x08, //LowFrq mode enables registers below 860MHz
  writeMask     =  0x80};
byte radioFnctn = SleepMode;

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
  byte lastEvent = EEPROM.read(eeprom.lastEvent);
  if(lastEvent != 27){
    EEPROM.update(eeprom.lastEvent, 27);
    lastEvent = 27;}
  if(lastEvent == (byte)255){
    lastEvent = 27;
    EEPROM.write(eeprom.lastEvent, lastEvent);}
  settings.inflightRecover = EEPROM.read(eeprom.inflightRecover);
  if(settings.inflightRecover > 0 && (lastEvent < 24 && lastEvent != 6 && lastEvent != 7)){
    
    //execute the rapid recovery routine
    Serial.print("Rapid Reset Initiated, setting: ");Serial.println(settings.inflightRecover);
    if(rapidReset()){
      //exit setup and immediately re-enter the loop
      return;}}
  
  //Start SD card
  beginSD();
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
  sensors.GPS = EEPROM.read(eeprom.GPSID);
  sensors.accelBusType = (char)EEPROM.read(eeprom.accelBusType);
  sensors.gyroBusType = (char)EEPROM.read(eeprom.gyroBusType);
  sensors.magBusType = (char)EEPROM.read(eeprom.magBusType);
  sensors.highGBusType = (char)EEPROM.read(eeprom.highGBusType);
  sensors.baroBusType = (char)EEPROM.read(eeprom.baroBusType);
  sensors.sdBusType = (char)EEPROM.read(eeprom.sdBusType);
  sensors.gpsBusType = (char)EEPROM.read(eeprom.gpsBusType);
  sensors.radioBusType = (char)EEPROM.read(eeprom.radioBusType);
  sensors.accelBusNum = EEPROM.read(eeprom.accelBusNum);
  sensors.gyroBusNum = EEPROM.read(eeprom.gyroBusNum);
  sensors.magBusNum = EEPROM.read(eeprom.magBusNum);
  sensors.highGBusNum = EEPROM.read(eeprom.highGBusNum);
  sensors.baroBusNum = EEPROM.read(eeprom.baroBusNum);
  sensors.sdBusNum = EEPROM.read(eeprom.sdBusNum);
  sensors.gpsBusNum = EEPROM.read(eeprom.gpsBusNum);
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
  //Set the pyro firing pins to LOW for safety
  digitalWrite(pins.pyro1Fire, LOW);
  digitalWrite(pins.pyro2Fire, LOW);
  digitalWrite(pins.pyro3Fire, LOW);
  digitalWrite(pins.pyro4Fire, LOW);
  //Set the device CS pins to HIGH
  digitalWrite(pins.highG_CS, HIGH);
  Serial.println("Set Pins");

  //Start Harware Serial communication
  setHWSERIAL();
  if(sensors.GPS == 3){HWSERIAL->begin(38400);Serial.println("Starting HWSerial at 38400 baud");}
  else{HWSERIAL->begin(9600);Serial.println("Starting HWSerial at 9600 baud");}
  
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
  analogReadResolution(16);

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
  //433MHz Radio
  if(sensors.status_RFM96W && settings.testMode && sensors.radio == 1){Serial.println(F("Starting 433MHz"));}
  //915MHz Radio
  if(sensors.status_RFM96W && settings.testMode && sensors.radio == 2){Serial.println(F("Starting 915MHz"));}
  //start the radio
  sensors.status_RFM96W = radioBegin(pins.radioRST);
  if(sensors.status_RFM96W){Serial.println(F("RFM96W/95W OK!"));}
  else{Serial.println(F("RFM96W/95W not found!"));}
  
  //read the flight settings from the SD card
  readFlightSettingsSD();
  
  //Update the EEPROM with the new settings
  EEPROM.update(eeprom.fltProfile, settings.fltProfile);
  EEPROM.update(eeprom.units, settings.units);
  EEPROM.update(eeprom.reportStyle, settings.reportStyle);
  ulongUnion.val = settings.setupTime;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.setupTime + i, ulongUnion.Byte[i]);}
  intUnion.val = settings.gTrigger;
  for(byte i=0; i<2; i++){EEPROM.update(eeprom.gTrigger + i, intUnion.Byte[i]);}
  ulongUnion.val =  settings.detectLiftoffTime;
  for(byte i=0; i<2; i++){EEPROM.update(eeprom.detectLiftoffTime + i, ulongUnion.Byte[i]);}
  floatUnion.val = settings.mainDeployAlt;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.mainDeployAlt + i, floatUnion.Byte[i]);}
  ulongUnion.val = settings.apogeeDelay;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.apogeeDelay + i, ulongUnion.Byte[i]);}
  ulongUnion.val = settings.rcdTime;
  for(byte i=0; i<4; i++){EEPROM.update(eeprom.rcdTime + i, ulongUnion.Byte[i]);}
  EEPROM.update(eeprom.silentMode, settings.silentMode);
  EEPROM.update(eeprom.magSwitchEnable, settings.magSwitchEnable);
  //ulongUnion.val = settings.rotnCalcRate;
  //for(byte i=0; i++; i<4){EEPROM.update(eeprom.rotnCalcRate + i, ulongUnion.Byte[i]);}
  EEPROM.update(eeprom.inflightRecover, settings.inflightRecover);
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
  
  //safety override of user settings
  if (settings.gTrigger < 1.5 * g) {settings.gTrigger = 1.5 * g;} //min 1.5G trigger
  if (settings.gTrigger > 5 * g) {settings.gTrigger = 5 * g;} //max 5G trigger
  if (settings.detectLiftoffTime < 100000UL) {settings.detectLiftoffTime = 100000UL;} //.1s min gTrigger detection
  if (settings.detectLiftoffTime > 1000000UL) {settings.detectLiftoffTime = 1000000UL;} //1s max gTrigger detection
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
  if (settings.TXpwr > 20){settings.TXpwr = 20;}
  if (settings.TXpwr < 2){settings.TXpwr = 2;}
  if(settings.fltProfile == '2' or settings.fltProfile == 'A'){boosterBurpTime = min(1000000UL, settings.boosterSeparationDelay-10000UL);}
  
  //check for silent mode
  if(settings.testMode && settings.silentMode){pins.beep = pins.nullCont; Serial.println(F("Silent Mode Confirmed"));}

  //if telemetry is disabled and silent mode is on, then use the LED to signal the beeps
  if(settings.testMode && settings.silentMode && !settings.TXenable){pins.beep = 13; pinMode(pins.beep, OUTPUT);}

  //setup the radio
  radioInterval = RIpreLiftoff;
  if(settings.TXenable){
    //915MHz FHSS
    if (sensors.radio == 2 && settings.FHSS){
      settings.TXfreq = 902.300; 
      RIpreLiftoff = 600000UL;
      if(settings.testMode){Serial.println("FHSS Active!");}}//sync freq
    //915MHz dedicated frequency (no FHSS)
    if (sensors.radio == 2 && !settings.FHSS){
      setRadioPWR(2);//minimum power due to FCC regulations
      if(settings.testMode){Serial.println(F("Dedicated ISM band frequency. Power limit 2mW"));}}
    //Set the radio output power & frequency
    setRadioFreq(settings.TXfreq);
    setRadioPWR(settings.TXpwr);//23 max setting; 20mW=13dBm, 30mW=15dBm, 50mW=17dBm, 100mW=20dBm
    if(settings.testMode){
      Serial.print("Radio Freq: ");Serial.println(settings.TXfreq, 3);
      Serial.print("Radio Power: ");Serial.println(settings.TXpwr);}}
  else{
    if(pins.radioEN != pins.nullCont){pinMode(pins.radioEN, OUTPUT);digitalWrite(pins.radioEN, LOW);}
    digitalWrite(pins.radioCS, HIGH);
    radioSetMode(SleepMode);
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

  //Decrease magnetometer gain to eliminate false positives if the magnetic switch is enabled
  if(settings.magSwitchEnable == 1){resetMagGain();}

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
    accel.biasX = accel.biasY = accel.biasZ = 0;}

  //If the High-G accelerometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for(byte i = 0; i < 2; i++){if( (byte)EEPROM.read(eeprom.accelBiasX+i) == (byte)255){unusedBytes++;}}
  if(unusedBytes >= 2 && !sensors.status_NoHighGAccel){
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
  if(sensors.status_ADS1115 && settings.testMode){
    Serial.println(F("Restarting ADS1115 at high rate"));
    beginADS1115('F');}

  //Overrides for bench test mode
  if(settings.testMode){
    settings.TXpwr = 2;
    setRadioPWR(settings.TXpwr);//lowest power setting
    Serial.print(F("Radio Power Reduced for Bench Test Mode: "));
    Serial.println(settings.TXpwr);
    settings.detectLiftoffTime = 10000UL; //0.01s
    settings.setupTime = 3000UL; //3s startup time
    settings.apogeeDelay = 1000000UL; //1s apogee delay
    settings.rcdTime = 15000000UL; //15s record time
    if(settings.stableRotn || settings.stableVert){settings.rcdTime = 30000000UL;}//30s of record time for serial plotting
    settings.gTrigger = (int)(1.5*g); //1.5G trigger
    baro.maxAlt = 11101/unitConvert;
    maxVelocity = 202/unitConvert;
    RIpostFlight = 1000000UL;
    thresholdVel = 15.5F;
    settings.magSwitchEnable = false;}

  // Rename the data file to FLIGHT01.txt
  dataString[0] ='F';
  dataString[1] ='L';
  dataString[2] ='I';
  dataString[3] ='G';
  dataString[4] ='H';
  dataString[5] ='T';
  dataString[6] ='0';
  dataString[7] ='1';
  dataString[8] ='.';
  dataString[9] ='t';
  dataString[10]='x';
  dataString[11]='t';
  dataString[12]='\0';

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
    resetMagGain();
    delay(250);
    byte ii = 0;
    boolean magDetect = false;
    //clear out the FIFO
    for(byte i = 0; i < 10; i++){
      getMag();
      delay(100);}
    while(!magDetect){
      ii=0;
      if(cont.error){
        //signal the continuity error alarm
        for(byte i = 0; i < 20; i++){
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
  //sample the sensors 4000 times over 3 seconds to determine the offsets and initial values
  gyro.biasX = 0;
  gyro.biasY = 0;
  gyro.biasZ = 0;
  //clear out the buffer
  getHighG();
  int sampSize = 4000;
  int magSamps = 0;
  uint32_t sampDelay = 714 * (1-0.8);//20% of avg cycle time
  for (int i = 0; i < sampSize; i++) { 
    //get a sensor event
    getAccel();
    getGyro();
    getHighG();
    if(micros() - mag.timeLastSamp > mag.timeBtwnSamp){
      getMag();
      mag.timeLastSamp = micros();
      //add up the magnetometer samples
      mag.sumX0 += mag.x;
      mag.sumY0 += mag.y;
      mag.sumZ0 += mag.z;
      magSamps++;}
    if(settings.testMode && i%100 == 0){
      Serial.print("Gyro: ");Serial.print(gyro.rawX);Serial.print(',');Serial.print(gyro.rawY);Serial.print(',');Serial.println(gyro.rawZ);
      Serial.print("Accel: ");Serial.print(accel.x);Serial.print(',');Serial.print(accel.y);Serial.print(',');Serial.println(accel.z);
      Serial.print("HighG: ");Serial.print(highG.x);Serial.print(',');Serial.print(highG.y);Serial.print(',');Serial.println(highG.z);
      Serial.print("Mag: ");Serial.print(mag.x);Serial.print(',');Serial.print(mag.y);Serial.print(',');Serial.println(mag.z);}
    
    //add up the gyro samples
    gyro.sumX0 += gyro.rawX;
    gyro.sumY0 += gyro.rawY;
    gyro.sumZ0 += gyro.rawZ;

    //add up the accelerometer samples
    accel.sumX0 += accel.x;
    accel.sumY0 += accel.y;
    accel.sumZ0 += accel.z;   

    //add up the analog accelerometer samples
    highG.sumX0 += highG.x;
    highG.sumY0 += highG.y;
    highG.sumZ0 += highG.z;
        
    //sample over a period of 3 seconds
    delayMicroseconds(sampDelay);}

  //Divide by 100 to set the average of 100 samples
  gyro.biasX = round(gyro.sumX0 / sampSize);
  gyro.biasY = round(gyro.sumY0 / sampSize);
  gyro.biasZ = round(gyro.sumZ0 / sampSize);
  accel.x0 = (int)round(accel.sumX0 / sampSize);
  accel.y0 = (int)round(accel.sumY0 / sampSize);
  accel.z0 = (int)round(accel.sumZ0 / sampSize);
  highG.x0 = (int)round(highG.sumX0 / sampSize);
  highG.y0 = (int)round(highG.sumY0 / sampSize);
  highG.z0 = (int)round(highG.sumZ0 / sampSize);
  mag.x0 = (int)round(mag.sumX0 / magSamps);
  mag.y0 = (int)round(mag.sumY0 / magSamps);
  mag.z0 = (int)round(mag.sumZ0 / magSamps);
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
  if(highG.orientX == 'Z'){
    //Z0 + correction = correctZ0
    //correctZ0 = accel.z0/A2D
    //correction = correctZ0 - Z0
    if(settings.testMode){Serial.print("Old Bias: ");Serial.println(highG.biasX);}
    highG.biasX -= highG.dirX*(highG.z0 - (int)((float)accel.z0 / (float)A2D));
    if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasX);}}
  if(highG.orientY == 'Z'){
    if(settings.testMode){Serial.print("Old Bias: ");Serial.println(highG.biasY);}
    highG.biasY -= highG.dirY*(highG.z0 - (int)((float)accel.z0 / (float)A2D));
    if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasY);}}
  if(highG.orientZ == 'Z'){
    if(settings.testMode){Serial.print("Old Bias: ");Serial.println(highG.biasZ);}
    highG.biasZ -= highG.dirZ*((int)(highG.z0 - (float)accel.z0 / (float)A2D));
    if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasZ);}}
  //highG.biasX = highGx0 - (int)((float)accel.x0 / (float)A2D) - 27;//old formula is kept for reference
  
  //Compute the acceleromter based rotation angle
  if (accel.y0 >= 0) {yawY0 = asin(min(1, (float)accel.y0 / (float)g)) * degRad;}
  else {yawY0 = asin(max(-1, (float)accel.y0 / (float)g)) * degRad;}

  if (accel.x0 >= 0) {pitchX0 = asin(min(1, (float)accel.x0 / (float)g)) * degRad;}
  else {pitchX0 = asin(max(-1, (float)accel.x0 / (float)g)) * degRad;}

  //update quaternion rotation
  getQuatRotn(pitchX0/degRad, yawY0/degRad, 0);
  if(settings.stableRotn || settings.stableVert){
    pitchX = (int)(pitchX0*10);
    yawY = (int)(yawY0*10);}
  if(settings.testMode){
    Serial.println(F("Rotation Computation Complete"));
    Serial.print(F("Yaw: "));Serial.println(yawY0, 2);
    Serial.print(F("Pitch: "));Serial.println(pitchX0, 2);
    Serial.print(F("Off Vertical: "));Serial.println(((float)offVert)/((float)10), 2);}  

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
  
  //Reset the G-trigger
  if(accel.orientX == 'Z'){settings.gTrigger -= accel.dirX*accel.biasX;}
  if(accel.orientY == 'Z'){settings.gTrigger -= accel.dirY*accel.biasY;}
  if(accel.orientZ == 'Z'){settings.gTrigger -= accel.dirZ*accel.biasZ;}
  
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

  //check for initial MPU programming
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
  radioTimer.begin(timerSendPkt, RIpreLiftoff);
  if(settings.fltProfile == 'B'){radio.event = Booster_Preflight;}
  
}//end setup

int cyclesBtwn = 0;
uint32_t sampleTime = 0UL;
uint32_t sampleStart = 0UL;

void loop(void){

  //debug
  if(settings.testMode){sampleStart = micros();}
  
  //Sample the Accelerometer
  if(micros()-accel.timeLastSamp > accel.timeBtwnSamp){
    getAccel();
    accel.timeLastSamp = micros();}
  
  //Sample the Gyroscope
  if(micros()-gyro.timeLastSamp > gyro.timeBtwnSamp){
    getGyro();
    gyro.timeLastSamp = micros();}

  //Sample the High-G accelerometer
  if(micros()-highG.timeLastSamp >= highG.timeBtwnSamp){
    getHighG();
    highG.timeLastSamp = micros();}
  
  //Sample the Barometric Pressure Sensor
  if(micros()-baro.timeLastSamp >= baro.timeBtwnSamp){
    getBaro();
    baro.timeLastSamp = micros();}

  //Sample the Magnetometer
  if(micros()-mag.timeLastSamp > mag.timeBtwnSamp){
    getMag();
    mag.timeLastSamp = micros();}

  //Sample continuity
  checkPyroContinuity();
  sampleTime = micros() - sampleStart;
  
  //process barometric samples
  //See if a new altitude reading is available
  if(baro.newSamp){processBaroSamp();}

  //look for a shutdown command and if seen, stop all progress for a hard reset
  if (events.preLiftoff && settings.magSwitchEnable){
    getMag();
    n=0;
    if(abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger){n=1;}
    while(n==1){digitalWrite(pins.beep, HIGH);delay(1000);}}

  //set timestamp
  fltTime.tmClock = micros();

  //detect liftoff
  if (!events.liftoff && accel.z > settings.gTrigger && !events.touchdown && !events.timeOut) {
    if(settings.testMode){Serial.println(' ');Serial.println(F("Simulated Liftoff Detected!"));}
    fltTime.padTime = fltTime.tmClockPrev = fltTime.tmClock;
    events.preLiftoff = false;
    events.liftoff = true;
    events.inFlight = true;
    fltTime.liftoff = fltTime.tmClock;
    radio.event = 1;
    radio.alt = 0;
    radioTimer.begin(timerSendPkt, RIinflight);
    noInterrupts();
    sendPkt = true;
    interrupts();
    liftoffHour = GPS.time.hour();
    liftoffMin = GPS.time.minute();
    liftoffSec = GPS.time.second();
    liftoffMili = GPS.time.centisecond();
    maxGPSalt = 0.0F;
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
    else if(!sensors.status_NoHighGAccel){accelNow = (highGsmooth - (float)high1G) *  highG.gainZ;}
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
      if(baro.newSamp && baro.Vel < 300 && baro.Vel < baro.maxVel && accelNow < 0.2F && fusionVel < 300.0F && baro.Alt < 9000){
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

    //if active stablization is on, then use the faster algorithm
    if(settings.stableRotn || settings.stableVert){
      getDCM2DRotn(dx, dy, dz); 
      dx = dy = dz = 0L;
      uint32_t controlTime = micros();
      if (controlTime - timeLastControl >= controlInterval) {
        disp=true;
        setCanards();
        timeLastControl = controlTime;}}
    
    //update the quaternion rotation
    else if(fltTime.timeCurrent - lastRotn > rotnRate){
      
      const float rotn2rad = gyro.gainZ * (3.14159265359 / 180) / 1000000;
      ddx = dx * rotn2rad;
      ddy = dy * rotn2rad;
      ddz = dz * rotn2rad;
      
      getQuatRotn( ddx, ddy, ddz);
      dx = dy = dz = 0L;
      
      lastRotn = fltTime.timeCurrent;}
 
    //run event logic
    checkEvents();

    //if RTB is on and we are post apogee, then set canards to return to launch point
    if(events.apogeeFire && !events.mainDeploy && settings.flyBack){setRTB();}
    
    //Read the battery voltage
    if((fltTime.timeCurrent - lastVolt > voltRate) || pyroFire){
      voltReading = analogRead(pins.batt);
      voltage = (float)(voltReading)*3.3*3.2*adcConvert;
      if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}
      writeVolt = true;
      lastVolt = fltTime.timeCurrent;}
        
    //Write the data to a string if we have a new sample
    if(accel.newSamp || gyro.newSamp || highG.newSamp || mag.newSamp || baro.newSamp || baro.newTemp || gpsWrite || writeVolt || SDradioTX){
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
      //Set the radio transmitter to post-flight data rate
      radioTimer.begin(timerSendPkt, RIpostFlight);
      //Read max altitude into its beep array
      parseBeep(long(baro.maxAlt*unitConvert), maxAltDigits, 6);
      //Read max velocity into its beep array
      parseBeep(long(maxVelocity*unitConvert), maxVelDigits, 4);
      //reset n, which we'll use to cycle through the reporting digits
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
      radio.maxG = (int)((maxG / 0.980665));
      radio.maxGPSalt = (int)maxGPSalt;
      //write out the SD card timing variables
      if(settings.testMode){
        Serial.print(F("Max SD Write Time: "));Serial.println(maxWriteTime);
        Serial.print(F("Write Threshold: "));Serial.print(writeThreshold);Serial.print(F(", Count: "));Serial.println(writeThreshCount);}
      //center canards
      if(settings.stableVert || settings.stableRotn){
        canardYaw1.write(90-servo1trim);
        canardYaw2.write(90-servo2trim);
        canardPitch3.write(90-servo3trim);
        canardPitch4.write(90-servo4trim);}
    }//end of timeout/touchdown protocols    
    
  }//end of liftoff flag

  //Radio packet handling
  noInterrupts();
  boolean pktFlag = sendPkt;
  interrupts();
  if(settings.TXenable && pktFlag && !syncFreq){radioSendPacket();}

  //Radio Synchronization packet when 915MHz FHSS is used
  if(syncFreq){syncPkt();}

  //Radio clear interrupts
  noInterrupts();
  boolean irqFlag = clearTX;
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
    
  //post-flight max altitude beeping
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
  /*if(!configGPSdefaults && micros() - timeLastGPS > 10000000UL){
    restoreGPSdefaults();
    configGPSdefaults = true; 
    gpsFix = 0;
    fixCount = 0;
    configGPSflight = false;}*/
  //5 seconds after touchdown put GPS into Power Save Mode (PSM)
  if( !GPSpsm && (events.touchdown || events.timeOut) && micros() - fltTime.touchdown > 5000000UL){GPSpowerSaveMode();GPSpsm = true;}
  //Read from serial
  if(HWSERIAL->available() > 0){msgRX = true;}
  while(HWSERIAL->available() > 0){
    char c = HWSERIAL->read();
    GPS.encode(c);
    if(settings.testMode && GPSecho &&!events.liftoff){serialBuffer[serialPosn] = c; serialPosn++;}}
  if(settings.testMode && GPSecho && !events.liftoff && msgRX){serialBuffer[serialPosn] = '\0'; Serial.print(serialBuffer); msgRX = false; serialPosn = 0;}
    
  radio.satNum = GPS.satellites.value();
  if(micros() - timeLastGPS > 2000000UL && !events.postFlight){gpsFix = 0;}
  if (GPS.location.isUpdated() || GPS.altitude.isUpdated()) {
        timeLastGPS = micros();
        fixCount++;
        gpsFix = 1;
        if(!configGPSflight && fixCount > 40){
          configGPS();
          gpsFix = 0;
          configGPSflight = true; 
          configGPSdefaults = false;}
        gpsWrite = true;
        gpsTransmit = true;
        convertLocation();
        radio.GPSalt = (int16_t)(GPS.altitude.meters() - baro.baseAlt);
        GPSlatitude.GPScoord = gpsLatitude;
        GPSlongitude.GPScoord = gpsLongitude;
        //capture GPS velocity
        GPSaltPosn++;
        if(GPSaltPosn >= (byte)(sizeof(GPSaltBuff)/sizeof(GPSaltBuff[0]))){GPSaltPosn = 0;}
        GPSvel = (GPS.altitude.meters() - GPSaltBuff[GPSaltPosn])/((micros() - GPStimeBuff[GPSaltPosn])*mlnth);
        GPSaltBuff[GPSaltPosn] = GPS.altitude.meters();
        GPStimeBuff[GPSaltPosn] = micros();
        //update sensor fusion velocity
        if(events.apogee){fusionVel = 0.9 * GPSvel + 0.1 * baro.Vel;}
        //capture max GPS alt
        if(GPS.altitude.meters() - baro.baseAlt > maxGPSalt){maxGPSalt = GPS.altitude.meters() - baro.baseAlt;}
        //capture the GPS takeoff position and correct base altitude
        if(events.preLiftoff){
          if(GPS.altitude.meters() != 0){
            //Correct sea level pressure with running average of 5 samples
            //GPS altitude running average
            GPSposn++;
            if(GPSposn >= (byte)(sizeof(GPSavgAlt)/sizeof(GPSavgAlt[0]))){GPSposn = 0;GPSbufferFull = true;}
            GPSaltSum = GPSaltSum + GPS.altitude.meters() - GPSavgAlt[GPSposn];
            GPSavgAlt[GPSposn] = GPS.altitude.meters();
            baseGPSalt = GPSaltSum/(float)(sizeof(GPSavgAlt)/sizeof(GPSavgAlt[0]));
            //barometric pressure running average
            pressurePosn++;
            if(pressurePosn >= (byte)(sizeof(pressureAvg5)/sizeof(pressureAvg5[0]))){pressurePosn = 0;}
            pressureSum = pressureSum + baro.pressure - pressureAvg5[pressurePosn];
            pressureAvg5[pressurePosn] = baro.pressure;
            pressureAvg = pressureSum/(float)(sizeof(pressureAvg5)/sizeof(pressureAvg5[0]));
            //sea level correction
            if(GPSbufferFull){baro.seaLevelPressure = pressureAvg / powf((44330 - baseGPSalt)/44330, 5.254861);}}
          liftoffLat=gpsLat;
          liftoffLatitude=gpsLatitude;
          liftoffLon=gpsLon;
          liftoffLongitude=gpsLongitude;
          liftoffYear = GPS.date.year();
          liftoffMonth = GPS.date.month();
          liftoffDay = GPS.date.day();}
        //capture the last GPS position
        if(events.mainDeploy || !events.touchdown || !events.timeOut){
          touchdownLat=gpsLat;
          touchdownLatitude=gpsLatitude;
          touchdownLon=gpsLon;
          touchdownLongitude=gpsLongitude;
          touchdownAlt = GPS.altitude.meters();}}

}//end void main loop
  
void parseBeep(long value, byte array[], byte arrayLen){
  boolean flag = false;
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

  //if the flight profile is complex, but there is no continuity on the extra pyros, then reset to a single stage flight
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
   if(pyro1.func == event){firePin = pyro1.firePin; digitalWrite(pyro1.firePin, HIGH); pyro1.fireStatus = true; pyro1.fireStart = fltTime.timeCurrent;}
   if(pyro2.func == event){firePin = pyro2.firePin; digitalWrite(pyro2.firePin, HIGH); pyro2.fireStatus = true; pyro2.fireStart = fltTime.timeCurrent;}
   if(pyro3.func == event){firePin = pyro3.firePin; digitalWrite(pyro3.firePin, HIGH); pyro3.fireStatus = true; pyro3.fireStart = fltTime.timeCurrent;}
   if(pyro4.func == event){firePin = pyro4.firePin; digitalWrite(pyro4.firePin, HIGH); pyro4.fireStatus = true; pyro4.fireStart = fltTime.timeCurrent;}}

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

void convertLocation(){
  //Convert back to NMEA format as required by the ground reciever
  //Latitude
  gpsInt = GPS.location.rawLat().deg;
  gpsFloat = GPS.location.lat();
  gpsLat = 'N';
  if(GPS.location.rawLat().negative){gpsFloat*=-1;gpsLat = 'S';}
  gpsLatitude = gpsInt*100+ 60*(gpsFloat-gpsInt);

  //Longitude
  gpsInt = GPS.location.rawLng().deg;
  gpsFloat = GPS.location.lng();
  gpsLon = 'E';
  if(GPS.location.rawLng().negative){gpsFloat*=-1;gpsLon = 'W';}
  gpsLongitude = gpsInt*100+ 60*(gpsFloat-gpsInt);}

void readEEPROMsettings(){

  //read user settings
  settings.fltProfile = (char)EEPROM.read(eeprom.fltProfile);
  settings.units = (char)EEPROM.read(eeprom.units);
  if(settings.units == 'M'){unitConvert = 1.0F;}
  settings.reportStyle = (char)EEPROM.read(eeprom.reportStyle);
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.setupTime + i);}
  settings.setupTime = ulongUnion.val;
  for(byte i=0; i<2; i++){intUnion.Byte[i] = (byte)EEPROM.read(eeprom.gTrigger + i);}
  settings.gTrigger = intUnion.val;
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.detectLiftoffTime + i);}
  settings.detectLiftoffTime = ulongUnion.val;
  for(byte i=0; i<4; i++){floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.mainDeployAlt + i);}
  settings.mainDeployAlt = floatUnion.val;
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.apogeeDelay + i);}
  settings.apogeeDelay = ulongUnion.val;
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.rcdTime + i);}
  settings.rcdTime = ulongUnion.val;
  settings.silentMode = (boolean)EEPROM.read(eeprom.silentMode);
  settings.magSwitchEnable = (boolean)EEPROM.read(eeprom.magSwitchEnable);
  settings.inflightRecover = (byte)EEPROM.read(eeprom.inflightRecover);
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

void timerSendPkt(){sendPkt = true;}
