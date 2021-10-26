//HPR Rocket Flight Computer
//Original sketch by Bryan Sparkman, TRA #12111, NAR #85720, L3
//This is built for the Teensy3.5 or 3.6
//Code Line Count: 2360 main + 651 Calibration + 319 Event_Logic + 285 GPSconfig + 571 Inflight_Recover + 372 Rotation + 287 SD + 2072 SensorDrivers + 625 SpeedTrig + 372 Telemetry = 7914 lines of code
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
//V4_3_5 removes auto-adjusting gain (no benefit in testing), improved calibration routines, extends baro calibration to all sensors, removed high-G axis mode from user settings, adds H3LIS331DL SPI bus support
//--------FEATURES----------
//1400Hz 3-axis digital 24G and 100G accelerometer data logging
//1400Hz 3-axis digital 2000dps gyroscope data logging
//1400Hz of flight events & continuity data logging
//1400Hz of sensor-fuzed speed & altitude
//100Hz of pitch, yaw, roll rotation
//40Hz of of magnetic data logging and magnetic roll
//30Hz-100Hz of digital barometric data logging (Altitude, pressure, temperature)
//30Hz of main battery voltage (1400Hz during events)
//20Hz of LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GNSS altitude, GNSS position, packet number)
//5Hz-25Hz of GNSS data logging (chip-dependent data rates & constellations)
//4 programmable high-current pyro outputs with continuity checks
//8 configurable servo outputs (4 powered, 4 un-powered)
//User Selectable Flight Mode: Single-Stage, Two-Stage, Airstart, or Booster
//Mach immune, sensor-fusion based apogee event
//Barometric based main deploy event
//LoRa telemetry over 433MHz or 915MHz (433MHz: USA amateur 70cm band, EUR licencse free) (915MHz: USA FHSS licence free or USA amateur license use non-FHSS) 
//Configurable Apogee delay
//Audible pre-flight continuity report
//Audible Post-flight max altitude & speed report
//Optional Audible Battery Voltage report at startup
//Optional Magnetic Switch Startup & Shut-down
//Mount in any orientation, automatic orientation detection during calibration
//Separate file for each flight up to 100 flights
//Bench-test mode activated w/ tactile button; user configurable status messages over USB Serial
//Built-in self-calibration mode 
//Reads user flight profile from SD card
//Report in SI or Metric units
//Preflight audible reporting options: Perfectflight or Marsa
//User selectable inflight brownout recovery
//User selectable active stabilization for roll, pitch, and yaw correction
//User selectable return-to-pad controlled recovery
//Base code is compatible with many different sensors over I2C
//Base code configurable to use different pin input/outputs and I2C bus options
//-------FUTURE UPGRADES----------
//Active Stabilization (started)
//Return-to-Base capability (started)
//Remote Arm & Shutdown Commands over LoRa
//Ground Station Bluetooth Datalink to smartphone
//Smartphone App
//------TO DO LIST------
//Test MS5611 code
//Bench test inflight recovery routines
//Consolidate BMP180 into a general getBaro routine
//Verify testing of new airstart code
//create RFM95W/96W routines to eliminate RadioHead library
//stuck-in-a-loop detection and breakout

//-------CODE START--------
#include <SdFat.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <PWMServo.h>

//Teensy 3.5 Hardware Serial for GPS
HardwareSerial HWSERIAL(Serial1);

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

//SDIO Setup: requires SDFat v2.1 or greater, is actually slower than V1.1.4
//SdFs SD;
//FsFile outputFile;
//FsFile settingsFile;
//#define USE_UTF8_LONG_NAMES 1

//SDIO Setup, requires SDFat v1.1.4, V2.0.0 is incompatible with RadioHead library
SdFatSdioEX SD;
File outputFile;
File settingsFile;

//ADC Setup
//ADC *adc = new ADC(); // adc object;

//GLOBAL VARIABLES
//-----------------------------------------
//Set code version
//-----------------------------------------
const float codeVersion = 4.35;
//-----------------------------------------
//EEPROM allocation
//-----------------------------------------
typedef struct{
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
    int imuXsign = 24;//24-29: IMU to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    int imuXptr = 25;
    int imuYsign= 26;
    int imuYptr = 27;
    int imuZsign = 28;
    int imuZptr = 29;
    int hiGxSign = 30;//30-35: highG to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    int hiGxPtr = 31;
    int hiGySign = 32;
    int hiGyPtr = 33;
    int hiGzSign = 34;
    int hiGzPtr = 35;
    int IMU2hiGxSign = 36;//36-41: IMU-to-highG orientation translation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
    int IMU2hiGxAxis = 37;
    int IMU2hiGySign = 38;
    int IMU2hiGyAxis = 39;
    int IMU2hiGzSign = 40;
    int IMU2hiGzAxis = 41;
    int i2cBus = 42;//42   : I2C bus
    int pyro1ContPin = 43;//43   : Pyro 1 Continuity Pin
    int pyro1FirePin = 44;//44   : Pyro 1 Fire Pin
    int pyro2ContPin = 45;//45   : Pyro 2 Continuity Pin
    int pyro2FirePin = 46;//46   : Pyro 2 Fire Pin
    int pyro3ContPin = 47;//47   : Pyro 3 Continuity Pin
    int pyro3FirePin = 48;//48   : Pyro 3 Fire Pin
    int pyro4ContPin = 49;//49   : Pyro 4 Continuity Pin
    int pyro4FirePin = 50;//50   : Pyro 4 Fire Pin
    int nullContPin = 51;//51   : Null Continuity / Fire Pin
    int nullFirePin = 51;
    int beepPin = 52;//52   : Beeper Pin
    int battReadPin = 53;//53   : Battery Read Pin
    int testModeGndPin = 54;//54   : Test Mode Button Gnd Pin
    int testModeRdPin= 55;//55   : Test Mode Read Pin
    int radioCSpin = 56;//56   : radio CS pin
    int radioIRQpin = 57;//57   : radio interrupt pin
    int radioRstPin = 58;//58   : radio reset pin
    int radioEnPin = 59;//59   : radio enable pin (optional w/ Adafruit breakout only)
    int accelID = 60;//60   : Sensor ID accel / mag 
    int gyroID = 61;//61   : Sensor ID gyro
    int highGID = 62;//62   : Sensor ID highG
    int baroID = 63;//63   : Sensor ID baro
    int radioID = 64;//64   : Sensor ID Radio
    int GPSID = 65;//65   : Sensor ID GPS
    int servo1pin = 66;//66   : servo 1 control pin
    int servo2pin = 67;//67   : servo 2 control pin
    int servo3pin = 68;//68   : servo 3 control pin
    int servo4pin = 69;//69   : servo 4 control pin
    int servo5pin = 70;//70   : servo 5 control pin
    int servo6pin = 71;//71   : servo 6 control pin
    int servo7pin = 72;//72   : servo 7 control pin
    int servo8pin = 73;//73   : servo 8 control pin
    int callSign = 74;//74-79: Ham Radio Callsign
    int baroPressOffset = 80;//80-83 : barometer pressure offset
    int baroTempOffset = 84;//84-87  : barometer temperature offset
    //-------------------------------------------------------------
    //flight settings stored in EEPROM to facilitate rapid recovery
    //-------------------------------------------------------------
    int rocketName = 88; //Maximum of 20 characters
    int fltProfile = 108;
    int units = 109;
    int reportStyle = 110;
    int setupTime = 111;//4
    int gTrigger = 115;//2
    int detectLiftoffTime = 117;//4
    int mainDeployAlt = 121;//2
    int rcdTime = 123;//4
    int apogeeDelay = 127;
    int highG3axis = 128;
    int silentMode = 129;
    int magSwitchEnable = 130;
    int inflightRecover = 131;
    int fireTime = 132;//4
    int pyro4Func = 136;
    int pyro3Func = 137;
    int pyro2Func = 138;
    int pyro1Func = 139;
    int radioTXenable = 140;
    int TXpwr = 141;
    int TXfreq = 142;//4
    int FHSS = 146;
    int boosterSeparationDelay = 147;//4
    int sustainerFireDelay = 151;//4
    int airStart1Event = 154;
    int airStart1Delay = 155;//4
    int airStart2Event = 159;
    int airStart2Delay = 160;//4
    int altThreshold = 164;//2
    int maxAngle = 166;
    int stableRotn = 167;
    int stableVert = 168;
    int flyback = 169;
    int serialDebug = 171;
    int lastFile = 172;//2
    int baseAlt = 173;//4
    int lastEvent = 177;
    //-------------------------------------------------------------
    //active stabilization parameters
    //-------------------------------------------------------------
    int servo1trim = 178;
    int servo2trim = 179;
    int servo3trim = 180;
    int servo4trim = 181;
    int servo5trim = 182;
    int servo6trim = 183;
    int servo7trim = 184;
    int servo8trim = 185;
    int Kp = 186;//4
    int Ki = 190;//4
    int Kd = 194;//4
    int highG_CS = 198;//1
    int serialNum = 199;//4
} EEPROM_map;
EEPROM_map eeprom;
//-----------------------------------------
//Sensor booleans
//-----------------------------------------
typedef struct{
  byte accel;
  byte gyro;
  byte highG;
  byte baro;
  byte radio;
  byte GPS;
  boolean status_LSM303 = false;
  boolean status_LSM9DS1 = false;
  boolean status_L3GD20H = false;
  boolean status_MS5611 = false;
  boolean status_MPL3115A2 = false;
  boolean status_BMP180 = false;
  boolean status_BMP280 = false;
  boolean status_BMP388 = false;
  boolean status_H3LIS331DL = false;
  boolean status_ADS1115 = false;
  boolean status_ADXL377 = false;
  boolean status_RFM96W = false;
  boolean pyroPWM = true;
} hardwareSensors;
hardwareSensors sensors;
//-----------------------------------------
//Sensor variables
//-----------------------------------------
byte rawData[12];
typedef struct{
  byte addr;
  byte reg;
  boolean SPIbus = false;
  byte i2cBytes;
  byte gainLevel;
  int16_t ADCmax;
  boolean writeGain = true;
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
  int8_t *ptrXdir;
  int8_t *ptrYdir;
  int8_t *ptrZdir;
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
} sensorData;
sensorData accel;
sensorData mag;
sensorData gyro;
sensorData highG;
//-----------------------------------------
//Set defaults for user defined variables
//-----------------------------------------
typedef struct {
  boolean testMode = false;
  boolean calibrationMode = false;
  //--------flight settings----------------
  char rocketName[20] = ""; //Maximum of 20 characters
  char fltProfile = 'S';
  char units = 'S';
  char reportStyle = 'P';
  unsigned long setupTime = 5000000UL;
  int gTrigger = 3415; //2.5G trigger
  unsigned long detectLiftoffTime = 500000UL; //0.5s
  float mainDeployAlt = 153;//Up to 458m for main deploy
  unsigned long rcdTime = 900000000UL; //15min
  unsigned long apogeeDelay = 1000000UL; //1.0s apogee delay
  boolean highG3axis = false;
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
  boolean radioTXenable = true;//false turns off radio transmissions
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
} userSettings;
userSettings settings;
//-----------------------------------------
//GPIO pin mapping
//-----------------------------------------
typedef struct{
  byte i2c;
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
  byte highG_CS;
} pinMap;
pinMap pins;
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
unsigned long TXdataStart;
boolean syncFreq = false;
byte pktPosn=0;
uint16_t sampNum = 0;
byte packetSamples = 4;
boolean radioInterference = false;
boolean gpsTransmit = false;
unsigned long TXnow = 0UL;
byte TXnum = 0;
boolean flagHopFreq = false;
boolean flagSyncFreq = false;
unsigned long RIpreLiftoff = 1000000UL;
unsigned long RIinflight = 50000UL;
unsigned long RIpostFlight = 10000000UL;
unsigned long RIsyncOffset = 0UL;
unsigned long radioInterval = 0UL;;
unsigned long lastTX = 0UL;
boolean radioTX = false;
typedef struct{
  int16_t packetnum = 0;
  uint8_t event = 0;
  uint16_t fltTime;
  uint16_t timeStamp = 0;
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
} radioPkt;
radioPkt radio;
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
           long gdt = 0L;
  unsigned long gyro = 0UL;
  unsigned long gyroClock = 0UL;
  unsigned long gyroClockPrev = 0UL;
} timerList;
timerList fltTime;
timerList resetFltTime;
//-----------------------------------------
//continuity Booleans
//-----------------------------------------
typedef struct{
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
  } contStatus;
contStatus cont;
//-----------------------------------------
//Non-Event Booleans
//-----------------------------------------
boolean rotnOK = true;
boolean altOK = false;
boolean beep = false;
boolean pyroFire = false;
boolean fileClose = false;
unsigned long boosterBurpTime;
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
long highGsum = 0L;
int highGfilter[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte sizeHighGfilter = 30;
byte filterPosn = 0;
float highGsmooth;
float A2D;//analog to digital gain conversion
union {
   int16_t calValue; 
   byte calByte[2];
} calUnion;
union {
  float val;
  byte calByte[4];
} calFloat;
//-----------------------------------------
//Altitude & Baro Sensor variables
//-----------------------------------------
float Alt = 0.0;
float baseAlt = 10.0;
float maxAltitude = 0.0;
float pressure;
float temperature;
float baseAltBuff[30];
float sumBaseAlt = 0.0;
byte baseAltPosn = 0;
const unsigned long tmpRdTime = 4500; //BMP180 needs 4.5ms to read temp
const unsigned long bmpRdTime = 25500; //BMP180 needs 25.5ms to read pressure
unsigned long lastBMP = 0UL;
unsigned long lastBaro = 0UL;
unsigned long timeBtwnBaro = 50000UL;
boolean newBaro = false;
float seaLevelPressure = 1013.25;
float pressureAvg = 0;
float pressureAvg5[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float pressureSum = 0.0;
byte pressurePosn = 0;
float baroTempOffset;
float baroPressOffset;
boolean filterFull = false;
//-----------------------------------------
//Baro Reporting Variables
//-----------------------------------------
byte baroApogeePosn = 0;
int baroApogee = 0;
int baroLast5 [5] = {0, 0, 0, 0, 0};
byte baroTouchdown = 0;
byte touchdownTrigger = 5;
unsigned long baroTime = 0UL;
unsigned long prevBaroTime = 0UL;
float prevBaroAlt = 0.0;
float rawAltSum = 0.0;
float rawAltBuff[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float altMoveAvg = 0.0;
float smoothAlt = 0.0F;
byte rawAltPosn = 0;
byte altAvgPosn = 0;
float altAvgBuff[30] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
unsigned long baroTimeBuff[30] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                  0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                  0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL};
int baroVelPosn = 0;
float baroVel = 0.0;
boolean getTemp = true;
boolean readTemp = false;
boolean getPress = false;
boolean readPress = false;
unsigned long tempReadStart;
unsigned long pressReadStart;
//-----------------------------------------
//Magnetometer Variables
//-----------------------------------------
unsigned long magTime = 25000UL;
unsigned long magCounter = 0UL;
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
float maxBaroVel = 0.0F;
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
uint32_t lastHighG = 0UL;
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
//debug
//-----------------------------------------
long debugStart;
long debugTime;
boolean GPSecho = false;
boolean radioDebug = false;
boolean disp = false;

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

void setup(void) {
  
  Serial.begin(38400);
  delay(500);

  //check to see if we are restarting from a powerloss
  //if so, then we need to rapidly get the system back up and going
  //call the rapidReset routine and immediately exit void setup
  byte lastEvent = EEPROM.read(eeprom.lastEvent);
  if(lastEvent == (byte)255){
    lastEvent = 27;
    EEPROM.write(eeprom.lastEvent, lastEvent);}
  settings.inflightRecover = EEPROM.read(eeprom.inflightRecover);
  if(settings.inflightRecover > 0 && (lastEvent < 24 && lastEvent != 6 && lastEvent != 7)){
    
    //execute the rapid recovery routine++
    Serial.print("Rapid Reset Initiated, setting: ");Serial.println(settings.inflightRecover);
    if(rapidReset()){
      //exit setup and immediately re-enter the loop
      return;}}

  //Start Harware Serial communication
  HWSERIAL.begin(9600);
  
  //Start SDIO Comms with the SD card
  if(!SD.begin()){Serial.println(F("SD card failed!"));}//SDFat v1.4
  //if(!SD.begin(SdioConfig(FIFO_SDIO))){Serial.println(F("SD card failed!"));}
  
  //If an EEPROM settings file exists, open it and copy the values into EEPROM
  byte kk;
  int8_t ii;
  if(SD.exists("EEPROMsettings.txt")){
    Serial.println(F("EEPROM file found!  Writing initial EEPROM Settings..."));
    settingsFile.open("EEPROMsettings.txt", O_READ);
    //-----------------------------------------------------------------
    //read the orientation of the accelerometers relative to each other
    //-----------------------------------------------------------------
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    EEPROM.update(eeprom.IMU2hiGxSign, ii); EEPROM.update(eeprom.IMU2hiGxAxis, (char)dataString[1]);//High-G X-axis translation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    EEPROM.update(eeprom.IMU2hiGySign, ii); EEPROM.update(eeprom.IMU2hiGyAxis, (char)dataString[1]);//High-G Y-axis translation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    EEPROM.update(eeprom.IMU2hiGzSign, ii); EEPROM.update(eeprom.IMU2hiGzAxis, (char)dataString[1]);//High-G Z-axis translation
    //-----------------------------------------------------------------
    //read the control pins and sensor IDs -- must be in the same order as the text file
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.i2cBus, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro1ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro1FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro2ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro2FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro3ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro3FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro4ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro4FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.nullContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.beepPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.battReadPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.testModeGndPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.testModeRdPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioCSpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioIRQpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioRstPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioEnPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.highG_CS, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.accelID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.gyroID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.highGID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.baroID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.GPSID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo1pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo2pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo3pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo4pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo5pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo6pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo7pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo8pin, kk);
    //-----------------------------------------------------------------
    //read the ham radio call sign
    //-----------------------------------------------------------------
    parseNextVariable(false);
    for(byte i = 0; i < 6; i++){EEPROM.update(eeprom.callSign+i, (char)dataString[i]);}
    //-----------------------------------------------------------------
    //read whether or not to put the unit into magnetometer calibration mode
    //-----------------------------------------------------------------
    magCalibrateMode = (byte)parseNextVariable(true);//Sets Magnetometer Calibration Mode, not stored in EEPROM
    settingsFile.close();
    SD.remove("EEPROMsettings.txt");
    Serial.println(F("Complete!"));}

  //Read pin settings from EEPROM
  Serial.print(F("Reading EEPROM..."));
  pins.i2c = EEPROM.read(eeprom.i2cBus);
  pins.pyro1Cont = EEPROM.read(eeprom.pyro1ContPin);
  pins.pyro1Fire = EEPROM.read(eeprom.pyro1FirePin);
  pins.pyro2Cont = EEPROM.read(eeprom.pyro2ContPin);
  pins.pyro2Fire = EEPROM.read(eeprom.pyro2FirePin);
  pins.pyro3Cont = EEPROM.read(eeprom.pyro3ContPin);
  pins.pyro3Fire = EEPROM.read(eeprom.pyro3FirePin);
  pins.pyro4Cont = EEPROM.read(eeprom.pyro4ContPin);
  pins.pyro4Fire = EEPROM.read(eeprom.pyro4FirePin);
  pins.nullCont = EEPROM.read(eeprom.nullContPin);
  pins.nullFire = pins.nullCont+1;
  pins.beep = EEPROM.read(eeprom.beepPin);
  pins.batt = EEPROM.read(eeprom.battReadPin);
  pins.testGnd = EEPROM.read(eeprom.testModeGndPin);
  pins.testRead = EEPROM.read(eeprom.testModeRdPin);
  pins.radioCS = EEPROM.read(eeprom.radioCSpin);
  pins.radioIRQ = EEPROM.read(eeprom.radioIRQpin);
  pins.radioRST = EEPROM.read(eeprom.radioRstPin);
  pins.radioEN = EEPROM.read(eeprom.radioEnPin);
  pins.highG_CS = EEPROM.read(eeprom.highG_CS);
  sensors.accel = EEPROM.read(eeprom.accelID);
  sensors.gyro = EEPROM.read(eeprom.gyroID);
  sensors.highG = EEPROM.read(eeprom.highGID);
  sensors.baro = EEPROM.read(eeprom.baroID);
  sensors.radio = EEPROM.read(eeprom.radioID);
  sensors.GPS = EEPROM.read(eeprom.GPSID);
  pins.servo1 = EEPROM.read(eeprom.servo1pin);
  pins.servo2 = EEPROM.read(eeprom.servo2pin);
  pins.servo3 = EEPROM.read(eeprom.servo3pin);
  pins.servo4 = EEPROM.read(eeprom.servo4pin);
  pins.servo5 = EEPROM.read(eeprom.servo5pin);
  pins.servo6 = EEPROM.read(eeprom.servo6pin);
  pins.servo7 = EEPROM.read(eeprom.servo7pin);
  pins.servo8 = EEPROM.read(eeprom.servo8pin);
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
  //Set the pyro firing pins to LOW for safety
  digitalWrite(pins.pyro1Fire, LOW);
  digitalWrite(pins.pyro2Fire, LOW);
  digitalWrite(pins.pyro3Fire, LOW);
  digitalWrite(pins.pyro4Fire, LOW);
  Serial.println("Set Pins Low");
  
  //check if the test mode button is being held
  digitalWrite(pins.testGnd, LOW);
  delay(50);
  if(digitalRead(pins.testRead) == LOW){
    settings.testMode = true; 
    Serial.println(F("Bench-Test Mode Confirmed"));
    Serial.println(F("Press the button again to calibrate the accelerometers."));
    Serial.println(F("Enter 'b' into the serial monitor to calibrate the barometer"));
    Serial.println(F("Enter 'c' into the serial monitor to adjust the canard trims"));}

  //setup the ADC for sampling the battery
  analogReadResolution(16);
    
  //Start I2C communication
  if(settings.testMode){Serial.print("Starting i2c bus  ");Serial.println(pins.i2c);}
  switch (pins.i2c){
   
    case 0: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
            break;
    case 1: Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
            break;
    case 2: Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, 400000);
            break;
    case 3: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_7_8, I2C_PULLUP_EXT, 400000);
            pins.i2c = 0;
            break;
    case 4: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
            pins.i2c = 0;
            break;
    case 5: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_33_34, I2C_PULLUP_EXT, 400000);
            pins.i2c = 0;
            break;
    case 6: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_47_48, I2C_PULLUP_EXT, 400000);
            pins.i2c = 0;
            break;}

  //Start SPI communication
  SPI.begin();

  //Start Sensors
  Serial.println(F("Starting Sensors..."));
  beginAccel();
  beginGyro();
  settings.highG3axis = true;
  beginHighG('F');
  beginBaro();

  //Start the radio
  //RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
  RH_RF95 rf95(pins.radioCS);
  pinMode(pins.radioRST, OUTPUT);
  digitalWrite(pins.radioRST, HIGH);
  delay(100);
  
  //Radio manual reset
  digitalWrite(pins.radioRST, LOW);
  delay(100);
  digitalWrite(pins.radioRST, HIGH);
  delay(100);
  
  //Initialize the radio
  sensors.status_RFM96W = rf95.init();
  if(settings.testMode){
    if(sensors.status_RFM96W){Serial.println(F("RFM96W/95W OK!"));}
    else{Serial.println(F("RFM96W/95W not found!"));}}
  //433MHz Radio
  if(sensors.status_RFM96W && settings.testMode && sensors.radio == 1){Serial.println(F("Starting 433MHz"));}
  //915MHz Radio
  if(sensors.status_RFM96W && settings.testMode && sensors.radio == 2){Serial.println(F("Starting 915MHz"));}
    
  if(settings.testMode){Serial.println(F("Reading User Settings from SD Card"));}
  //Open the settings file
  settingsFile.open("Settings.txt", O_READ);

  //Read in the user defined variables
  parseNextVariable(false);n=0;
  //Rocket name
  while (dataString[n]!='\0'){settings.rocketName[n] = dataString[n];n++;}settings.rocketName[n]='\0';n=0;
  parseNextVariable(false); settings.fltProfile = dataString[0];
  parseNextVariable(false); settings.units = dataString[0]; if(settings.units == 'M'){unitConvert = 1.0F;}
  parseNextVariable(false); settings.reportStyle = dataString[0];
  settings.setupTime = (unsigned long)(parseNextVariable(true)*1000UL);
  settings.gTrigger = (int)(parseNextVariable(true)*g);
  settings.detectLiftoffTime = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.mainDeployAlt = (float)(parseNextVariable(true)/unitConvert);
  settings.apogeeDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.rcdTime = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.silentMode = (boolean)(parseNextVariable(true));
  settings.magSwitchEnable = (boolean)(parseNextVariable(true));
  settings.inflightRecover = (byte)(parseNextVariable(true));
  settings.fireTime = (unsigned long) (parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.pyro4Func = dataString[0];
  parseNextVariable(false); settings.pyro3Func = dataString[0];
  parseNextVariable(false); settings.pyro2Func = dataString[0];
  parseNextVariable(false); settings.pyro1Func = dataString[0];
  settings.radioTXenable = (boolean)parseNextVariable(true);
  settings.TXpwr = (byte)(parseNextVariable(true));
  settings.TXfreq = (float)(parseNextVariable(true));
  settings.FHSS = (boolean)parseNextVariable(true);
  settings.boosterSeparationDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.sustainerFireDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.airStart1Event = dataString[0];
  settings.airStart1Delay = (unsigned long)(parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.airStart2Event = dataString[0];
  settings.airStart2Delay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.altThreshold = (int)(parseNextVariable(true)/unitConvert);
  settings.maxAngle = (int)parseNextVariable(true)*10;
  settings.stableRotn = (boolean)parseNextVariable(true);
  settings.stableVert = (boolean)parseNextVariable(true);
  settings.flyBack = (boolean)parseNextVariable(true);
  settings.serialDebug = (byte)parseNextVariable(true);
  switch (settings.serialDebug){
    case 0: GPSecho = false; radioDebug = false; break;
    case 1: GPSecho = true; radioDebug = false; break;
    case 2: GPSecho = false; radioDebug = true; break;
    case 3: GPSecho = true; radioDebug = true; break;}
  //close the settings file
  settingsFile.close();

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
  EEPROM.update(eeprom.highG3axis, settings.highG3axis);
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
  EEPROM.update(eeprom.radioTXenable, settings.radioTXenable);
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
  Serial.print(F("Flight Profile: ")); Serial.println(settings.fltProfile);
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

  //check for silent mode
  if(settings.testMode && settings.silentMode){pins.beep = pins.nullCont; Serial.println(F("Silent Mode Confirmed"));}
  
  //if telemetry is disabled and silent mode is on, then use the LED to signal the beeps
  if(settings.testMode && settings.silentMode && !settings.radioTXenable){pins.beep = 13; pinMode(pins.beep, OUTPUT);}

  //setup the radio
  if(!settings.radioTXenable){
    if(pins.radioEN != pins.nullCont){pinMode(pins.radioEN, OUTPUT);digitalWrite(pins.radioEN, LOW);}
    if(settings.testMode){Serial.println(F("Telemetry OFF!"));}}
  else{
    //Set the radio output power & frequency
    rf95.setTxPower(settings.TXpwr, false);//23 max setting; 20mW=13dBm, 30mW=15dBm, 50mW=17dBm, 100mW=20dBm
    if (sensors.radio == 2 && settings.FHSS){
      settings.TXfreq = 902.300; 
      RIpreLiftoff = 600000UL;
      if(settings.testMode){Serial.println("FHSS Active!");}}//sync freq
    rf95.setFrequency(settings.TXfreq);
    if(settings.testMode){
      Serial.print("Radio Freq: ");Serial.println(settings.TXfreq, 3);
      Serial.print("Radio Power: ");Serial.println(settings.TXpwr);}
    radioInterval = RIpreLiftoff;}

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
  baroPressOffset = floatUnion.val;
  for(byte i = 0; i < 4; i++){floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.baroTempOffset +i);}
  baroTempOffset = floatUnion.val;

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
    Serial.print(F("baroTempOffset: "));Serial.println(baroTempOffset, 1);
    Serial.print(F("baorPressOffset: "));Serial.println(baroPressOffset, 2);}

  //read the orientation variables from EEPROM
  //IMU X-Axis
  accel.orientX = mag.orientX = gyro.orientX = (char)EEPROM.read(eeprom.imuXptr);
  accel.dirX = mag.dirX = gyro.dirX = (int8_t)EEPROM.read(eeprom.imuXsign);
  if(accel.orientX == 'X'){
    accel.ptrX = &accel.rawX; mag.ptrX = &mag.rawX; gyro.ptrX = &gyro.rawX;
    accel.ptrXdir = &accel.dirX; mag.ptrXdir = &mag.dirX; gyro.ptrXdir = &gyro.dirX;}
  if(accel.orientX == 'Y'){
    accel.ptrX = &accel.rawY; mag.ptrX = &mag.rawY; gyro.ptrX = &gyro.rawY;
    accel.ptrXdir = &accel.dirY; mag.ptrXdir = &mag.dirY; gyro.ptrXdir = &gyro.dirY;}
  if(accel.orientX == 'Z'){
    accel.ptrX = &accel.rawZ; mag.ptrX = &mag.rawZ; gyro.ptrX = &gyro.rawZ;
    accel.ptrXdir = &accel.dirZ; mag.ptrXdir = &mag.dirZ; gyro.ptrXdir = &gyro.dirZ;}
  //IMU Y-Axis
  accel.orientY = mag.orientY = gyro.orientY = (char)EEPROM.read(eeprom.imuYptr);
  accel.dirY = mag.dirY = gyro.dirY = (int8_t)EEPROM.read(eeprom.imuYsign);
  if(accel.orientY == 'X'){
    accel.ptrY = &accel.rawX; mag.ptrY = &mag.rawX; gyro.ptrY = &gyro.rawX;
    accel.ptrYdir = &accel.dirX; mag.ptrYdir = &mag.dirX; gyro.ptrYdir = &gyro.dirX;}
  if(accel.orientY == 'Y'){
    accel.ptrY = &accel.rawY; mag.ptrY = &mag.rawY; gyro.ptrY = &gyro.rawY;
    accel.ptrYdir = &accel.dirY; mag.ptrYdir = &mag.dirY; gyro.ptrYdir = &gyro.dirY;}
  if(accel.orientY == 'Z'){
    accel.ptrY = &accel.rawZ; mag.ptrY = &mag.rawZ; gyro.ptrY = &gyro.rawZ;
    accel.ptrYdir = &accel.dirZ; mag.ptrYdir = &mag.dirZ; gyro.ptrYdir = &gyro.dirZ;}
  //IMU Z-Axis
  accel.orientZ = mag.orientZ = gyro.orientZ = (char)EEPROM.read(eeprom.imuZptr);
  accel.dirZ = mag.dirZ = gyro.dirZ = (int8_t)EEPROM.read(eeprom.imuZsign);
  if(accel.orientZ == 'X'){
    accel.ptrZ = &accel.rawX; mag.ptrZ = &mag.rawX; gyro.ptrZ = &gyro.rawX;
    accel.ptrZdir = &accel.dirX; mag.ptrZdir = &mag.dirX; gyro.ptrZdir = &gyro.dirX;}
  if(accel.orientZ == 'Y'){
    accel.ptrZ = &accel.rawY; mag.ptrZ = &mag.rawY; gyro.ptrZ = &gyro.rawY;
    accel.ptrZdir = &accel.dirY; mag.ptrZdir = &mag.dirY; gyro.ptrZdir = &gyro.dirY;}
  if(accel.orientZ == 'Z'){
    accel.ptrZ = &accel.rawZ; mag.ptrZ = &mag.rawZ; gyro.ptrZ = &gyro.rawZ;
    accel.ptrZdir = &accel.dirZ; mag.ptrZdir = &mag.dirZ; gyro.ptrZdir = &gyro.dirZ;}
  //highG
  highG.dirX = (int8_t)EEPROM.read(eeprom.hiGxSign);
  highG.orientX = (char)EEPROM.read(eeprom.hiGxPtr);
  if(highG.orientX == 'X'){highG.ptrX = &highG.rawX; highG.ptrXdir = &highG.dirX;}
  if(highG.orientX == 'Y'){highG.ptrX = &highG.rawY; highG.ptrXdir = &highG.dirY;}
  if(highG.orientX == 'Z'){highG.ptrX = &highG.rawZ; highG.ptrXdir = &highG.dirZ;}
  highG.dirY = (int8_t)EEPROM.read(eeprom.hiGySign);
  highG.orientY = (char)EEPROM.read(eeprom.hiGyPtr);
  if(highG.orientY == 'X'){highG.ptrY = &highG.rawX; highG.ptrYdir = &highG.dirX;}
  if(highG.orientY == 'Y'){highG.ptrY = &highG.rawY; highG.ptrYdir = &highG.dirY;}
  if(highG.orientY == 'Z'){highG.ptrY = &highG.rawZ; highG.ptrYdir = &highG.dirZ;}
  highG.dirZ = (int8_t)EEPROM.read(eeprom.hiGzSign);
  highG.orientZ = (char)EEPROM.read(eeprom.hiGzPtr);
  if(highG.orientZ == 'X'){highG.ptrZ = &highG.rawX; highG.ptrZdir = &highG.dirX;}
  if(highG.orientZ == 'Y'){highG.ptrZ = &highG.rawY; highG.ptrZdir = &highG.dirY;}
  if(highG.orientZ == 'Z'){highG.ptrZ = &highG.rawZ; highG.ptrZdir = &highG.dirZ;}
  //display values from EEPROM
  if(settings.testMode){
    Serial.print(F("IMU.X is pointed to real world: "));Serial.print((accel.dirX == 1) ? '+' : '-');Serial.println(accel.orientX);
    Serial.print(F("IMU.Y is pointed to real world: "));Serial.print((accel.dirY == 1) ? '+' : '-');Serial.println(accel.orientY);
    Serial.print(F("IMU.Z is pointed to real world: "));Serial.print((accel.dirZ == 1) ? '+' : '-');Serial.println(accel.orientZ);
    Serial.print(F("highG.X is pointed to real world: "));Serial.print((highG.dirX == 1) ? '+' : '-');Serial.println(highG.orientX);
    Serial.print(F("highG.Y is pointed to real world: "));Serial.print((highG.dirY == 1) ? '+' : '-');Serial.println(highG.orientY);
    Serial.print(F("highG.Z is pointed to real world: "));Serial.print((highG.dirZ == 1) ? '+' : '-');Serial.println(highG.orientZ);}

  //restart the highG accelerometer at the higher rate
  if(settings.testMode){Serial.println(F("Restarting High-G Accelerometer at high rate"));}
  if(!highG.SPIbus){settings.highG3axis = false;}
  beginHighG('F');

  //Initialize the base altitude average
  for(byte i = 0; i < (byte)(sizeof(baseAltBuff)/sizeof(baseAltBuff[0])); i++){baseAltBuff[i] = 0.0F;}

  //Overrides for bench test mode
  if (settings.testMode) {
    settings.TXpwr = 2;
    rf95.setTxPower(settings.TXpwr, false);//lowest power setting
    Serial.print(F("Radio Power Reduced for Bench Test Mode: "));
    Serial.println(settings.TXpwr);
    settings.detectLiftoffTime = 10000UL; //0.01s
    settings.setupTime = 3000UL; //3s startup time
    settings.apogeeDelay = 1000000UL; //1s apogee delay
    settings.rcdTime = 15000000UL; //15s record time
    if(settings.stableRotn || settings.stableVert){settings.rcdTime = 30000000UL;}//30s of record time for serial plotting
    settings.gTrigger = (int)(1.5*g); //1.5G trigger
    maxAltitude = 11101/unitConvert;
    maxVelocity = 202/unitConvert;
    RIpostFlight = 1000000UL;
    radioInterval = RIpreLiftoff;
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

  if(settings.testMode){Serial.print(F("Creating new SD card file: FLIGHT"));}
  
 //Create and open the next file on the SD card
 n=0;
 while (SD.exists(dataString)) {
    n++;
    if(n<10){itoa(n, dataString + 7,10);}
    else{itoa(n, dataString + 6,10);}
    dataString[8]='.';}
  outputFile = SD.open(dataString, FILE_WRITE);
  dataString[0]=(char)0;
  //Print header
  outputFile.print(settings.rocketName);
  outputFile.print(F(" Code V"));
  outputFile.print(codeVersion);
  outputFile.print(F(",accelX,accelY,accelZ,gyroX,gyroY,gyroZ,"));
  if(settings.highG3axis){outputFile.print(F("highGx,highGy,"));}
  outputFile.println(F("highGz,smoothHighGz,rollZ,yawY,pitchX,offVert,intVel,intAlt,fusionVel,fusionAlt,fltEvents,pyroCont,pyroFire,pyroPin,baroAlt,altMoveAvg,baroVel,baroPress,baroTemp,battVolt,magX,magY,magZ,gnssLat,gnssLon,gnssSpeed,gnssAlt,gnssAngle,gnssSatellites,radioPacketNum"));
  outputFile.sync();

  if(settings.testMode){
    if(n<10){Serial.print('0');Serial.println(n);}
    else{Serial.println(n);}}

  //check continuity
  if (digitalRead(pyro1.contPin) == HIGH) {pyro1.contStatus = true;}
  if (digitalRead(pyro2.contPin) == HIGH) {pyro2.contStatus = true;}
  if (digitalRead(pyro3.contPin) == HIGH) {pyro3.contStatus = true;}
  if (digitalRead(pyro4.contPin) == HIGH) {pyro4.contStatus = true;}
  
  if(settings.testMode){
    Serial.print("Pyro4 Continuity: ");Serial.println((pyro4.contStatus) ? "Y" : "N");
    Serial.print("Pyro3 Continuity: ");Serial.println((pyro3.contStatus) ? "Y" : "N");
    Serial.print("Pyro2 Continuity: ");Serial.println((pyro2.contStatus) ? "Y" : "N");
    Serial.print("Pyro1 Continuity: ");Serial.println((pyro1.contStatus) ? "Y" : "N");}
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
  if(settings.testMode){Serial.print(F("Reporting continuity: "));Serial.println(cont.beepCode);}
  
  //set the beep delay and preflight beep code
  beep_delay = long_beep_delay;
  beepCode = cont.beepCode;
  
  //if the magnetic switch is enabled, beep the continuity code until the magnet is sensed
  if(settings.magSwitchEnable){
    setMagGain(false);
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
          delay(13);}}
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
  getHighG(true);
  int sampSize = 4000;
  int magSamps = 0;
  uint32_t sampDelay = 714 * (1-0.8);//20% of avg cycle time
  for (int i = 0; i < sampSize; i++) { 
    //get a sensor event
    getAccel();
    getGyro();
    getHighG(true);
    if(micros() - magCounter > magTime){
      magCounter = micros();
      getMag();
      //add up the magnetometer samples
      mag.sumX0 += mag.x;
      mag.sumY0 += mag.y;
      mag.sumZ0 += mag.z;
      magSamps++;}
    if(settings.testMode && i%100 == 0){
      Serial.print("Gyro: ");Serial.print(gyro.x);Serial.print(',');Serial.print(gyro.y);Serial.print(',');Serial.println(gyro.z);
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
  magCounter = 0;
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
  A2D = highG.gainZ / accel.gainZ;
  if(settings.testMode){
    Serial.println(F("Re-calibrating high-G accelerometer..."));
    Serial.print(F("Old Bias: "));Serial.println(highG.biasX);
    Serial.print(F("HighG.Z0: "));Serial.println(highG.z0);}
  if(highG.orientZ == 'X'){highG.biasX += highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));}
  if(highG.orientZ == 'Y'){highG.biasY += highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));}
  if(highG.orientZ == 'Z'){highG.biasZ += highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));}
  //highG.biasX = highGx0 - (int)((float)accel.x0 / (float)A2D) - 27;//old formula is kept for reference
  if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasX);}
  
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
  if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}
  
  //Reset the G-trigger
  if(accel.orientX == 'Z'){settings.gTrigger -= accel.dirX*accel.biasX;}
  if(accel.orientY == 'Z'){settings.gTrigger -= accel.dirY*accel.biasY;}
  if(accel.orientZ == 'Z'){settings.gTrigger -= accel.dirZ*accel.biasZ;}

  //set the booster burp check time
  boosterBurpTime = min(1000000UL, settings.boosterSeparationDelay-10000UL);
  
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
      if(settings.testMode){Serial.print(maxAltDigits[i-1]);}
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

  if(settings.testMode){
    Serial.println(F("Setup Complete.  Awaiting simulated launch."));
    Serial.print(F("Beginning Serial Output and Continuity Reporting: "));
    Serial.println(beepCode);
    delay(3000);}

  //initialize the radio timing
  radioInterval = RIpreLiftoff;
  lastTX = micros();
  
}//end setup

void loop(void){
  //RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
  RH_RF95 rf95(pins.radioCS);

  //Sample the Accelerometer & Gyro w/ timestamps
  getAccel();
  getGyro();

  //Get a barometric event if needed
  if(!sensors.status_BMP180 && micros()-lastBaro >= timeBtwnBaro){
      prevBaroAlt = Alt;
      getBaro();
      lastBaro = micros();
      prevBaroTime = baroTime;
      baroTime = lastBaro;
      newBaro=true;}

  //Get a BMP180 barometric event if needed
  //See if a new temp is needed
  if (sensors.status_BMP180){

    if(getTemp){
      initiateTemp();
      tempReadStart = micros();
      getTemp = false;
      readTemp = true;}

    if(readTemp && micros() - tempReadStart > tmpRdTime){
      initiatePressure(&temperature);
      pressReadStart = micros();
      readTemp = false;
      readPress = true;}

    if(readPress && micros() - pressReadStart > bmpRdTime){
      getPressure(&pressure);
      lastBMP = micros();
      prevBaroAlt = Alt;
      prevBaroTime = baroTime;
      baroTime = lastBMP;
      Alt = pressureToAltitude(seaLevelPressure, pressure);
      readPress = false;
      getTemp = true;
      newBaro = true;}}
      
  if(events.preLiftoff && newBaro){
      sumBaseAlt -= baseAltBuff[baseAltPosn];
      sumBaseAlt += Alt;
      baseAltBuff[baseAltPosn] = Alt;
      baseAltPosn++;
      if(baseAltPosn >= (byte)(sizeof(baseAltBuff)/sizeof(baseAltBuff[0]))){baseAltPosn = 0;}        
      baseAlt = sumBaseAlt/(float)(sizeof(baseAltBuff)/sizeof(baseAltBuff[0]));
      radio.baseAlt = (int16_t)baseAlt;}

  //look for a shutdown command and if seen, stop all progress for a hard reset
  if (events.preLiftoff && settings.magSwitchEnable){
    getMag();
    n=0;
    if(abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger){n=1;}
    while(n==1){digitalWrite(pins.beep, HIGH);delay(1000);}}
  
  //detect liftoff
  if (!events.liftoff && accel.z > settings.gTrigger && !events.touchdown && !events.timeOut) {
    if(settings.testMode){Serial.println(' ');Serial.println(F("Simulated Liftoff Detected!"));}
    fltTime.padTime = micros();
    events.preLiftoff = false;
    events.liftoff = true;
    events.inFlight = true;
    fltTime.liftoff = fltTime.timeCurrent;
    radio.event = 1;
    radio.alt = 0;
    radioInterval = RIinflight;
    if(settings.radioTXenable){lastTX = micros() - radioInterval;}
    liftoffHour = GPS.time.hour();
    liftoffMin = GPS.time.minute();
    liftoffSec = GPS.time.second();
    liftoffMili = GPS.time.centisecond();
    maxGPSalt = 0.0F;
    //store base alt in EEPROM
    floatUnion.val = baseAlt;
    if(settings.inflightRecover != 0){
      for(byte i = 0; i<4; i++){EEPROM.update(eeprom.baseAlt + i, floatUnion.Byte[i]);}
      EEPROM.update(eeprom.lastEvent, radio.event);}}

  if (events.liftoff) {

    //Update master gyro timing variables
    fltTime.gdt = long(fltTime.gyroClock - fltTime.gyroClockPrev);
    fltTime.gyro += (unsigned long)fltTime.gdt;

    //update master timing variables
    fltTime.dt = fltTime.tmClock - fltTime.tmClockPrev;
    fltTime.timeCurrent += fltTime.dt;
    
    //Get High-G Accelerometer Data and update the moving average
    //which greatly improves accuracy with minimal latency
    getHighG(settings.highG3axis);
    highGsum -= highGfilter[filterPosn];
    highGfilter[filterPosn] = highG.z;
    highGsum += highGfilter[filterPosn];
    filterPosn++;
    lastHighG = fltTime.timeCurrent;
    if(filterPosn >= sizeHighGfilter){filterPosn = 0; if(!filterFull){filterFull = true;}}
    if(!filterFull){highGsmooth = (int16_t)(highGsum / filterPosn);}
    else{highGsmooth = (int16_t)(highGsum / sizeHighGfilter);}
      
    //See if a new altitude reading is available
    if(newBaro){
      Alt -= baseAlt;
      if(Alt > maxAltitude && !events.apogee){maxAltitude = Alt;}
      //Smoothed barometric altitude of the last 10 altitude readings
      rawAltSum -= rawAltBuff[rawAltPosn];
      rawAltBuff[rawAltPosn] = Alt;
      rawAltSum += rawAltBuff[rawAltPosn];
      rawAltPosn++;
      if(rawAltPosn >= (byte)(sizeof(rawAltBuff)/sizeof(rawAltBuff[0]))){rawAltPosn = 0;}
      altMoveAvg = rawAltSum / (float)(sizeof(rawAltBuff)/sizeof(rawAltBuff[0]));
      //barometric velocity & apogee trigger based on the last half-second of altitude measurments
      if(events.apogee){baroVelPosn = altAvgPosn;}
      else{
        baroVelPosn = altAvgPosn - 10;
        if(baroVelPosn < 0){baroVelPosn = (int)((sizeof(altAvgBuff)/sizeof(altAvgBuff[0])) - (10 - altAvgPosn));}}
      baroVel = (altMoveAvg - altAvgBuff[baroVelPosn])/((float)(fltTime.timeCurrent - baroTimeBuff[baroVelPosn])*mlnth);
      if(baroVel > maxBaroVel){maxBaroVel = baroVel;}
      radio.vel = baroVel;
      altAvgBuff[altAvgPosn] = altMoveAvg;
      baroTimeBuff[altAvgPosn] = fltTime.timeCurrent;
      altAvgPosn++;
      if(altAvgPosn >= (byte)(sizeof(altAvgBuff)/sizeof(altAvgBuff[0]))){altAvgPosn = 0;}
      //Baro touchdown trigger
      (events.mainDeploy && fabs(baroVel) < 1.0F )? baroTouchdown++ : baroTouchdown = 0;}

    //Get magnetometer data
    magCounter += fltTime.dt;
    if (magCounter >= magTime){
      getMag();
      magCounter = 0;}

    //Compute the current g-load. Use the high-G accelerometer if the IMU is pegged
    if(abs(accel.z) < accel.ADCmax){accelNow = (float)(accel.z - g) * accel.gainZ;}
    else{accelNow = (highGsmooth - (float)high1G) *  highG.gainZ;}
    
    //Integrate velocity and altitude data prior to apogee
    if(!events.apogeeFire || settings.testMode){

      //Capture the max acceleration
      if(accelNow > maxG){maxG = accelNow;}

      //calculate the new acceleration based velocity
      //this makes the apogee event mach immune
      accelVel += accelNow * (float)fltTime.dt * mlnth;
      
      //calculate the new acceleration based altitude
      accelAlt += accelVel * (float)fltTime.dt * mlnth;
      
      //calculate the new sensor fusion based velocity
      fusionVel += accelNow * (float)fltTime.dt * mlnth;
      if(newBaro && baroVel < fusionVel && baroVel < maxBaroVel && accelNow < 0.2F && fusionVel < 300.0F &&  Alt < 9000){
        fusionVel *= 0.99F;
        fusionVel += 0.01F * baroVel;}
      radio.vel = fusionVel;
      
      //update maximum velocity if it exceeds the previous value
      if(fusionVel > maxVelocity){maxVelocity = fusionVel;}
    
      //calculate the new sensor fusion based altitude
      fusionAlt += fusionVel * (float)fltTime.dt * mlnth;
      if(newBaro &&  Alt < 9000){
        fusionAlt *= 0.95;
        fusionAlt += 0.05 * Alt;}
      if(!altOK && (fusionAlt > settings.altThreshold || settings.testMode)){altOK = true;}

    }//end if !apogee
    
    //caluclate the partial rotation
    dx += gyro.x * fltTime.gdt;
    dy += gyro.y * fltTime.gdt;
    dz += gyro.z * fltTime.gdt; 

    //if active stablization is on, then use the faster algorithm
    if(settings.stableRotn || settings.stableVert){
      getDCM2DRotn(dx, dy, dz); 
      dx = dy = dz = 0L;
      uint32_t controlTime = micros();
      if (controlTime - timeLastControl >= controlInterval) {
        disp=true;
        setCanards();
        timeLastControl = controlTime;
      }}
    
    //if required update the quaternion rotation
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

    //check continuity
    pyro1.contStatus = ((digitalRead(pyro1.contPin) == HIGH) ? true : false);
    pyro2.contStatus = ((digitalRead(pyro2.contPin) == HIGH) ? true : false);
    pyro3.contStatus = ((digitalRead(pyro3.contPin) == HIGH) ? true : false);
    pyro4.contStatus = ((digitalRead(pyro4.contPin) == HIGH) ? true : false);
    
    //Read the battery voltage
    if((fltTime.timeCurrent - lastVolt > voltRate) || pyroFire){
      voltReading = analogRead(pins.batt);
      voltage = (float)(voltReading)*3.3*3.2*adcConvert;
      if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}
      writeVolt = true;
      lastVolt = fltTime.timeCurrent;}
        
    //Write the data to a string
    writeSDflightData();
        
    //Close file at Touchdown or Timeout
    if (events.timeOut || events.touchdown) {
      //write the final data to SD card
      writeSDfooter();      
      //close the file
      outputFile.close();
      fileClose = true;
      events.liftoff = false;
      digitalWrite(firePin, LOW);
      //Set the radio transmitter to post-flight data rate
      radioInterval = RIpostFlight;
      //Read max altitude into its beep array
      parseBeep(long(maxAltitude*unitConvert), maxAltDigits, 6);
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
      radio.maxAlt = (int16_t)maxAltitude;
      radio.maxVel = (int16_t)maxVelocity;
      radio.maxG = (int)((maxG / 0.980665));
      radio.maxGPSalt = (int)maxGPSalt;
      //write out the SD card timing variables
      if(settings.testMode){
        Serial.print(F("Max SD Write Time: "));Serial.println(maxWriteTime);
        Serial.print(F("Write Threshold: "));Serial.print(writeThreshold);Serial.print(F(", Count: "));Serial.println(writeThreshCount);}
      //center canards
      if (settings.stableVert || settings.stableRotn) {
        canardYaw1.write(90-servo1trim);
        canardYaw2.write(90-servo2trim);
        canardPitch3.write(90-servo3trim);
        canardPitch4.write(90-servo4trim);}
    }//end of timeout/touchdown protocols    
    
  }//end of liftoff flag

  //radio handling
  uint32_t timeNow = micros();
  if(settings.radioTXenable && !syncFreq && timeNow - lastTX >= radioInterval){lastTX += ((timeNow-lastTX) - (timeNow-lastTX)%radioInterval); radioSendPacket();}
  if(syncFreq && timeNow - TXdataStart > (140000UL + RIsyncOffset)){syncPkt();}//only used for 915MHz FHSS
  
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
  if(HWSERIAL.available() > 0){msgRX = true;}
  while(HWSERIAL.available() > 0){
    char c = HWSERIAL.read();
    GPS.encode(c);
    if(settings.testMode && GPSecho &&!events.liftoff){serialBuffer[serialPosn] = c; serialPosn++;}}
  if(settings.testMode && GPSecho && !events.liftoff && msgRX){serialBuffer[serialPosn] = '\0'; Serial.print(serialBuffer); msgRX = false; serialPosn = 0;}
    
  radio.satNum = GPS.satellites.value();
  if(micros() - timeLastGPS > 2000000UL){gpsFix = false;}
  if (GPS.location.isUpdated() || GPS.altitude.isUpdated()) {
        timeLastGPS = micros();
        fixCount++;
        gpsFix = 1;
        if(!configGPSflight && fixCount > 40){
          configGPS();
          gpsFix = false;
          configGPSflight = true; 
          configGPSdefaults = false;}
        gpsWrite = true;
        gpsTransmit = true;
        convertLocation();
        radio.GPSalt = (int16_t)(GPS.altitude.meters() - baseAlt);
        GPSlatitude.GPScoord = gpsLatitude;
        GPSlongitude.GPScoord = gpsLongitude;
        //capture GPS velocity
        GPSaltPosn++;
        if(GPSaltPosn >= (byte)(sizeof(GPSaltBuff)/sizeof(GPSaltBuff[0]))){GPSaltPosn = 0;}
        GPSvel = (GPS.altitude.meters() - GPSaltBuff[GPSaltPosn])/((micros() - GPStimeBuff[GPSaltPosn])*mlnth);
        GPSaltBuff[GPSaltPosn] = GPS.altitude.meters();
        GPStimeBuff[GPSaltPosn] = micros();
        //update sensor fusion velocity
        if(events.apogee){fusionVel = 0.9 * GPSvel + 0.1 * baroVel;}
        //capture max GPS alt
        if(GPS.altitude.meters() - baseAlt > maxGPSalt){maxGPSalt = GPS.altitude.meters() - baseAlt;}
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
            pressureSum = pressureSum + pressure - pressureAvg5[pressurePosn];
            pressureAvg5[pressurePosn] = pressure;
            pressureAvg = pressureSum/(float)(sizeof(pressureAvg5)/sizeof(pressureAvg5[0]));
            //sea level correction
            if(GPSbufferFull){seaLevelPressure = pressureAvg / powf((44330 - baseGPSalt)/44330, 5.254861);}}
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
       
float parseNextVariable(boolean flag){
  byte n=0;
  float dataValue;
  char c;
  n=0;
  c='\0';
  while (c != '='){c = settingsFile.read();}
  c = settingsFile.read();
  while (c != '\n'&& c !='\r' && n < sizeof(dataString)){
    c = settingsFile.read();
    if(c != '\n' && c != '\r'){dataString[n]=c;}
    else{dataString[n]='\0';}
    n++;}
  if(flag){
    dataValue = atof(dataString);
    return dataValue;}
  else{return '\0';}}//end void

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
  settings.highG3axis = (boolean)EEPROM.read(eeprom.highG3axis);
  settings.silentMode = (boolean)EEPROM.read(eeprom.silentMode);
  settings.magSwitchEnable = (boolean)EEPROM.read(eeprom.magSwitchEnable);
  settings.inflightRecover = (byte)EEPROM.read(eeprom.inflightRecover);
  for(byte i=0; i<4; i++){ulongUnion.Byte[i] = (byte)EEPROM.read(eeprom.fireTime + i);}
  settings.fireTime = ulongUnion.val; 
  settings.pyro1Func = (char)EEPROM.read(eeprom.pyro1Func); 
  settings.pyro2Func = (char)EEPROM.read(eeprom.pyro2Func);
  settings.pyro3Func = (char)EEPROM.read(eeprom.pyro3Func);
  settings.pyro4Func = (char)EEPROM.read(eeprom.pyro4Func);
  settings.radioTXenable = (boolean)EEPROM.read(eeprom.radioTXenable);
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
