///HPR Rocket Flight Computer
//Original sketch by SparkyVT
//This is built for the Teensy3.5 board
//-----------Change Log------------
//V4_0 is cross-compatible across hardware and can be mounted in any orientation
//Compatible Sentences: GPGGA, GPRMC, GNGGA, GNRMC
//--------FEATURES----------
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
//-------FUTURE UPGRADES----------
//User adjustable radio bandwidth, & modulation
//Magnetic sensor fusion
//Field Adjustable Radio Frequency
//Ground Station Bluetooth Datalink to smartphone
//Smartphone App
//------TO DO LIST------
//create GPS parsing code to eliminate external library
//create RFM95W/96W code to eliminate external library
//inflight calibration of the magnetometer
//inflight calibration of the highG accelerometer gain
//scalable gyro & accelerometer sensitivity
//pre-allocate SD card memory with new library
//PWM outputs for digital servo control
//Servo control routines for active stabilization
//----EEPROM ALLOCATION:----
//00-05: maximum altitude of last flight
//06-11: accel.bias(X,Y,Z)
//12-17: highG.bias(X,Y,Z)
//18-23: mag.bias(X,Y,Z)
//24-29: IMU to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
//30-35: highG to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
//36-41: IMU-to-highG orientation translation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
//42   : I2C bus
//43   : Pyro 1 Continuity Pin
//44   : Pyro 1 Fire Pin
//45   : Pyro 2 Continuity Pin
//46   : Pyro 2 Fire Pin
//47   : Pyro 3 Continuity Pin
//48   : Pyro 3 Fire Pin
//49   : Pyro 4 Continuity Pin
//50   : Pyro 4 Fire Pin
//51   : Null Continuity / Fire Pin
//52   : Beeper Pin
//53   : Battery Read Pin
//54   : Magnetic Switch Pin (obsolete)
//55   : Test Mode Button Gnd Pin
//56   : Test Mode Read Pin
//57   : radio CS pin
//58   : radio interrupt pin
//59   : radio reset pin
//60   : radio enable pin (optional w/ Adafruit breadkout only)
//61   : Sensor accel / mag 
//62   : Sensor gyro
//63   : Sensor baro
//64   : Sensor highG
//65   : Sensor Radio
//66   : Report Style
//67   : Units Metric or standard
//68-71: Least Squares Estimate of HighG Gain

//-------CODE START--------
#include <SdFat.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
//#include <ADC.h>

//Radio setup
/*#define RF95_FREQ     433.250
#define RFM95_RST     37//OK
#define RFM95_CS      10//OK
#define RFM95_IRQ     32//OK
RH_RF95 rf95(RFM95_CS, RFM95_IRQ);*/

//Teensy 3.5 Hardware Serial for GPS
HardwareSerial HWSERIAL(Serial1);

// GPS Setup
TinyGPSPlus GPS;

//SDIO Setup
SdFatSdioEX SD;
File outputFile;
File settingsFile;

//ADC Setup
//ADC *adc = new ADC(); // adc object;

//GLOBAL VARIABLES
//-----------------------------------------
//Set code version
//-----------------------------------------
const float codeVersion = 4.0;
//-----------------------------------------
//Sensor booleans
//-----------------------------------------
typedef struct{
  byte accel;
  byte gyro;
  byte highG;
  byte baro;
  byte radio;
  boolean status_LSM303 = false;
  boolean status_LSM9DS1 = false;
  boolean status_L3GD20H = false;
  boolean status_MPL3115A2 = false;
  boolean status_BMP180= false;
  boolean status_BMP280 = false;
  boolean status_BMP388 = false;
  boolean status_H3LIS331DL = false;
  boolean status_ADS1115 = false;
  boolean status_ADXL377 = false;
  boolean status_RFM96W = false;
} hardwareSensors;
hardwareSensors sensors;
//-----------------------------------------
//Sensor variables
//-----------------------------------------
byte rawData[12];
typedef struct{
  byte addr;
  byte reg;
  byte i2cBytes;
  byte gainLevel;
  byte maxGainLvl;
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
  int32_t sumX0;
  int32_t sumY0;
  int32_t sumZ0;
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
  boolean silentMode = false; //true turns off beeper
  boolean twoStage = false;
  char rocketName[20] = ""; //Maximum of 20 characters
  char callSign[7]= "";
  char pyro4Func = 'M';
  char pyro3Func = 'A';
  char pyro2Func = 'N';
  char pyro1Func = 'N';
  int max_ang = 45; //degrees
  boolean magSwitchEnable = false;
  boolean radioTXmode = true;//false turns off radio transmissions
  unsigned long setupTime = 5000000UL;
  float TXfreq = 433.250;
  byte TXpwr = 13;
  int gTrigger = 3415; //2.5G trigger
  unsigned long detectLiftoffTime = 500000UL; //0.5s
  unsigned long apogeeDelay = 1000000UL; //1.0s apogee delay
  int Alt_threshold = 120; //120m = 400ft
  unsigned long sustainerFireDelay = 1000000UL; //1.0s
  unsigned long separation_delay = 500000UL; //0.5s
  float mainDeployAlt = 153;//Up to 458m for main deploy
  unsigned long rcd_time = 900000000UL; //15min
  unsigned long fireTime = 500000UL;//0.5s
  boolean quatEnable = false;
  boolean highG3axis = true;
  char reportStyle = 'P';
  char units = 'S';
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
  unsigned long fireStop;
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
int16_t packetnum = 0;
byte dataPacket[81];
byte pktPosn=0;
byte sampNum = 0;
uint16_t radioTime;
unsigned long sampDelay = 50000UL;
unsigned long timeLastSample = 50000UL;
union {
   float GPScoord; 
   byte GPSbyte[4];
} GPSunion;
unsigned long lastTX = 0UL;
unsigned long radioDelay;
unsigned long RDpreLiftoff = 5000000UL;
unsigned long RDinFlight = 200000UL;//5 packets per second
unsigned long RDpostFlight = 10000000UL;
boolean radioTX = false;
boolean onGround = true;
uint8_t radioEvent = 0;
int16_t radioInt;
int16_t radioVel = 0;
//-----------------------------------------
//flight events
//-----------------------------------------
typedef struct{
  boolean preLiftoff = true;
  boolean liftoff = false;
  boolean falseLiftoffCheck = true;
  boolean boosterBurnout = false;
  boolean boosterBurnoutCheck = false;
  boolean boosterSeparation = false;
  boolean sustainerFireCheck = false;
  boolean sustainerFire = false;
  boolean sustainerIgnition = false;
  boolean sustainerBurnout = false;
  boolean airstartIgnition = false;
  boolean airstartBurnout = false;
  boolean apogee = false;
  boolean apogeeFire = false;
  boolean apogeeSeparation = false;
  boolean mainDeploy = false;
  boolean touchdown = false;
  boolean timeOut = false;
  } eventList;
eventList events;
eventList resetEvents;

typedef struct{
  unsigned long timeBoosterBurnout = 0UL;
  unsigned long timeBoosterSeparation = 0UL;
  unsigned long timeSustainerFireCheck = 0UL;
  unsigned long timeSustainerFire = 0UL;
  unsigned long timeSustainerIgnition = 0UL;
  unsigned long timeSustainerBurnout = 0UL;
  unsigned long timeApogee = 0UL;
  unsigned long timeApogeeFire = 0UL;
  unsigned long timeApogeeSeparation = 0UL;
  unsigned long timeMainDeploy = 0UL;
  unsigned long timeTouchdown = 0UL;
  unsigned long timeClock = 0UL;
  unsigned long timeClockPrev = 0UL;
  unsigned long timeCurrent = 0UL;
  unsigned long dt = 0UL;
  long gdt = 0L;
  unsigned long timeGyro = 0UL;
  unsigned long timeGyroClock = 0UL;
  unsigned long timeGyroClockPrev = 0UL;
  unsigned long timeLastEvent = 0UL;
} timerList;
timerList fltTime;
timerList resetFlightTime;

boolean preLiftoff = true;
boolean liftoff = false;
boolean boosterBurnout = false;
boolean boosterBurnoutCheck = false;
boolean boosterSeparation = false;
boolean sustainerFireCheck = false;
boolean sustainerFire = false;
boolean sustainerIgnition = false;
boolean sustainerBurnout = false;
boolean airstartIgnition = false;
boolean airstartBurnout = false;
boolean apogee = false;
boolean apogeeFire = false;
boolean apogeeSeparation = false;
boolean mainDeploy = false;
boolean touchdown = false;
boolean timeOut = false;

boolean rotnOK = true;
boolean altOK = false;
boolean beep = false;
boolean pyroFire = false;
boolean fileClose = false;
boolean contApogee = false;
boolean contMain = false;
boolean contStage = false;
boolean contSep = false;
boolean contAirstart = false;
boolean contError = false;
//-----------------------------------------
//Master timing variables
//-----------------------------------------
unsigned long timeClock = 0UL;
unsigned long timeClockPrev = 0UL;
unsigned long timeCurrent = 0UL;
unsigned long dt = 0UL;
long gdt = 0L;
unsigned long timeGyro = 0UL;
unsigned long timeGyroClock = 0UL;
unsigned long timeGyroClockPrev = 0UL;
unsigned long timeLastEvent = 0UL;
unsigned long boosterBurpTime;
boolean checkFalseTrigger = true;
unsigned long padTime = 0UL;
//-----------------------------------------
//digital accelerometer variables
//-----------------------------------------
int g = 1366;
const float convertG = 66.93989; //66.93989 = HighGgain / digitalGain = 0.049/0.000732
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
float A2D;//12.15139442 = analogGain / digitalGain = 0.0158337/0.000753
union {
   int16_t calValue; 
   byte calByte[2];
} calUnion;
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
boolean newBMP = false;
float seaLevelPressure = 1013.25;
float pressureAvg = 0;
float pressureAvg5[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float pressureSum = 0.0;
byte pressurePosn = 0;
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
byte rawAltPosn = 0;
byte altAvgPosn = 0;
float altAvgBuff[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
unsigned long baroTimeBuff[10] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL};
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
unsigned long magTime = 100000UL;
unsigned long magCounter = 0UL;
int16_t magTrigger = 0;
byte magCalibrate = 0;
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
int rollZ = 0;
const long oneDeg = 14285714L; //(long)((float)1000000/(float)gyroDegLSB);//oneDeg = mln/0.070 = 14285714L;
const long oneTenthDeg = oneDeg/10;//oneTenthDeg = oneDeg/10 = 1428571L
const float degRad = 57.296; //degrees per radian
const float mlnth = 0.000001;
//-----------------------------------------
//DCM Integration variables
//-----------------------------------------
float cosZ = 1.0;
float sinZ = 0.0;
float PrevCosZ = 1.0;
float PrevSinZ = 0.0;
float tanPitch;
float tanYaw;
int counterSign = 1;
boolean calcOffVert = false;
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
unsigned long rotnRate = 5000UL;//100 updates per second
//-----------------------------------------
//velocity calculation variables
//-----------------------------------------
float accelVel = 0.0;
float accelAlt = 0.0;
float maxVelocity = 0.0;
//-----------------------------------------
//beeper variables
//-----------------------------------------
unsigned long setupTime = 0UL;
byte beep_counter = 0;
unsigned long beep_delay;
int beepCode = 0;
const unsigned long beep_len = 100000UL;
unsigned long timeBeepStart;
unsigned long timeLastBeep;
const unsigned long short_beep_delay = 100000UL;
const unsigned long long_beep_delay = 800000UL;
float unitConvert = 3.2808F;
//-----------------------------------------
//SD card writing variables
//-----------------------------------------
int strPosn = 0;
boolean syncCard = false;
const byte decPts = 2;
const byte base = 10;
char dataString[256];
byte maxAltDigits[6];
byte maxVelDigits[4];
byte voltageDigits[2];
byte altDigits = 6;
byte velDigits = 4;
byte n = 1;
float voltage = 0.0F;
uint16_t voltReading;
int32_t voltSum = 0;
boolean reportCode = true;//true = report max altitude, false = report max velocity
byte postFlightCode = 0;
float adcConvert = 0.000015259;
const char cs = ',';
const uint16_t ADCmaxValue = 65536;
const uint16_t ADCmidValue = 32768;
const uint32_t ADCdataRate = 2000;
uint32_t lastHighG = 0UL;
//-----------------------------------------
//GPS Variables
//-----------------------------------------
int maxGPSalt = 0;
float baseGPSalt = 0.0;
float GPSavgAlt[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float GPSaltSum = 0.0;
byte GPSposn = 0;
boolean gpsWrite = false;
boolean gpsTransmit = false;
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
boolean configGPSdefaults = true;
boolean configGPSflight = false;
unsigned long timeLastGPS = 0UL;
uint16_t fixCount = 0;
boolean GPSpsm = false;
//-----------------------------------------
//debug
//-----------------------------------------
long debugStart;
long debugTime;

void setup(void) {

  Serial.begin(9600);
  
  //Start Harware Serial communication
  HWSERIAL.begin(9600);
  
  //Start SDIO Comms with the SD card
  if(!SD.begin()){Serial.println(F("SD card failed!"));}
  
  //If an EEPROM settings file exists, open it and copy the values into EEPROM
  byte kk;
  int8_t ii;
  if(SD.exists("EEPROMsettings.txt")){
    Serial.println(F("EEPROM file found!  Writing initial EEPROM Settings..."));
    settingsFile.open("EEPROMsettings.txt", O_READ);
    //00-05: maximum altitude of last flight
    //06-11: accel.bias(X,Y,Z)
    //12-17: highG.bias(X,Y,Z)
    //18-23: mag.bias(X,Y,Z)
    //24-29: IMU to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    //30-35: highG to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
    //36-41: IMU-to-highG orientation translation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
    //42   : I2C bus
    //43   : Pyro 1 Continuity Pin
    //44   : Pyro 1 Fire Pin
    //45   : Pyro 2 Continuity Pin
    //46   : Pyro 2 Fire Pin
    //47   : Pyro 3 Continuity Pin
    //48   : Pyro 3 Fire Pin
    //49   : Pyro 4 Continuity Pin
    //50   : Pyro 4 Fire Pin
    //51   : Null Continuity / Fire Pin
    //52   : Beeper Pin
    //53   : Battery Read Pin
    //54   : Magnetic Switch Pin
    //55   : Test Mode Button Gnd Pin
    //56   : Test Mode Read Pin
    //57   : radio CS pin
    //58   : radio interrupt pin
    //59   : radio reset pin
    //60   : radio enable pin (optional w/ Adafruit breadkout only)
    //61   : Sensor accel / mag 
    //62   : Sensor gyro
    //63   : Sensor baro
    //64   : Sensor highG
    //65   : Sensor Radio
    //66   : Report Style
    //67   : Units Metric or standard
    //68-71: Least Squares Estimate of HighG Gain
    for(byte j = 42; j < 66; j++){
      kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));
      EEPROM.update(j, kk);}
    magCalibrate = (byte)parseNextVariable(true);
    parseNextVariable(false);
    EEPROM.update(66, (char)dataString[0]);
    parseNextVariable(false);
    EEPROM.update(67, (char)dataString[0]);
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    EEPROM.update(36, ii); EEPROM.update(37, (char)dataString[1]);
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    EEPROM.update(38, ii); EEPROM.update(39, (char)dataString[1]);
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    EEPROM.update(40, ii); EEPROM.update(41, (char)dataString[1]);
    GPSunion.GPScoord = parseNextVariable(true);
    for(byte i = 0; i < 4; i++){EEPROM.update(68+i, GPSunion.GPSbyte[i]);}
    settingsFile.close();
    SD.remove("EEPROMsettings.txt");
    Serial.println(F("Complete!"));}

  //Read pin settings from EEPROM
  byte j = 42;
  Serial.print(F("Reading EEPROM..."));
  pins.i2c = EEPROM.read(j);j++;
  pins.pyro1Cont = EEPROM.read(j);j++;
  pins.pyro1Fire = EEPROM.read(j);j++;
  pins.pyro2Cont = EEPROM.read(j);j++;
  pins.pyro2Fire = EEPROM.read(j);j++;
  pins.pyro3Cont = EEPROM.read(j);j++;
  pins.pyro3Fire = EEPROM.read(j);j++;
  pins.pyro4Cont = EEPROM.read(j);j++;
  pins.pyro4Fire = EEPROM.read(j);j++;
  pins.nullCont = EEPROM.read(j);j++;
  pins.nullFire = pins.nullCont+1;
  pins.beep = EEPROM.read(j);j++;
  pins.batt = EEPROM.read(j);j++;
  pins.mag = EEPROM.read(j);j++;
  pins.testGnd = EEPROM.read(j);j++;
  pins.testRead = EEPROM.read(j);j++;
  pins.radioCS = EEPROM.read(j);j++;
  pins.radioIRQ = EEPROM.read(j);j++;
  pins.radioRST = EEPROM.read(j);j++;
  pins.radioEN = EEPROM.read(j);j++;
  sensors.accel = EEPROM.read(j);j++;
  sensors.gyro = EEPROM.read(j);j++;
  sensors.highG = EEPROM.read(j);j++;
  sensors.baro = EEPROM.read(j);j++;
  sensors.radio = EEPROM.read(j);j++;
  settings.reportStyle = (char)EEPROM.read(j);j++;
  settings.units = (char)EEPROM.read(j);j++;
  Serial.println(F("complete!"));

  //Set the unit conversion factor SI or Metric
  if(settings.units == 'M'){unitConvert = 1.0;}

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
  pinMode(pins.mag, INPUT_PULLUP); 
  //Set the pyro firing pins to LOW for safety
  digitalWrite(pins.pyro1Fire, LOW);
  digitalWrite(pins.pyro2Fire, LOW);
  digitalWrite(pins.pyro3Fire, LOW);
  digitalWrite(pins.pyro4Fire, LOW);
  Serial.println("Set Pins Low");
  
  //check if the test mode button is being held
  digitalWrite(pins.testGnd, LOW);
  delay(50);
  if(digitalRead(pins.testRead) == LOW){settings.testMode = true; Serial.println(F("Test Mode Confirmed"));}

  //setup the ADC
  analogReadResolution(16);
//  adc->adc0->setAveraging(32); // set number of averages (was 10)
//  adc->adc0->setResolution(16); // set bits of resolution
//  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
//  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
    
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
  beginHighG('S');
  beginBaro();

  //Start the radio
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
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
    
  if(settings.testMode){Serial.println(F("Reading User Settings from SD Card"));}
  //Open the settings file
  settingsFile.open("Settings.txt", O_READ);

  //Read in the user defined variables
  parseNextVariable(false);n=0;
  while (dataString[n]!='\0'){settings.rocketName[n] = dataString[n];n++;}settings.rocketName[n]='\0';n=0;
  parseNextVariable(false);
  while (dataString[n]!='\0'){settings.callSign[n] = dataString[n];n++;}settings.callSign[n]='\0';n=0;
  settings.magSwitchEnable = (boolean)(parseNextVariable(true));
  settings.radioTXmode = (boolean)parseNextVariable(true);
  settings.TXpwr = (byte)(parseNextVariable(true));
  settings.TXfreq = (float)(parseNextVariable(true));
  settings.silentMode = (boolean)(parseNextVariable(true));
  settings.setupTime = (unsigned long)(parseNextVariable(true)*1000UL);
  settings.gTrigger = (int)(parseNextVariable(true)*g);
  settings.detectLiftoffTime = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.apogeeDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.mainDeployAlt = (float)(parseNextVariable(true)/unitConvert);
  settings.rcd_time = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.fireTime = (unsigned long) (parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.pyro4Func = dataString[0];
  parseNextVariable(false); settings.pyro3Func = dataString[0];
  parseNextVariable(false); settings.pyro2Func = dataString[0];
  parseNextVariable(false); settings.pyro1Func = dataString[0];
  settings.twoStage = (boolean)parseNextVariable(true);
  settings.sustainerFireDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.separation_delay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.Alt_threshold = (int)(parseNextVariable(true)/unitConvert);
  settings.max_ang = (int)parseNextVariable(true)*10;
  settings.quatEnable = (boolean)parseNextVariable(true);
  settings.highG3axis = (boolean)parseNextVariable(true);

  //close the settings file
  settingsFile.close();

  //configure pyro outupts
  pyro1.func = settings.pyro1Func; pyro1.contPin = pins.pyro1Cont; pyro1.firePin = pins.pyro1Fire; pyro1.fireStatus = false; pyro1.fireStop = 0UL;
  pyro2.func = settings.pyro2Func; pyro2.contPin = pins.pyro2Cont; pyro2.firePin = pins.pyro2Fire; pyro2.fireStatus = false; pyro2.fireStop = 0UL;
  pyro3.func = settings.pyro3Func; pyro3.contPin = pins.pyro3Cont; pyro3.firePin = pins.pyro3Fire; pyro3.fireStatus = false; pyro3.fireStop = 0UL;
  pyro4.func = settings.pyro4Func; pyro4.contPin = pins.pyro4Cont; pyro4.firePin = pins.pyro4Fire; pyro4.fireStatus = false; pyro4.fireStop = 0UL;

  if(settings.testMode){
  Serial.print(F("Pyro 4 Function: ")); Serial.println(pyro4.func);
  Serial.print(F("Pyro 3 Function: ")); Serial.println(pyro3.func);
  Serial.print(F("Pyro 2 Function: ")); Serial.println(pyro2.func);
  Serial.print(F("Pyro 1 Function: ")); Serial.println(pyro1.func);}
  
  //safety override of manual variables
  if (settings.gTrigger < 1.5 * g) {settings.gTrigger = 1.5 * g;} //min 1.5G trigger
  if (settings.gTrigger > 5 * g) {settings.gTrigger = 5 * g;} //max 5G trigger
  if (settings.detectLiftoffTime < 100000UL) {settings.detectLiftoffTime = 100000UL;} //.1s min gTrigger detection
  if (settings.detectLiftoffTime > 1000000UL) {settings.detectLiftoffTime = 1000000UL;} //1s max gTrigger detection
  if (settings.apogeeDelay > 5000000UL) {settings.apogeeDelay = 5000000UL;} //5s max apogee delay
  if (settings.fireTime > 1000000UL) {settings.fireTime = 1000000UL;} //1s max firing length
  if (settings.mainDeployAlt > 458){settings.mainDeployAlt = 458;}//max of 1500 ft
  if (settings.mainDeployAlt < 30) {settings.mainDeployAlt = 30;}//minimum of 100ft
  if (settings.sustainerFireDelay > 8000000UL){settings.sustainerFireDelay = 8000000UL;}//maximum 8s 2nd stage ignition delay
  if (settings.separation_delay > 3000000UL){settings.separation_delay = 3000000UL;}//max 3s booster separation delay after burnout
  if (settings.Alt_threshold < 91){settings.Alt_threshold = 91;}//minimum 100ft threshold
  if (settings.max_ang > 450){settings.max_ang = 450;}//maximum 90 degree off vertical
  if (settings.rcd_time < 300000000UL){settings.rcd_time = 300000000UL;}//min 5min of recording time
  if (settings.fireTime < 200000UL){settings.fireTime = 200000UL;}//min 0.2s of firing time
  if (settings.fireTime > 1000000UL){settings.fireTime = 1000000UL;}//max 1.0s of firing time
  if (settings.setupTime > 60000UL) {settings.setupTime = 60000UL;}//max 60 seconds from power-on to preflight start
  if (settings.setupTime < 3000UL) {settings.setupTime = 3000UL;}//min 3 seconds of setup time
  if (settings.TXpwr > 23){settings.TXpwr = 23;}
  if (settings.TXpwr < 1){settings.TXpwr = 1;}

  //check for silent mode
  if(settings.testMode && settings.silentMode){pins.beep = pins.nullCont; Serial.println(F("Silent Mode Confirmed"));}
  
  //check for disabling of the telemetry
  if(settings.testMode && settings.silentMode && !settings.radioTXmode){pins.beep = 13; pinMode(pins.beep, OUTPUT);}
  if(!settings.radioTXmode){
    if(pins.radioEN != pins.nullCont){pinMode(pins.radioEN, OUTPUT);digitalWrite(pins.radioEN, LOW);}
    if(settings.testMode){Serial.println(F("Telemetry OFF!"));}}
  if(settings.radioTXmode){
    //Set the radio output power & frequency
    rf95.setTxPower(settings.TXpwr, false);//23 max setting; 20mW=13dBm, 30mW=15dBm, 50mW=17dBm, 100mW=20dBm
    rf95.setFrequency(settings.TXfreq);
    radioDelay = RDpreLiftoff;}
  
  //signal if in test-mode
  if (settings.testMode){

    Serial.println(F("Signaling Test Mode"));
    beep_counter = 0;
    beep_delay = long_beep_delay;
    while(beep_counter < 8){
      
      timeClock = micros();

      //Look for the user to release the button
      if(digitalRead(pins.testRead) == HIGH){settings.testMode = false;delay(10);}

      //Look for the user to put it into calibration mode
      if(digitalRead(pins.testRead) == LOW && !settings.testMode){
        settings.calibrationMode = true;
        settings.testMode = true;
        beep_counter = 8;
        digitalWrite(pins.beep, LOW);
        beep = false;}

      //starts the beep
      if (!beep && timeClock - timeLastBeep > beep_delay){
          digitalWrite(pins.beep, HIGH);
          timeBeepStart = timeClock;
          beep = true;
          beep_counter++;}
      
      //stops the beep
      if(beep && (timeClock - timeBeepStart > 500000UL)){
        digitalWrite(pins.beep, LOW);
        timeBeepStart = 0UL;
        timeLastBeep = timeClock;
        beep = false;}
      }//end while

    //Reset variables
    beep_counter = 0;
    timeBeepStart = 0UL;
    timeLastBeep = 0UL;
    timeClock = 0UL;
    if(!settings.calibrationMode){settings.testMode = true;}}//end testMode

  //calibration mode
  if(settings.calibrationMode){
    
    Serial.println(F("Accelerometer Calibration Mode Confirmed. Please hold altimeter vertical and still"));
    
    for (byte i = 1; i < 20; i++){
      digitalWrite(pins.beep, HIGH);
      delay(250);
      digitalWrite(pins.beep, LOW);
      delay(250);}

    digitalWrite(pins.beep, HIGH);

    //Reset to default orientation with IMU axes aligned to world axis
    accel.orientX = mag.orientX = gyro.orientX = 'X';
    accel.orientY = mag.orientY = gyro.orientY = 'Y';
    accel.orientZ = mag.orientZ = gyro.orientZ = 'Z';
    accel.ptrX = &accel.rawX;
    accel.ptrY = &accel.rawY;
    accel.ptrZ = &accel.rawZ;
    mag.ptrX = &mag.rawX;
    mag.ptrY = &mag.rawY;
    mag.ptrZ = &mag.rawZ;
    gyro.ptrX = &gyro.rawX;
    gyro.ptrY = &gyro.rawY;
    gyro.ptrZ = &gyro.rawZ;

    //Align the highG accelerometer to the IMU
    highG.dirX = (int8_t)EEPROM.read(36);
    if((char)EEPROM.read(37)=='X'){highG.ptrX = &highG.rawX; highG.orientX = 'X';}
    if((char)EEPROM.read(37)=='Y'){highG.ptrX = &highG.rawY; highG.orientX = 'Y';}
    if((char)EEPROM.read(37)=='Z'){highG.ptrX = &highG.rawZ; highG.orientX = 'Z';}
    highG.dirY = (int8_t)EEPROM.read(38);
    if((char)EEPROM.read(39)=='X'){highG.ptrY = &highG.rawX; highG.orientY = 'X';}
    if((char)EEPROM.read(39)=='Y'){highG.ptrY = &highG.rawY; highG.orientY = 'Y';}
    if((char)EEPROM.read(39)=='Z'){highG.ptrY = &highG.rawZ; highG.orientY = 'Z';}
    highG.dirZ = (int8_t)EEPROM.read(40);
    if((char)EEPROM.read(41)=='X'){highG.ptrZ = &highG.rawX; highG.orientZ = 'X';}
    if((char)EEPROM.read(41)=='Y'){highG.ptrZ = &highG.rawY; highG.orientZ = 'Y';}
    if((char)EEPROM.read(41)=='Z'){highG.ptrZ = &highG.rawZ; highG.orientZ = 'Z';}

    //align the IMU
    switch (sensors.accel){
  
      //LSM303 & L3GD20H
      case 1:
        //orient to common axis
        accel.dirX = mag.dirX = gyro.dirX = 1;
        accel.dirY = mag.dirY = gyro.dirY = 1;
        accel.dirZ = mag.dirZ = gyro.dirZ = 1;
        break;
        
      //LSM9DS1
      case 2: 
        //orient to common axis
        accel.dirX = mag.dirX = gyro.dirX = -1;
        accel.dirY = mag.dirY = gyro.dirY = 1;
        accel.dirZ = mag.dirZ = gyro.dirZ = 1;
        highG.dirX *= -1;
        break;}

    Serial.println(F("Calibrating..."));

    accel.biasX = accel.biasY = accel.biasZ = 0;
    accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
    highG.biasX = highG.biasY = highG.biasZ = 0;
    highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0; 

    for (byte i = 1; i < 101; i++){
      getAccel();
      getHighG(true);
      Serial.print(F("Accel X,Y,Z: "));Serial.print(accel.rawX);Serial.print(',');Serial.print(accel.rawY);Serial.print(',');Serial.println(accel.rawZ);
      Serial.print(F("highG X,Y,Z: "));Serial.print(highG.rawX);Serial.print(',');Serial.print(highG.rawY);Serial.print(',');Serial.println(highG.rawZ);
      accel.sumX0 += accel.rawX;
      accel.sumY0 += accel.rawY;
      accel.sumZ0 += accel.rawZ;
      highG.sumX0 += highG.rawX;
      highG.sumY0 += highG.rawY;
      highG.sumZ0 += highG.rawZ;
      delay(300);}
      
    //calculate the bias
    accel.biasX = (int)(accel.sumX0 / 100);
    accel.biasY = (int)(accel.sumY0 / 100);
    accel.biasZ = (int)(accel.sumZ0 / 100);
    highG.biasX = (int)(highG.sumX0 / 100);
    highG.biasY = (int)(highG.sumY0 / 100);
    highG.biasZ = (int)(highG.sumZ0 / 100);

    //reset the counters
    accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
    highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0;
    
    //determine the altimeter orientation
    setOrientation();

    //Store bias in EEPROM    
    writeCalibration(accel.biasX, 6);
    writeCalibration(accel.biasY, 8);
    writeCalibration(accel.biasZ, 10);
    writeCalibration(highG.biasX, 12);
    writeCalibration(highG.biasY, 14);
    writeCalibration(highG.biasZ, 16);
        
    digitalWrite(pins.beep, LOW);
    Serial.println(F("Calibration complete!  Calculated values are:"));
    Serial.print(F("accel.biasX: "));Serial.println(accel.biasX);
    Serial.print(F("accel.biasY: "));Serial.println(accel.biasY);
    Serial.print(F("accel.biasZ: "));Serial.println(accel.biasZ);
    Serial.print(F("highG.biasX: "));Serial.println(highG.biasX);
    Serial.print(F("highG.biasY: "));Serial.println(highG.biasY);
    Serial.print(F("highG.biasZ: "));Serial.println(highG.biasZ);
    }//end calibration mode

  //Calibrate the magnetometer
  if(magCalibrate == 1){

    //reset the magnetometer gain
    setMagGain(false);

    //Clear out the FIFO
    getMag();
    delay(100);
    getMag();
    
    Serial.println(F("Magnetometer Calibration Mode"));
    //give a 20 second audible warning
    for(byte i = 0; i < 20; i++){
      digitalWrite(pins.beep, HIGH);
      delay(250);
      digitalWrite(pins.beep, LOW);
      delay(750);}

    //give a long tone while sampling
    digitalWrite(pins.beep, HIGH);

    //initialize the offsets
    int maxMagX, maxMagY, maxMagZ;
    int minMagX, minMagY, minMagZ;
    boolean initMag = true;
    
    Serial.println(F("Sampling Magnetometer"));    
    //sample at 10Hz for 30seconds
    for(int i = 0; i < 300; i++){
      getMag();
      Serial.print(mag.rawX);Serial.print(',');Serial.print(mag.rawY);Serial.print(',');Serial.println(mag.rawZ);
      if(initMag){
        maxMagX = minMagX = mag.rawX;
        maxMagY = minMagY = mag.rawY;
        maxMagZ = minMagZ = mag.rawZ;
        initMag = false;}
      if(mag.rawX > maxMagX){maxMagX = mag.rawX;}
      if(mag.rawY > maxMagY){maxMagY = mag.rawY;}
      if(mag.rawZ > maxMagZ){maxMagZ = mag.rawZ;}
      if(mag.rawX < minMagX){minMagX = mag.rawX;}
      if(mag.rawY < minMagY){minMagY = mag.rawY;}
      if(mag.rawZ < minMagZ){minMagZ = mag.rawZ;}
      delay(100);}

    //Calculate the bias
    Serial.print("minMagX: ");Serial.print(minMagX);Serial.print(", maxMagX: ");Serial.println(maxMagX);
    Serial.print("minMagY: ");Serial.print(minMagY);Serial.print(", maxMagY: ");Serial.println(maxMagY);
    Serial.print("minMagZ: ");Serial.print(minMagZ);Serial.print(", maxMagZ: ");Serial.println(maxMagZ);
    mag.biasX = (int16_t)(minMagX + (maxMagX - minMagX)/2);
    mag.biasY = (int16_t)(minMagY + (maxMagY - minMagY)/2);
    mag.biasZ = (int16_t)(minMagZ + (maxMagZ - minMagZ)/2);

    //Write to EEPROM
    writeCalibration(mag.biasX, 18);
    writeCalibration(mag.biasY, 20);
    writeCalibration(mag.biasZ, 22);
    
    Serial.println(F("Magnetometer Calibration Complete!"));
  }//end magCalibrate

  //read the bias from EEPROM  
  calUnion.calByte[0]=EEPROM.read(6); calUnion.calByte[1]=EEPROM.read(7);
  accel.biasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(8); calUnion.calByte[1]=EEPROM.read(9);
  accel.biasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(10); calUnion.calByte[1]=EEPROM.read(11);
  accel.biasZ = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(12); calUnion.calByte[1]=EEPROM.read(13);
  highG.biasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(14); calUnion.calByte[1]=EEPROM.read(15);
  highG.biasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(16); calUnion.calByte[1]=EEPROM.read(17);
  highG.biasZ = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(18); calUnion.calByte[1]=EEPROM.read(19);
  mag.biasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(20); calUnion.calByte[1]=EEPROM.read(21);
  mag.biasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(22); calUnion.calByte[1]=EEPROM.read(23);
  mag.biasZ = calUnion.calValue;

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
    Serial.print(F("mag.biasZ: "));Serial.println(mag.biasZ);}

  //read the orientation variables from EEPROM
  //IMU
  j = 24;
  accel.dirX = mag.dirX = gyro.dirX = (int8_t)EEPROM.read(j); j++;
  accel.orientX = mag.orientX = gyro.orientX = (char)EEPROM.read(j); j++;
  if(accel.orientX == 'X'){accel.ptrX = &accel.rawX; mag.ptrX = &mag.rawX; gyro.ptrX = &gyro.rawX;}
  if(accel.orientX == 'Y'){accel.ptrX = &accel.rawY; mag.ptrX = &mag.rawY; gyro.ptrX = &gyro.rawY;}
  if(accel.orientX == 'Z'){accel.ptrX = &accel.rawZ; mag.ptrX = &mag.rawZ; gyro.ptrX = &gyro.rawZ;}
  accel.dirY = mag.dirY = gyro.dirY = (int8_t)EEPROM.read(j); j++;
  accel.orientY = mag.orientY = gyro.orientY = (char)EEPROM.read(j); j++;
  if(accel.orientY == 'X'){accel.ptrY = &accel.rawX; mag.ptrY = &mag.rawX; gyro.ptrY = &gyro.rawX;}
  if(accel.orientY == 'Y'){accel.ptrY = &accel.rawY; mag.ptrY = &mag.rawY; gyro.ptrY = &gyro.rawY;}
  if(accel.orientY == 'Z'){accel.ptrY = &accel.rawZ; mag.ptrY = &mag.rawZ; gyro.ptrY = &gyro.rawZ;}
  accel.dirZ = mag.dirZ = gyro.dirZ = (int8_t)EEPROM.read(j); j++;
  accel.orientZ = mag.orientZ = gyro.orientZ = (char)EEPROM.read(j); j++;
  if(accel.orientZ == 'X'){accel.ptrZ = &accel.rawX; mag.ptrZ = &mag.rawX; gyro.ptrZ = &gyro.rawX;}
  if(accel.orientZ == 'Y'){accel.ptrZ = &accel.rawY; mag.ptrZ = &mag.rawY; gyro.ptrZ = &gyro.rawY;}
  if(accel.orientZ == 'Z'){accel.ptrZ = &accel.rawZ; mag.ptrZ = &mag.rawZ; gyro.ptrZ = &gyro.rawZ;}
  //highG
  highG.dirX = (int8_t)EEPROM.read(j); j++;
  highG.orientX = (char)EEPROM.read(j); j++;
  if(highG.orientX == 'X'){highG.ptrX = &highG.rawX;}
  if(highG.orientX == 'Y'){highG.ptrX = &highG.rawY;}
  if(highG.orientX == 'Z'){highG.ptrX = &highG.rawZ;}
  highG.dirY = (int8_t)EEPROM.read(j); j++;
  highG.orientY = (char)EEPROM.read(j); j++;
  if(highG.orientY == 'X'){highG.ptrY = &highG.rawX;}
  if(highG.orientY == 'Y'){highG.ptrY = &highG.rawY;}
  if(highG.orientY == 'Z'){highG.ptrY = &highG.rawZ;}
  highG.dirZ = (int8_t)EEPROM.read(j); j++;
  highG.orientZ = (char)EEPROM.read(j); j++;
  if(highG.orientZ == 'X'){highG.ptrZ = &highG.rawX;}
  if(highG.orientZ == 'Y'){highG.ptrZ = &highG.rawY;}
  if(highG.orientZ == 'Z'){highG.ptrZ = &highG.rawZ;}
  //display values from EEPROM
  if(settings.testMode){
    Serial.print(F("IMU.X: "));Serial.print((accel.dirX == 1) ? '+' : '-');Serial.println(accel.orientX);
    Serial.print(F("IMU.Y: "));Serial.print((accel.dirY == 1) ? '+' : '-');Serial.println(accel.orientY);
    Serial.print(F("IMU.Z: "));Serial.print((accel.dirZ == 1) ? '+' : '-');Serial.println(accel.orientZ);
    Serial.print(F("highG.X: "));Serial.print((highG.dirX == 1) ? '+' : '-');Serial.println(highG.orientX);
    Serial.print(F("highG.Y: "));Serial.print((highG.dirY == 1) ? '+' : '-');Serial.println(highG.orientY);
    Serial.print(F("highG.Z: "));Serial.print((highG.dirZ == 1) ? '+' : '-');Serial.println(highG.orientZ);}

  //restart the highG accelerometer at the higher rate
  beginHighG('F');

  //Initialize the base altitude average
  for(byte i = 0; i <= (byte)(sizeof(baseAltBuff)/sizeof(baseAltBuff[0])); i++){baseAltBuff[i] = 0.0F;}

  //Overrides for bench test mode
  if (settings.testMode) {
    rf95.setTxPower(10, false);//10% power, or 20mW
    settings.detectLiftoffTime = 10000UL; //0.01s
    settings.setupTime = 3000UL; //3s startup time
    settings.apogeeDelay = 1000000UL; //1s apogee delay
    settings.rcd_time = 15000000UL; //15s record time
    settings.gTrigger = (int)(1.5*g); //1.5G trigger
    maxAltitude = 11101/unitConvert;
    maxVelocity = 202/unitConvert;
    RDpreLiftoff = 1000000UL;
    RDpostFlight = 1000000UL;
    radioDelay = RDpreLiftoff;
    settings.magSwitchEnable = false;
    //g += (abs(accel.biasX) + 50);
    }

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
  outputFile.println(F(",accelX,accelY,accelZ,gyroX,gyroY,gyroZ,highGx,highGy,highGz,smoothHighGz,roll,yaw,pitch,offVert,intVel,intAlt,fltEvents,pyroCont,pyroFire,pyroPin,baroAlt,altMoveAvg,baroVel,baroPress,baroTemp,battVolt,magX,magY,magZ,gpsLat,gpsLon,gpsSpeed,gpsAlt,gpsAngle,gpsSatellites,radioPacketNum"));
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
    if(pyro1.func == 'M'){contMain = true;}
    if(pyro1.func == 'A'){contApogee = true;}
    if(pyro1.func == 'I'){contStage = true;}
    if(pyro1.func == 'B'){contSep = true;}
    if(pyro1.func == 'S'){contAirstart = true;}}
  if(pyro2.contStatus){
    if(pyro2.func == 'M'){contMain = true;}
    if(pyro2.func == 'A'){contApogee = true;}
    if(pyro2.func == 'I'){contStage = true;}
    if(pyro2.func == 'B'){contSep = true;}
    if(pyro2.func == 'S'){contAirstart = true;}}
  if(pyro3.contStatus){
    if(pyro3.func == 'M'){contMain = true;}
    if(pyro3.func == 'A'){contApogee = true;}
    if(pyro3.func == 'I'){contStage = true;}
    if(pyro3.func == 'B'){contSep = true;}
    if(pyro3.func == 'S'){contAirstart = true;}}
  if(pyro4.contStatus){
    if(pyro4.func == 'M'){contMain = true;}
    if(pyro4.func == 'A'){contApogee = true;}
    if(pyro4.func == 'I'){contStage = true;}
    if(pyro4.func == 'B'){contSep = true;}
    if(pyro4.func == 'S'){contAirstart = true;}}
  
  //Report single-stage pre-flight status
  if (!settings.twoStage){
    if (contMain && contApogee) {beepCode = 3;}
    else if (contMain){beepCode = 2;}
    else if (contApogee) {beepCode = 1;}
    else {beepCode = 4;}
    postFlightCode = 1;}

  //Report two-stage pre-flight status
  if (settings.twoStage){
    if (contSep && contStage && contApogee && contMain) {beepCode = 4;}
    else {
      contError = true;
      if (!contSep) {beepCode = 1;}
      else if(!contStage) {beepCode = 2;}
      else if (!contApogee) {beepCode = 3;}
      else if (!contMain){beepCode = 5;}}}

  //Change if Marsa style is desired
  if(settings.reportStyle == 'M'){
    beepCode = 2;
    if(!pyro1.contStatus && pyro1.func != 'N'){beepCode = 1;}
    if(!pyro2.contStatus && pyro2.func != 'N'){beepCode = 1;}
    if(!pyro3.contStatus && pyro3.func != 'N'){beepCode = 1;}
    if(!pyro4.contStatus && pyro4.func != 'N'){beepCode = 1;}}
  if(settings.testMode){Serial.print(F("Reporting continuity: "));Serial.println(beepCode);}
  
  //set the beep delay and preflight beep code
  beep_delay = long_beep_delay;
  
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
  //sample the sensors 100 times over 3 seconds to determine the offsets and initial values
  gyro.biasX = 0;
  gyro.biasY = 0;
  gyro.biasZ = 0;
  //clear out the buffer
  getHighG(true);
  for (byte i = 1; i < 101; i++) { 
    //get a sensor event
    getAccel();
    getGyro();
    getHighG(true);
    getMag();
    Serial.print("Gyro: ");Serial.print(gyro.x);Serial.print(',');Serial.print(gyro.y);Serial.print(',');Serial.println(gyro.z);
    Serial.print("Accel: ");Serial.print(accel.x);Serial.print(',');Serial.print(accel.y);Serial.print(',');Serial.println(accel.z);
    Serial.print("HighG: ");Serial.print(highG.x);Serial.print(',');Serial.print(highG.y);Serial.print(',');Serial.println(highG.z);
    Serial.print("Mag: ");Serial.print(mag.x);Serial.print(',');Serial.print(mag.y);Serial.print(',');Serial.println(mag.z);
    
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
        
    //add up the magnetometer samples
    mag.sumX0 += mag.x;
    mag.sumY0 += mag.y;
    mag.sumZ0 += mag.z;

    //sample over a period of 3 seconds
    delay(30);}

  //Divide by 100 to set the average of 100 samples
  gyro.biasX = gyro.sumX0 / 100;
  gyro.biasY = gyro.sumY0 / 100;
  gyro.biasZ = gyro.sumZ0 / 100;
  accel.x0 = (int)(accel.sumX0 / 100);
  accel.y0 = (int)(accel.sumY0 / 100);
  accel.z0 = (int)(accel.sumZ0 / 100);
  highG.x0 = (int)(highG.sumX0 / 100);
  highG.y0 = (int)(highG.sumY0 / 100);
  highG.z0 = (int)(highG.sumZ0 / 100);
  mag.x0 = (int)(mag.sumX0 / 100);
  mag.y0 = (int)(mag.sumY0 / 100);
  mag.z0 = (int)(mag.sumZ0 / 100);
  if(settings.testMode){
    Serial.println(F("Sampling complete"));
    Serial.print(F("Gyro Offsets: "));
    Serial.print(gyro.biasX);Serial.print(F(", "));
    Serial.print(gyro.biasY);Serial.print(F(", "));
    Serial.println(gyro.biasZ);}

  //Calibrate the analog accelerometer to the digital one
  A2D = highG.gainZ / accel.gainZ;
  if(settings.testMode){
    Serial.println(F("Re-calibrating high-G accelerometer..."));
    Serial.print(F("Old Bias: "));Serial.println(highG.biasX);
    Serial.print(F("HighG.Z0: "));Serial.println(highG.z0);}
  if(highG.orientZ == 'X'){highG.biasX += highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));}
  if(highG.orientZ == 'Y'){highG.biasY += highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));}
  if(highG.orientZ == 'Z'){highG.biasZ += highG.dirZ*(highG.z0 - (int)((float)accel.z0 / (float)A2D));}
  //highG.biasX = highGx0 - (int)((float)accel.x0 / (float)A2D) - 27;//old formula
  if(settings.testMode){Serial.print(F("New Bias: "));Serial.println(highG.biasX);}
  
  //Compute the acceleromter based rotation angle
  if (accel.y0 >= 0) {yawY0 = asin(min(1, (float)accel.y0 / (float)g)) * degRad;}
  else {yawY0 = asin(max(-1, (float)accel.y0 / (float)g)) * degRad;}

  if (accel.x0 >= 0) {pitchX0 = asin(min(1, (float)accel.x0 / (float)g)) * degRad;}
  else {pitchX0 = asin(max(-1, (float)accel.x0 / (float)g)) * degRad;}

  if(settings.quatEnable){
    //update quaternion
    if(settings.testMode){Serial.println(F("Quaternions Enabled!"));}
    getQuatRotn(pitchX0/degRad, yawY0/degRad, 0);}
  else{ 
    //Initialize the rotation angles
    yawY = int(yawY0*(float)10);
    pitchX = int(pitchX0*(float)10);
    offVert = (int16_t)(atan(pow(pow(tan(pitchX0/degRad),2) + pow(tan(yawY0/degRad),2), 0.5)) * degRad * 10);}
  if(settings.testMode){
    Serial.println(F("Rotation Computation Complete"));
    Serial.print(F("Yaw: "));Serial.println(yawY0, 2);
    Serial.print(F("Pitch: "));Serial.println(pitchX0, 2);
    Serial.print(F("Off Vertical: "));Serial.println((float)(offVert/10), 2);}  

  //Read the battery voltage
  voltReading = analogRead(pins.batt);
  voltage = (float)(voltReading)*3.3*3.2*adcConvert;
  if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}
  
  //Reset the G-trigger
  if(accel.orientX == 'Z'){settings.gTrigger -= accel.dirX*accel.biasX;}
  if(accel.orientY == 'Z'){settings.gTrigger -= accel.dirY*accel.biasY;}
  if(accel.orientZ == 'Z'){settings.gTrigger -= accel.dirZ*accel.biasZ;}

  //set the booster burp check time
  boosterBurpTime = min(1000000UL, settings.separation_delay-10000UL);
  
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
  for(byte i = 0; i < altDigits; i++){if(EEPROM.read(i) != (byte)255){initPgm = false;}}
  //Write initial values into EEPROM
  if(initPgm){
    if(settings.testMode){Serial.println(F("Initial Use Detected: Writing Initial EEPROM values for prior flight"));}
    for(byte j = 0; j <6; j++){EEPROM.update(j,j);}}

//  EEPROM.update(0, 0);
//  EEPROM.update(1, 0);
//  EEPROM.update(2, 5);
//  EEPROM.update(3, 1);
//  EEPROM.update(4, 2);
//  EEPROM.update(5, 7);
  
  //Beep out the last flight's altitude
  if(settings.reportStyle == 'P'){
    if(settings.testMode){Serial.print(F("Reporting last flight: "));}
    for(byte i=0;i<6;i++){maxAltDigits[i]=EEPROM.read(i);}
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
    Serial.print(F("Beginning GPS NMEA Serial Output and Continuity Reporting: "));
    Serial.println(beepCode);
    delay(3000);}
}//end setup

void loop(void){
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);

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
      newBMP=true;}

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
      newBMP = true;}}
      
  if(preLiftoff && newBMP){
      sumBaseAlt -= baseAltBuff[baseAltPosn];
      sumBaseAlt += Alt;
      baseAltBuff[baseAltPosn] = Alt;
      baseAltPosn++;
      if(baseAltPosn >= (byte)(sizeof(baseAltBuff)/sizeof(baseAltBuff[0]))){baseAltPosn = 0;}        
      baseAlt = sumBaseAlt/(float)(sizeof(baseAltBuff)/sizeof(baseAltBuff[0]));}
  
  //look for a shutdown command and if seen, stop all progress for a hard reset
  if (preLiftoff && settings.magSwitchEnable){
    getMag();
    n=0;
    if(abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger){n=1;}
    while(n==1){digitalWrite(pins.beep,HIGH);delay(1000);}}
   
  if (!liftoff && accel.z > settings.gTrigger && !touchdown && !timeOut) {
    //timeGyroClock = timeClockPrev; //initializes timeGyro to an appropriate value
    if(settings.testMode){Serial.println(' ');Serial.println(F("Simulated Liftoff Detected!"));}
    padTime = micros();
    lastTX = 0UL;
    radioDelay = RDinFlight; //transmit packets at a faster rate
    preLiftoff = false;
    liftoff = true;
    onGround = false;
    timeLastEvent = timeCurrent;
    radioEvent = 1;
    liftoffHour = GPS.time.hour();
    liftoffMin = GPS.time.minute();
    liftoffSec = GPS.time.second();
    liftoffMili = GPS.time.centisecond();}

  if (liftoff) {

    //Update master gyro timing variables
    gdt = long(timeGyroClock - timeGyroClockPrev);
    timeGyro += (unsigned long)gdt;

    //update master timing variables
    dt = timeClock - timeClockPrev;
    timeCurrent += dt;

    //Get High-G Accelerometer Data and update the 30-point moving average
    //this roughly equates to 0.02 seconds, which greatly improves accuracy with minimal latency
    getHighG(settings.highG3axis);
    highGsum -= highGfilter[filterPosn];
    highGfilter[filterPosn] = highG.z;
    highGsum += highGfilter[filterPosn];
    filterPosn++;
    lastHighG = timeCurrent;
    if(filterPosn >= sizeHighGfilter){filterPosn = 0;}
    highGsmooth = (int16_t)(highGsum / sizeHighGfilter);
      
    //See if a new altitude reading is available
    if(newBMP){
      Alt -= baseAlt;
      if(Alt > maxAltitude && !apogee){maxAltitude = Alt;}
      //Smoothed barometric altitude of the last 10 altitude readings
      rawAltSum -= rawAltBuff[rawAltPosn];
      rawAltBuff[rawAltPosn] = Alt;
      rawAltSum += rawAltBuff[rawAltPosn];
      rawAltPosn++;
      if(rawAltPosn >= (byte)(sizeof(rawAltBuff)/sizeof(rawAltBuff[0]))){rawAltPosn = 0;}
      altMoveAvg = rawAltSum / (float)(sizeof(rawAltBuff)/sizeof(rawAltBuff[0]));
      //barometric velocity & apogee trigger based on the last half-second of altitude measurments
      baroVel = (altMoveAvg - altAvgBuff[altAvgPosn])/((float)(timeCurrent - baroTimeBuff[altAvgPosn])*mlnth);
      if(timeCurrent < 500000L){baroVel = 0;}
      altAvgBuff[altAvgPosn] = altMoveAvg;
      baroTimeBuff[altAvgPosn] = timeCurrent;
      altAvgPosn++;
      if(altAvgPosn >= (byte)(sizeof(altAvgBuff)/sizeof(altAvgBuff[0]))){altAvgPosn = 0;}
      radioVel = (int16_t)baroVel;
      //Baro touchdown trigger
      (mainDeploy && abs(baroVel) < 1 )? baroTouchdown++ : baroTouchdown = 0;}

    //Get magnetometer data
    magCounter += dt;
    if (magCounter >= magTime){
      getMag();
      magCounter = 0;}

    //Compute the current g-load. Use the high-G accelerometer if the standard one is pegged
    if(abs(accel.z) < accel.ADCmax){accelNow = (float)(accel.z - g) * accel.gainZ;}
    else{accelNow = (highGsmooth - (float)high1G) *  highG.gainZ;}
        
    //Integrate velocity, altitude, and rotation data prior to apogee
    if(!apogeeFire || settings.testMode){

      //Capture the max acceleration
      if(accelNow > maxG){maxG = accelNow;}
      
      //calculate the new acceleration based velocity
      //this makes the apogee event mach immune
      accelVel += accelNow * (float)dt * mlnth;
      if(!boosterBurnout || (accelVel > 300)){radioVel = (int16_t)accelVel;}
      else{float frac = accelVel/300; radioVel = (int16_t)((frac)*accelVel + (1.0 - frac)*baroVel);}
      
      //update maximum velocity if it exceeds the previous value
      if(accelVel > maxVelocity){maxVelocity = accelVel;}
    
      //calculate the new acceleration based altitude
      accelAlt += accelVel * (float)dt * mlnth;
      if(!altOK && (accelAlt > settings.Alt_threshold || settings.testMode)){altOK = true;}

      if(settings.quatEnable){
        //get the quaternion rotation
        //caluclate the partial rotation
        const float deg2rad = 0.00122173;
        dx += gyro.x * gdt;
        dy += gyro.y * gdt;
        dz += gyro.z * gdt;

        //if required update the rotation
        if(timeCurrent - lastRotn > rotnRate){
        
          ddx = (dx*deg2rad)*mlnth;
          ddy = (dy*deg2rad)*mlnth;
          ddz = (dz*deg2rad)*mlnth;
        
          getQuatRotn( ddx , ddy , ddz);
          dx = 0L;
          dy = 0L;
          dz = 0L;
          lastRotn = timeCurrent;}}
      if(!settings.quatEnable){getRotnDCM2D();}
      }//end if !apogee

    //Check for timeout
    if (!timeOut && !pyroFire && timeCurrent > settings.rcd_time) {
      timeOut = true;
      onGround = true;
      radioEvent = 9;
      timeLastEvent = micros();
      touchdownHour = GPS.time.hour();
      touchdownMin = GPS.time.minute();
      touchdownSec = GPS.time.second();
      touchdownMili = GPS.time.centisecond();}
    
    //Check false trigger until the flight time has passed the minimum time
    if (checkFalseTrigger) {
      if (timeCurrent > settings.detectLiftoffTime) {checkFalseTrigger = false;}
      if (accel.z < settings.gTrigger && accelVel < 15.5F) {
        Serial.println("False Trigger Reset");
        //reset the key triggers
        timeCurrent = 0UL;
        timeGyro = 0UL;
        timeLastEvent = 0UL;
        preLiftoff = true;
        liftoff = false;
        onGround = true;
        radioEvent = 0;
        pktPosn = 0;
        packetnum = 0;
        boosterBurnout = false;
        baroApogee = 0;
        baroApogeePosn = 0;
        for(byte i=0;i<5;i++){baroLast5[i]=0;}
        highGsmooth = 0;
        highGsum = 0;
        for(byte i=0; i<10; i++){highGfilter[i]=0;}
        baroTouchdown = 0;
        accelVel = 0;
        accelAlt = 0;
        packetnum = 1;
        radioDelay = RDpreLiftoff;
        lastTX = 0UL;}
    }//end checkFalseTrigger

    //check for booster burnout: if the z acceleration is negative
    if (!boosterBurnout && liftoff && accel.z <= 0) {
      boosterBurnout = true;
      radioEvent = 2;
      boosterBurnoutCheck = true;
      timeLastEvent = timeCurrent;}
      
    //check for booster motor burp for 1 second after burnout is detected
    if (boosterBurnoutCheck){
      if(timeCurrent - timeLastEvent > boosterBurpTime){boosterBurnoutCheck = false;}
      else if (boosterBurnout && !settings.testMode && accel.z > 0){boosterBurnout = false; boosterBurnoutCheck = false; radioEvent = 1;}}

    //Fire separation charge if burnout is detected and time is past the separation delay
    if (settings.twoStage && !boosterSeparation &&  liftoff && boosterBurnout && !checkFalseTrigger && timeCurrent - timeLastEvent > settings.separation_delay) {
      boosterSeparation = true;
      timeLastEvent = timeCurrent;
      firePyros('B');
      radioEvent = 3;}

    //Fire second stage
    if (settings.twoStage && !sustainerFireCheck && (!apogee || settings.testMode) && liftoff && boosterBurnout && boosterSeparation && !pyroFire && timeCurrent - timeLastEvent > settings.sustainerFireDelay) {
      sustainerFireCheck = true;
      postFlightCode = 1;
      timeLastEvent = timeCurrent;
      //Check for staging inhibit and fire if OK
      if (altOK && rotnOK) {
        sustainerFire = true;
        firePyros('I');
        radioEvent = 4;}
      else if (!rotnOK && !altOK){postFlightCode = 4; radioEvent = 12;}
      else if (!rotnOK) {postFlightCode = 3; radioEvent = 10;}
      else if (!altOK) {postFlightCode = 2; radioEvent = 11;}}

    // Check for sustainer ignition
    if(settings.twoStage && !apogee && !sustainerIgnition && sustainerFire && accelNow > 10.0){radioEvent = 13; sustainerIgnition = true; timeLastEvent = timeCurrent;}
    
    //Check for sustainer burnout
    if(settings.twoStage && !apogee && !sustainerBurnout && sustainerIgnition && accel.z < 0 && timeCurrent - timeLastEvent > 100000UL){radioEvent = 14; sustainerBurnout = true; timeLastEvent = timeCurrent;}
    
    //Check for apogee if the accelerometer velocity or barometric velocity < 0
    //Above 9000 meters, only use accelerometer velocity for apogee trigger
    if (!apogee && boosterBurnout && !boosterBurnoutCheck && !pyroFire && (accelVel < 0 || (baroVel < 0 && accelVel < 70 && (Alt + baseAlt) < 9000))) {
      apogee = true;
      timeLastEvent = timeCurrent;
      radioEvent = 5;}


    //*****************************************************
    /*Start of New Code*/
    //*****************************************************

    /*//Check false trigger until the flight time has passed the minimum time
    if (events.falseLiftoffCheck) {
      if (fltTime.timeCurrent > settings.detectLiftoffTime) {events.falseLiftoffCheck = false;}
      if (accel.z < settings.gTrigger && accelVel < 15.5F) {
        Serial.println("False Trigger Reset");
        //reset the key triggers
        events = resetEvents;
        fltTime.timeCurrent = 0UL;
        fltTime.timeGyro = 0UL;
        radioEvent = 0;
        pktPosn = 0;
        packetnum = 0;
        baroApogee = 0;
        baroApogeePosn = 0;
        for(byte i=0;i<5;i++){baroLast5[i]=0;}
        highGsmooth = 0;
        highGsum = 0;
        for(byte i=0; i<10; i++){highGfilter[i]=0;}
        baroTouchdown = 0;
        accelVel = 0;
        accelAlt = 0;
        packetnum = 1;
        radioDelay = RDpreLiftoff;
        lastTX = 0UL;}
    }//end falseLiftoffCheck

    //check for booster burnout: if the z acceleration is negative
    if (!events.boosterBurnout && events.liftoff && accel.z <= 0) {
      events.boosterBurnout = true;
      radioEvent = 2;
      events.boosterBurnoutCheck = true;
      fltTime.timeBoosterBurnout = fltTime.timeCurrent;}
      
    //check for booster motor burp for 1 second after burnout is detected
    if (events.boosterBurnoutCheck){
      if(fltTime.timeCurrent - fltTime.timeBoosterBurnout > boosterBurpTime){events.boosterBurnoutCheck = false;}
      else if (events.boosterBurnout && !settings.testMode && accel.z > 0){events.boosterBurnout = false; events.boosterBurnoutCheck = false; radioEvent = 1;}}

    //2-Stage Flight Profile
    if(settings.profile == '2'){

      //Fire separation charge if burnout is detected and time is past the separation delay
      if (settings.twoStage && !events.boosterSeparation &&  events.liftoff && events.boosterBurnout && !checkFalseTrigger && fltTime.timeCurrent - settings.boosterBurnoutTime > settings.separationDelay) {
      events.boosterSeparation = true;
      fltTime.timeBoosterSeparation = fltTime.timeCurrent;
      firePyros('B');
      radioEvent = 3;}

    //Fire second stage
    if (settings.twoStage && !events.sustainerFireCheck && (!events.apogee || settings.testMode) && events.liftoff && events.boosterBurnout && events.boosterSeparation && !pyroFire && fltTime.timeCurrent - fltTime.timeBoosterSeparation > settings.sustainerFireDelay) {
      sustainerFireCheck = true;
      postFlightCode = 1;
      fltTime.timeSustainerFireCheck = fltTime.timeCurrent;
      //Check for staging inhibit and fire if OK
      if (altOK && rotnOK) {
        events.sustainerFire = true;
        firePyros('I');
        radioEvent = 4;}
      else if (!rotnOK && !altOK){postFlightCode = 4; radioEvent = 12;}
      else if (!rotnOK) {postFlightCode = 3; radioEvent = 10;}
      else if (!altOK) {postFlightCode = 2; radioEvent = 11;}}

    // Check for sustainer ignition
    if(settings.twoStage && !events.apogee && !events.sustainerIgnition && events.sustainerFire && accelNow > 10.0){radioEvent = 13; events.sustainerIgnition = true; fltTime.timeSustainerIgnition = fltTime.timeCurrent;}
    
    //Check for sustainer burnout
    if(settings.twoStage && !events.apogee && !events.sustainerBurnout && events.sustainerIgnition && accelNow < 0.0 && fltTime.timeCurrent - fltTime.timeSustainerIgnition > 100000UL){radioEvent = 14; events.sustainerBurnout = true; fltTime.timeSustainerBurnout = fltTime.timeCurrent;}
    
    }//end 2-stage profile

    //Airstart Flight Profile
    if(settings.profile == 'A'){

      //AirStart motor 1 if event 1 is main booster ignition
      if(settings.firstPyroEvent == 'I' && !events.checkAirStart1 && !checkFalseTrigger && fltTime.timeCurrent > settings.firstEventDelay){
        events.checkAirStart1 = true;
        fltTime.timeCheckAirStart1 = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for staging inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart1Fire = true;
          fltTime.timeAirStart1Fire = timeCurrent;
          firePyros('1');
          radioEvent = 15;}
        else if (!rotnOK && !altOK){postFlightCode = 4; radioEvent = 12;}
        else if (!rotnOK) {postFlightCode = 3; radioEvent = 10;}
        else if (!altOK) {postFlightCode = 2; radioEvent = 11;}}
      
      //AirStart motor 1 if event 1 is main booster burnout
      if(settings.firstPyroEvent == 'B' && events.boosterBurnout && !events.checkAirStart1 && fltTime.timeCurrent > (fltTime.timeBoosterBurnout + settings.firstEventDelay)){
        events.checkAirStart1 = true;
        fltTime.timeCheckAirStart1 = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for staging inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart1Fire = true;
          fltTime.timeAirStart1Fire = timeCurrent;
          firePyros('1');
          radioEvent = 15;}
        else if (!rotnOK && !altOK){postFlightCode = 4; radioEvent = 12;}
        else if (!rotnOK) {postFlightCode = 3; radioEvent = 10;}
        else if (!altOK) {postFlightCode = 2; radioEvent = 11;}}

      //Look for AirStart 1 Ignition
      if(events.airStart1Fire && !apogee && !events.airStart1Ignition && accelNow > 10.0){events.airStart1Igniton = true; fltTime.timeAirStart1Ignition = fltTime.timeCurrent; radioEvent = 13;}

      //Look for AirStart 1 Burnout with a check for a motor burp
      if(events.airStart1Ignition && !events.apogee && !events.airStart1Burnout && accelNow < 0.0){events.airStart1BurnoutCheck = true; fltTime.timeAirStart1Burnout = fltTime.timeCurrent;}
      if(events.airStart1BurnoutCheck && fltTime.timeCurrent < (fltTime.timeAirStart1Burnout + boosterBurpTime) && accel.z > 0){events.airStartBurnoutCheck = false;}
      if(events.airStart1BurnoutCheck && fltTime.timeCurrent > (fltTime.timeAirStart1Burnout + boosterBurpTime)){events.airStart1BurnoutCheck = false; events.airStart1Burnout = true; radioEvent = 14;}
        
      //AirStart motor 2 if event 2 is airstart1 motor ignition
      if(settings.secondPyroEvent == 'I' && !events.apogee && !events.checkAirStart2 && events.airStart1Ignition && fltTime.timeCurrent > fltTime.timeAirStart1Ignition + settings.secondEventDelay){
        events.checkAirStart2 = true;
        fltTime.timeCheckAirStart2 = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for staging inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart2Fire = true;
          fltTime.timeAirStart2Fire = fltTime.timeCurrent;
          firePyros('2');
          radioEvent = 15;}
        else if (!rotnOK && !altOK){postFlightCode = 4; radioEvent = 12;}
        else if (!rotnOK) {postFlightCode = 3; radioEvent = 10;}
        else if (!altOK) {postFlightCode = 2; radioEvent = 11;}}
        
      //Airstart motor 2 if event 2 is airstart1 motor burnout
      if(settings.secondPyroEvent == 'B' && !events.apogee && !events.checkAirStart2 && events.airStart1Burnout && fltTime.timeCurrent > (events.timeAirStart1Burnout + settings.secondEventDelay)){
        events.checkAirStart2 = true;
        fltTime.timeCheckAirStart2 = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for staging inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart2Fire = true;
          fltTime.timeAirStart2Fire = fltTime.timeCurrent;
          firePyros('2');
          radioEvent = 15;}
        else if (!rotnOK && !altOK){postFlightCode = 4; radioEvent = 12;}
        else if (!rotnOK) {postFlightCode = 3; radioEvent = 10;}
        else if (!altOK) {postFlightCode = 2; radioEvent = 11;}}

      //Look for AirStart 2 Ignition
      if(events.airStart2Fire && !events.apogee && !events.airStart2Ignition && accelNow > 10.0){events.airStart2Igniton = true; fltTime.timeAirStart2Ignition = fltTime.timeCurrent; radioEvent = 16;}

      //Look for AirStart 2 Burnout
      if(events.airStart2Ignition && !events.apogee && !events.airStart2Burnout && accelNow < 0.0){events.airStart1Burnout = true; fltTime.timeAirstart2Burnout = fltTime.timeCurrent; radioEvent = 17;}
        
    }//End Airstart Flight Mode

    //Fire apgogee charge if the current time > apogeeTime + apogeeDelay
    if (!events.apogeeFire && events.apogee && fltTime.timeCurrent - fltTime.timeApogee >= settings.apogeeDelay) {
      events.apogeeFire = true;
      fltTime.apogeeFire = fltTime.timeCurrent;
      firePyros('A');
      if(!events.apogeeSeparation){radioEvent = 5;}}
      
    //Write the data to the card 3s after apogeeFire and mainDeploy in case of crash or powerloss
    if(events.apogeeFire && !syncCard && !settings.testMode && fltTime.timeCurrent - fltTime.timeMainDeploy >= 3000000UL){outputFile.sync();syncCard = true;}

    //Detect separation after apogee
    if(events.apogeeFire && !events.mainDeploy && accel.z > 4*g && fltTime.timeCurrent - fltTime.timeApogeeFire <= 2000000UL){events.apogeeSeparation = true; radioEvent = 6;}

    //Fire main chute charge if the baro altitude is lower than the threshold and at least 1s has passed since apogee
    if (!mainDeploy && apogee && Alt < settings.mainDeployAlt && timeCurrent - timeLastEvent >= 1000000UL) {
      mainDeploy = true;
      timeLastEvent = timeCurrent;
      firePyros('M');
      radioEvent = 7;
      //reset the sync boolean so the card syncs again 3s after main deploy
      syncCard = false;}

    //Detect deployment of the mains
    if(mainDeploy && timeCurrent-timeLastEvent > 50000UL && timeCurrent-timeLastEvent < 3000000UL && accelNow > 50.0){radioEvent = 15;}
    */

    //Fire apgogee charge if the current time > apogeeTime + apogeeDelay
    if (!apogeeFire && apogee && timeCurrent - timeLastEvent >= settings.apogeeDelay) {
      apogeeFire = true;
      timeLastEvent = timeCurrent;
      firePyros('A');}
      
    //Write the data to the card 3s after apogeeFire and mainDeploy in case of crash or powerloss
    if(apogeeFire && !syncCard && !settings.testMode && timeCurrent - timeLastEvent >= 3000000UL){outputFile.sync();syncCard = true;}

    //Detect separation after apogee
    if(apogeeFire && !mainDeploy && accel.z > 4*g && timeCurrent - timeLastEvent <= 2000000UL){apogeeSeparation = true; radioEvent = 6;}

    //Fire main chute charge if the baro altitude is lower than the threshold and at least 1s has passed since apogee
    if (!mainDeploy && apogee && Alt < settings.mainDeployAlt && timeCurrent - timeLastEvent >= 1000000UL) {
      mainDeploy = true;
      timeLastEvent = timeCurrent;
      firePyros('M');
      radioEvent = 7;
      //reset the sync boolean so the card syncs again 3s after main deploy
      syncCard = false;}

    //Detect deployment of the mains
    if(mainDeploy && timeCurrent-timeLastEvent > 50000UL && timeCurrent-timeLastEvent < 3000000UL && accelNow > 50.0){radioEvent = 15;}

    //Turn off the pyros after the allotted time
    if (pyroFire) {
      if(pyro1.fireStatus && timeCurrent > pyro1.fireStop){digitalWrite(pyro1.firePin, LOW);pyro1.fireStatus = false;}
      if(pyro2.fireStatus && timeCurrent > pyro2.fireStop){digitalWrite(pyro2.firePin, LOW);pyro2.fireStatus = false;}
      if(pyro3.fireStatus && timeCurrent > pyro3.fireStop){digitalWrite(pyro3.firePin, LOW);pyro3.fireStatus = false;}
      if(pyro4.fireStatus && timeCurrent > pyro4.fireStop){digitalWrite(pyro4.firePin, LOW);pyro4.fireStatus = false;}
      if(!pyro1.fireStatus && !pyro2.fireStatus && !pyro3.fireStatus && !pyro4.fireStatus){pyroFire = false;}}

    //Check for touchdown
    if (!touchdown && mainDeploy && !pyroFire && !settings.testMode && baroTouchdown > touchdownTrigger && Alt < 46) {
      touchdown = true;
      onGround = true;
      timeLastEvent = micros();
      radioEvent = 8;
      touchdownHour = GPS.time.hour();
      touchdownMin = GPS.time.minute();
      touchdownSec = GPS.time.second();
      touchdownMili = GPS.time.centisecond();}

    //check continuity
    pyro1.contStatus = ((digitalRead(pyro1.contPin) == HIGH) ? true : false);
    pyro2.contStatus = ((digitalRead(pyro2.contPin) == HIGH) ? true : false);
    pyro3.contStatus = ((digitalRead(pyro3.contPin) == HIGH) ? true : false);
    pyro4.contStatus = ((digitalRead(pyro4.contPin) == HIGH) ? true : false);

    //Read the battery voltage
    if(newBMP){
      voltReading = analogRead(pins.batt);
      voltage = (float)(voltReading)*3.3*3.2*adcConvert;
      if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}}
      
    //build the packet of 4 samples: 13 bytes per sample, 12 bytes GPS & pktnum, 13 x 4 + 12 = 64 bytes flight data
    if(settings.radioTXmode && timeCurrent - timeLastSample > sampDelay){
      timeLastSample = timeCurrent;
      sampNum++;

      //event
      dataPacket[pktPosn]=radioEvent; pktPosn++;//1
      //time
      radioTime = (uint16_t)(timeCurrent/10000);
      dataPacket[pktPosn] = lowByte(radioTime);pktPosn++;//2
      dataPacket[pktPosn] = highByte(radioTime);pktPosn++;//3
      //velocity
      dataPacket[pktPosn] = lowByte(radioVel);pktPosn++;//4
      dataPacket[pktPosn] = highByte(radioVel);pktPosn++;//5
      //altitude
      radioInt = (int16_t)(Alt);
      dataPacket[pktPosn] = lowByte(radioInt);pktPosn++;//6
      dataPacket[pktPosn] = highByte(radioInt);pktPosn++;//7
      //Rotation data
      dataPacket[pktPosn] = lowByte(rollZ);pktPosn++;//8
      dataPacket[pktPosn] = highByte(rollZ);pktPosn++;//9
      dataPacket[pktPosn] = lowByte(offVert);pktPosn++;//10
      dataPacket[pktPosn] = highByte(offVert);pktPosn++;//11
      //Acceleration
      radioInt = (int16_t)(accelNow * 33.41406087); //33.41406087 = 32768 / 9.80665 / 100
      dataPacket[pktPosn] = lowByte(radioInt);pktPosn++;//12
      dataPacket[pktPosn] = highByte(radioInt);pktPosn++;}//13
      
    //GPS Data collected once per packet then transmit
    if(sampNum >= 4){
      packetnum++;
      dataPacket[pktPosn] = lowByte(packetnum); pktPosn++;//53
      dataPacket[pktPosn] = highByte(packetnum); pktPosn++;//54
      if(gpsTransmit){
        gpsTransmit=false;
        radioInt = (int16_t)(GPS.altitude.meters() - baseAlt);
        dataPacket[pktPosn] = lowByte(radioInt);pktPosn++;//55
        dataPacket[pktPosn] = highByte(radioInt);pktPosn++;//56
        GPSunion.GPScoord = gpsLatitude;
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSunion.GPSbyte[i];pktPosn++;}//60
        GPSunion.GPScoord = gpsLongitude;
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSunion.GPSbyte[i];pktPosn++;}}//64
      //send packet
      rf95.send((uint8_t *)dataPacket, pktPosn);
      //reset counting variables
      sampNum = 0;
      pktPosn = 0;
      radioTX = true;}
      
    //Write the data to a string
    //Cycle timestamp
    writeLongData(timeCurrent);
    //Accel Data
    writeIntData(accel.x);
    writeIntData(accel.y);
    writeIntData(accel.z);
    //Gyro Data
    writeIntData(gyro.x);
    writeIntData(gyro.y);
    writeIntData(gyro.z);
    //HighG Accel Data
    if(settings.highG3axis){writeIntData(highG.x);writeIntData(highG.y);}
    else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    writeIntData(highG.z);
    writeIntData(highGsmooth);
    //Integrated Rotation Values
    writeLongData(rollZ);
    writeIntData(yawY);
    writeIntData(pitchX);
    writeIntData(offVert);
    //Integrated Speed and Altitude
    writeFloatData(accelVel, 2);
    writeFloatData(accelAlt, 2);
    //Flight Event Flags
    writeBoolData(liftoff);
    writeBoolData(boosterBurnout);
    writeBoolData(boosterBurnoutCheck);
    writeBoolData(boosterSeparation);
    writeBoolData(sustainerFireCheck);
    writeBoolData(sustainerFire);
    writeBoolData(apogee);
    writeBoolData(apogeeFire);
    writeBoolData(mainDeploy);
    writeBoolData(touchdown);
    writeBoolData(timeOut);
    dataString[strPosn] = cs;strPosn++;
    //Continuity Data
    dataString[strPosn] = 'C';strPosn++;
    writeBoolData(pyro1.contStatus);
    writeBoolData(pyro2.contStatus);
    writeBoolData(pyro3.contStatus);
    writeBoolData(pyro4.contStatus);
    dataString[strPosn] = cs;strPosn++;
    //Pyro Firing Data
    dataString[strPosn] = 'F';strPosn++;
    writeBoolData(pyro1.fireStatus);
    writeBoolData(pyro2.fireStatus);
    writeBoolData(pyro3.fireStatus);
    writeBoolData(pyro4.fireStatus);
    dataString[strPosn] = cs;strPosn++;
    writeIntData(firePin);
    //Barometer & Battery Voltage data
    if (newBMP) {
      writeFloatData(Alt, 2);
      writeFloatData(altMoveAvg, 2);
      writeFloatData(baroVel, 2);
      writeFloatData(pressure, 2);
      writeFloatData(temperature, 2);
      writeFloatData(voltage, 2);
      newBMP=false;}
    else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    //Magnetometer Data
    if (magCounter == 0){
      writeIntData(mag.x);
      writeIntData(mag.y);
      writeIntData(mag.z);}
    else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    //GPS Data
    if(gpsWrite){
      dataString[strPosn]=gpsLat;strPosn++;
      writeFloatData(gpsLatitude,6);
      dataString[strPosn]=gpsLon;strPosn++;
      writeFloatData(gpsLongitude,6);
      writeFloatData((float)GPS.speed.mph(),2);
      writeFloatData(((float)GPS.altitude.meters()-baseAlt),2);
      writeFloatData((float)GPS.course.deg(),2);
      writeIntData((int)GPS.satellites.value());
      gpsWrite=false;}
    else{
      dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;
      dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    //update the radio packet number
    if (radioTX){writeIntData(packetnum);radioTX = false;}
    else{dataString[strPosn]=cs;strPosn++;}
    //end of sample - carriage return, newline, and null value
    dataString[strPosn] = '\r';strPosn++;
    dataString[strPosn] = '\n';strPosn++;
    dataString[strPosn] = '\0';
    // write the string to file
    outputFile.write(dataString, strPosn);
    strPosn = 0;
    
    //Close file at Touchdown or Timeout
    if (timeOut || touchdown) {
      //Print the initial conditions
      outputFile.println(F("Max Baro Alt,Max GPS Alt,Max Speed,Max Gs,baseAlt,initial Y ang,initial X ang,accelX0,accelY0,accelZ0,highGz0,magX0,magY0,magZ0,gyroBiasX,gyroBiasY,gyroBiasZ,accelBiasX,accelBiasY,accelBiasZ,highGbiasX,highGbiasY,highGbiasZ,padTime"));
      writeLongData((unsigned long)(maxAltitude*unitConvert));
      writeLongData((unsigned long)(maxGPSalt*unitConvert));
      writeLongData((unsigned long)(maxVelocity*unitConvert));
      writeFloatData(maxG/9.80665, 2);
      writeFloatData(baseAlt, 2);
      writeFloatData(yawY0, 2);
      writeFloatData(pitchX0, 2);
      writeIntData(accel.x0);
      writeIntData(accel.y0);
      writeIntData(accel.z0);
      writeIntData(highG.z0);
      writeIntData(mag.x0);
      writeIntData(mag.y0);
      writeIntData(mag.z0);
      writeIntData(gyro.biasX);
      writeIntData(gyro.biasY);
      writeIntData(gyro.biasZ);
      writeIntData(accel.biasX);
      writeIntData(accel.biasY);
      writeIntData(accel.biasZ);
      writeIntData(highG.biasX);
      writeIntData(highG.biasY);
      writeIntData(highG.biasZ);
      writeLongData(padTime);
      //carriage return, newline, and null value
      dataString[strPosn] = '\r';strPosn++;
      dataString[strPosn] = '\n';strPosn++;
      dataString[strPosn] = '\0';
      outputFile.write(dataString, strPosn);
      
      //write out the launch time and locations
      strPosn = 0;
      outputFile.println(F("launch date, UTC time, launch altitude, launch latitude, launch longitude"));
      //Write out the GPS liftoff date
      outputFile.print(liftoffDay);outputFile.print("/");outputFile.print(liftoffMonth);outputFile.print("/");outputFile.print(liftoffYear);outputFile.print(",");
      //Write out the GPS liftoff time
      outputFile.print(liftoffHour);outputFile.print(":");outputFile.print(liftoffMin);outputFile.print(":");outputFile.print((int)liftoffSec);outputFile.print(",");
      //Write out GPS launch location
      writeFloatData(baseGPSalt,2);
      dataString[strPosn]=liftoffLat; strPosn++; writeFloatData2(liftoffLatitude,4);
      dataString[strPosn]=liftoffLon; strPosn++; writeFloatData2(liftoffLongitude,4);
      //end of sample - carriage return, newline, and null value
      dataString[strPosn] = '\r';strPosn++;
      dataString[strPosn] = '\n';strPosn++;
      dataString[strPosn] = '\0';
      outputFile.write(dataString, strPosn);

      //Write out the GPS landing location
      strPosn = 0;
      outputFile.println(F("landing date, UTC time, landing altitude, landing latitude, landing longitude"));
      //Write out the GPS landing date
      outputFile.print(liftoffDay);outputFile.print("/");outputFile.print(liftoffMonth);outputFile.print("/");outputFile.print(liftoffYear);outputFile.print(",");
      //Write out the GPS landing time
      outputFile.print(touchdownHour);outputFile.print(":");outputFile.print(touchdownMin);outputFile.print(":");outputFile.print((int)touchdownSec);outputFile.print(",");
      writeFloatData(touchdownAlt,2);
      dataString[strPosn]=touchdownLat; strPosn++; writeFloatData2(touchdownLatitude,4);
      dataString[strPosn]=touchdownLon; strPosn++; writeFloatData2(touchdownLongitude,4);
      //end of sample - carriage return, newline, and null value
      dataString[strPosn] = '\r'; strPosn++;
      dataString[strPosn] = '\n'; strPosn++;
      dataString[strPosn] = '\0';
      outputFile.write(dataString, strPosn);
      
      //write out the settings for the flight
      strPosn = 0;
      outputFile.println(F("Rocket Name, callsign, gTrigger, detectLiftoffTime, apogeeDelay, mainDeployAlt, rcdTime, fireTime, 2Stage, ignitionDelay, sepDelay, altThreshold, maxAng, seaLevelPressure"));
      outputFile.print(settings.rocketName);outputFile.print(cs);
      outputFile.print(settings.callSign);outputFile.print(cs);
      writeFloatData((float)(settings.gTrigger/g),1);
      writeFloatData(settings.detectLiftoffTime*mlnth,1);
      writeFloatData(settings.apogeeDelay*mlnth,1);
      writeIntData((int)(10*int(settings.mainDeployAlt*(unitConvert/10))));
      writeFloatData(settings.rcd_time*mlnth,0);
      writeFloatData(settings.fireTime*mlnth,1);
      if(settings.twoStage){writeIntData(1);} else{writeIntData(0);}
      writeFloatData(settings.sustainerFireDelay*mlnth,1);
      writeFloatData(settings.separation_delay*mlnth,1);
      writeIntData((int)(10*int(settings.Alt_threshold*(unitConvert/10))));
      writeIntData(settings.max_ang/10);
      writeFloatData(seaLevelPressure,2);
      //end of sample - carriage return, newline, and null value
      dataString[strPosn] = '\r';
      strPosn++;
      dataString[strPosn] = '\n';
      strPosn++;
      dataString[strPosn] = '\0';
      outputFile.write(dataString, strPosn);    
      strPosn=0;
      //close the file
      outputFile.close();
      fileClose = true;
      liftoff = false;
      digitalWrite(firePin, LOW);
      //Set the radio transmitter to post-flight data rate
      radioDelay = RDpostFlight;
      //Read max altitude into its beep array
      parseBeep(long(maxAltitude*unitConvert), maxAltDigits, 6);
      //Read max velocity into its beep array
      parseBeep(long(maxVelocity*unitConvert), maxVelDigits, 4);
      //reset n, which we'll use to cycle through the reporting digits
      while (maxAltDigits[altDigits-1]==0){altDigits--;}
      while (maxVelDigits[velDigits-1]==0){velDigits--;}  
      n=altDigits;
      reportCode = true;
      //store the maximum altitude in EEPROM
      if(!settings.testMode){for(byte i=0;i<6;i++){EEPROM.update(i,maxAltDigits[i]);}}
    }//end of timeout/touchdown check    
    
  }//end of liftoff flag
  
  //Code to start the beep
  if ((preLiftoff || fileClose) && !beep && timeClock - timeLastBeep > beep_delay)  {
    digitalWrite(pins.beep, HIGH);
    timeBeepStart = timeClock;
    beep_counter++;
    if (beep_counter == beepCode) {
      beep_counter = 0;
      beep_delay = long_beep_delay;
      //If we are post-flight reporting, cycle through the reporting variable
      if (fileClose && postFlightCode != 1){beepCode = postFlightCode;}
      else if (fileClose && reportCode){beepCode = maxAltDigits[n-1];
        if(n==altDigits){beep_delay = 3000000UL;}
        n--;
        //switch reporting codes
        if(n==0){
          n=velDigits;
          reportCode = false;}}
      else if (fileClose && !reportCode){beepCode = maxVelDigits[n-1];
        if(n==velDigits){beep_delay = 3000000UL;}
        n--;
        //switch reporting codes
        if(n==0){
          n=altDigits;
          reportCode = true;}}}
    else {beep_delay = short_beep_delay;}
    beep = true;}

  //Code to stop the beep
  if (beep && (timeClock - timeBeepStart > beep_len)) {
    digitalWrite(pins.beep, LOW);
    timeBeepStart = 0UL;
    timeLastBeep = timeClock;
    beep = false;}

  //Telemetry ground code
  if(settings.radioTXmode && onGround && micros() - lastTX > radioDelay){
    if(preLiftoff){//37 byte packet
      pktPosn=0;
      dataPacket[pktPosn]=radioEvent; pktPosn++;
      dataPacket[pktPosn]=gpsFix; pktPosn++;
      if(settings.twoStage){dataPacket[pktPosn]=beepCode; pktPosn++;}
      else{dataPacket[pktPosn]=(beepCode + 5); pktPosn++;}
      for (byte j = 0; j < sizeof(settings.rocketName); j++){
        dataPacket[pktPosn] = settings.rocketName[j];
        pktPosn++;}
      radioInt = (int)baseAlt;
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      radioInt = (int)GPS.altitude.meters();
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=gpsLat; pktPosn++;
      GPSunion.GPScoord = gpsLatitude;
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSunion.GPSbyte[i]; pktPosn++;}
      dataPacket[pktPosn]=gpsLon; pktPosn++;
      GPSunion.GPScoord = gpsLongitude;
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSunion.GPSbyte[i]; pktPosn++;}
      //even thought the packet is shorter, sending 64 bytes is needed to activate the
      //Ublox counter-interference software for in-flight data capture
      rf95.send((uint8_t *)dataPacket, 64);
      lastTX = micros();
      pktPosn = 0;}
    if(touchdown || timeOut){//22 byte packet
      pktPosn=0;
      dataPacket[pktPosn]=radioEvent; pktPosn++;
      radioInt = (int)maxAltitude;
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      radioInt = (int)maxVelocity;
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      radioInt = (int)((maxG / 9.80665) * 327.68);
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      radioInt = (int)maxGPSalt;
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=gpsFix; pktPosn++;
      radioInt = (int)(GPS.altitude.meters());
      dataPacket[pktPosn]=lowByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=highByte(radioInt); pktPosn++;
      dataPacket[pktPosn]=gpsLat; pktPosn++;
      GPSunion.GPScoord = gpsLatitude;
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSunion.GPSbyte[i]; pktPosn++;}
      dataPacket[pktPosn]=gpsLon;pktPosn++;
      GPSunion.GPScoord = gpsLongitude;
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSunion.GPSbyte[i];pktPosn++;}
      rf95.send((uint8_t *)dataPacket, pktPosn);
      lastTX = micros();}
    }//end telemetry ground code
    
  //GPS Code
  /*if(!configGPSdefaults && micros() - timeLastGPS > 10000000UL){
    restoreGPSdefaults();
    configGPSdefaults = true; 
    gpsFix = 0;
    fixCount = 0;
    configGPSflight = false;}*/
  //5 seconds after touchdown put GPS into Power Save Mode (PSM)
  if( !GPSpsm && (touchdown || timeOut) && micros() - timeLastEvent > 5000000UL){GPSpowerSaveMode();GPSpsm = true;}
  //Read from serial
  while(HWSERIAL.available() > 0){
    char c = HWSERIAL.read();
    GPS.encode(c);
    if(settings.testMode && !liftoff){Serial.print(c);}}
 
  if (GPS.location.isUpdated() || GPS.altitude.isUpdated()) {
        timeLastGPS = micros();
        fixCount++;
        gpsFix = 1;
        if(!configGPSflight && fixCount > 40){configGPS();configGPSflight = true; configGPSdefaults = false;}
        gpsWrite = true;
        gpsTransmit = true;
        convertLocation();
        if(GPS.altitude.meters() > maxGPSalt){maxGPSalt = GPS.altitude.meters();}
        //capture the GPS takeoff position and correct base altitude
        if(preLiftoff){
          if(GPS.altitude.meters() != 0){
            //Correct sea level pressure with running average of 5 samples
            //GPS altitude running average
            GPSposn++;
            if(GPSposn >= (byte)(sizeof(GPSavgAlt)/sizeof(GPSavgAlt[0]))){GPSposn = 0;}
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
            seaLevelPressure = pressureAvg / pow((44330 - baseGPSalt)/44330, 5.254861);}
          liftoffLat=gpsLat;
          liftoffLatitude=gpsLatitude;
          liftoffLon=gpsLon;
          liftoffLongitude=gpsLongitude;
          liftoffYear = GPS.date.year();
          liftoffMonth = GPS.date.month();
          liftoffDay = GPS.date.day();}
        //capture the last GPS position
        if(mainDeploy || !touchdown || !timeOut){
          touchdownLat=gpsLat;
          touchdownLatitude=gpsLatitude;
          touchdownLon=gpsLon;
          touchdownLongitude=gpsLongitude;
          touchdownAlt = GPS.altitude.meters();}}

}//end void main loop
  
void parseBeep(long value, byte array[], byte arrayLen){
  boolean flag = false;
  for (byte i = arrayLen; i >= 1; i--){
       array[i-1] = byte(value/pow(10,i-1));
       value -= array[i-1]*pow(10,i-1);
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
   if(pyro1.func == event){firePin = pyro1.firePin; digitalWrite(pyro1.firePin, HIGH); pyro1.fireStatus = true; pyro1.fireStop = timeCurrent + settings.fireTime;}
   if(pyro2.func == event){firePin = pyro2.firePin; digitalWrite(pyro2.firePin, HIGH); pyro2.fireStatus = true; pyro2.fireStop = timeCurrent + settings.fireTime;}
   if(pyro3.func == event){firePin = pyro3.firePin; digitalWrite(pyro3.firePin, HIGH); pyro3.fireStatus = true; pyro3.fireStop = timeCurrent + settings.fireTime;}
   if(pyro4.func == event){firePin = pyro4.firePin; digitalWrite(pyro4.firePin, HIGH); pyro4.fireStatus = true; pyro4.fireStop = timeCurrent + settings.fireTime;}}
   
void writeIntData(int dataValue) {
  itoa(dataValue, dataString + strPosn, base);
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = cs;
  strPosn++;}//end void

void writeLongData(unsigned long dataValue){
  ultoa(dataValue, dataString + strPosn, base);
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = cs;
  strPosn++;}//end void
  
void writeFloatData2(float dataValue, byte decimals) {
  dtostrf(dataValue, 2, decimals, dataString + strPosn);
  updateStrPosn();}//end void

void writeFloatData(float dataValue, byte decimals){
  //limited to 4 decimal places only!  

  long fracInt;
  float partial;

  //sign portion
  if(dataValue < 0){
    dataString[strPosn] = '-'; 
    strPosn++;
    dataValue *= -1;}
  
  //integer portion
  itoa((int)dataValue, dataString + strPosn, base);
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn]='.'; strPosn++;
  
  //fractional portion
  partial = dataValue - (int)(dataValue);
  fracInt = (long)(partial*pow(10,decimals));
  if(fracInt == 0){dataString[strPosn] = '0'; strPosn++; dataString[strPosn] = cs; strPosn++;}
  else{
    decimals--;
    while(fracInt < pow(10, decimals)){dataString[strPosn]='0';strPosn++;decimals--;}
    ltoa(fracInt, dataString + strPosn, base);
    while(dataString[strPosn]!= '\0'){strPosn++;}
    dataString[strPosn]=','; strPosn++;}}
    
void writeBoolData(boolean dataBool) { 
  dataString[strPosn] = (dataBool) ? '1' : '0';
  strPosn ++;}//end void

void updateStrPosn(){
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = cs;
  strPosn++;}

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

void writeCalibration(int16_t inValue, byte eepromStart){

    calUnion.calValue = inValue;
    EEPROM.write(eepromStart, calUnion.calByte[0]);
    EEPROM.write(eepromStart + 1, calUnion.calByte[1]);}

void setOrientation(){

  int16_t *temPtr1;
  int16_t *temPtr2;
  int8_t tempDirX;
  int8_t tempDirY;
  int8_t tempDirZ;
  int8_t val = 1;
  byte k;
  char j;

  if(sensors.status_LSM9DS1){val = -1;}
  
  //Main IMU Z-axis is pointed to the nose
  if(abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX)){
  //EEPROM Allocation
  //24-29: IMU to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
  //30-35: highG to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
  //36-41: IMU-to-highG orientation translation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)

    //0 degree rotation, only change bias
    if(accel.biasZ > 0){
        Serial.println("Z-up Detected");
        //IMU direction
        //highG direction
        //IMU rotation
        //highG rotation
        //accel bias
        accel.biasZ -= g;
        //highG bias
        if((char)EEPROM.read(37)=='Z'){highG.biasX -= (int8_t)EEPROM.read(36)*high1G;}
        if((char)EEPROM.read(39)=='Z'){highG.biasY -= (int8_t)EEPROM.read(38)*high1G;}
        if((char)EEPROM.read(41)=='Z'){highG.biasZ -= (int8_t)EEPROM.read(40)*high1G;}}
    
    //180 degree rotation, no change in axes alignment
    else if(accel.biasZ < 0){
      Serial.println("Z-down Detected");
      //IMU direction
      accel.dirY *= -1;
      gyro.dirY *= -1;
      mag.dirY *= -1;
      accel.dirZ *= -1;
      gyro.dirZ *= -1;
      mag.dirZ *= -1;
      //highG direction
      highG.dirY *= -1;
      highG.dirZ *= -1;
      //IMU rotation - none
      //highG rotation - none
      //IMU bias
      accel.biasZ += g;
      //highG bias
      if((char)EEPROM.read(37)=='Z'){highG.biasX += (int8_t)EEPROM.read(36)*high1G;}
      if((char)EEPROM.read(39)=='Z'){highG.biasY += (int8_t)EEPROM.read(38)*high1G;}
      if((char)EEPROM.read(41)=='Z'){highG.biasZ += (int8_t)EEPROM.read(40)*high1G;}}
      
      //write to EEPROM
      k = 24;
      //IMU
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Z';
      EEPROM.update(k, accel.dirX); k++;
      EEPROM.update(k, accel.orientX); k++;
      EEPROM.update(k, accel.dirY); k++;
      EEPROM.update(k, accel.orientY); k++;
      EEPROM.update(k, accel.dirZ); k++;
      EEPROM.update(k, accel.orientZ); k++;
      //highG
      highG.orientX = (char)EEPROM.read(37);
      highG.orientY = (char)EEPROM.read(39);
      highG.orientZ = (char)EEPROM.read(41);
      EEPROM.update(k, highG.dirX); k++;
      EEPROM.update(k, highG.orientX); k++;
      EEPROM.update(k, highG.dirY); k++;
      EEPROM.update(k, highG.orientY); k++;
      EEPROM.update(k, highG.dirZ); k++;
      EEPROM.update(k, highG.orientZ);}
        
  //Main IMU X-axis is pointed to the nose
  else if(abs(accel.biasX) > abs(accel.biasY) && abs(accel.biasX) > abs(accel.biasZ)){
    //X-axis pointed up
    if(accel.biasX*val > 0){
      Serial.println("X-up Detected");
      //IMU direction
      tempDirX = accel.dirX;
      tempDirY = accel.dirY;
      tempDirZ = accel.dirZ;
      accel.dirX = gyro.dirX = mag.dirX = -1 * tempDirZ;
      accel.dirZ = gyro.dirZ = mag.dirZ = tempDirX;
      //highG direction
      tempDirX = highG.dirX;
      tempDirY = highG.dirY;
      tempDirZ = highG.dirZ;
      highG.dirX = -1 * tempDirZ;
      highG.dirZ = tempDirX;
      //IMU rotation
      accel.ptrX = &accel.rawZ;
      accel.ptrZ = &accel.rawX;
      //highG rotation
      temPtr1 = highG.ptrZ;
      temPtr2 = highG.ptrX;
      highG.ptrX = temPtr1;
      highG.ptrZ = temPtr2;
      //IMU bias
      accel.biasX -= g*val;
      //highG bias
      if((char)EEPROM.read(37)=='X'){highG.biasX -= val*(int8_t)EEPROM.read(36)*high1G;}
      if((char)EEPROM.read(39)=='X'){highG.biasY -= val*(int8_t)EEPROM.read(38)*high1G;}
      if((char)EEPROM.read(41)=='X'){highG.biasZ -= val*(int8_t)EEPROM.read(40)*high1G;}}

    //X-axis pointed down
    else if(accel.biasX*val < 0){
      Serial.println("X-down Detected");
      //IMU direction
      tempDirX = accel.dirX;
      tempDirY = accel.dirY;
      tempDirZ = accel.dirZ;
      accel.dirX = gyro.dirX = mag.dirX = tempDirZ;
      accel.dirZ = gyro.dirZ = mag.dirZ = -1 * tempDirX;

      //highG direction
      tempDirX = highG.dirX;
      tempDirY = highG.dirY;
      tempDirZ = highG.dirZ;
      highG.dirX = tempDirZ;
      highG.dirZ = -1 * tempDirX;
      //IMU rotation
      accel.ptrX = &accel.rawZ;
      accel.ptrZ = &accel.rawX;
      //highG rotation
      temPtr1 = highG.ptrZ;
      temPtr2 = highG.ptrX;
      highG.ptrX = temPtr1;
      highG.ptrZ = temPtr2;
      //IMU bias
      accel.biasX += g*val;
      //highG bias
      if((char)EEPROM.read(37)=='X'){highG.biasX += val*(int8_t)EEPROM.read(36)*high1G;}
      if((char)EEPROM.read(39)=='X'){highG.biasY += val*(int8_t)EEPROM.read(38)*high1G;}
      if((char)EEPROM.read(41)=='X'){highG.biasZ += val*(int8_t)EEPROM.read(40)*high1G;}}

      //write to EEPROM
      k = 24;
      //IMU
      accel.orientX = mag.orientX = gyro.orientX = 'Z';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'X';
      EEPROM.update(k, accel.dirX); k++;
      EEPROM.update(k, accel.orientX); k++;
      EEPROM.update(k, accel.dirY); k++;
      EEPROM.update(k, accel.orientY); k++;
      EEPROM.update(k, accel.dirZ); k++;
      EEPROM.update(k, accel.orientZ); k++;
      //highG
      highG.orientX = (char)EEPROM.read(41);
      highG.orientY = (char)EEPROM.read(39);
      highG.orientZ = (char)EEPROM.read(37);
      EEPROM.update(k, highG.dirX); k++;
      EEPROM.update(k, highG.orientX); k++;
      EEPROM.update(k, highG.dirY); k++;
      EEPROM.update(k, highG.orientY); k++;
      EEPROM.update(k, highG.dirZ); k++;
      EEPROM.update(k, highG.orientZ);}
      
  //Main IMU Y-axis is pointed to the nose
  else if(abs(accel.biasY) > abs(accel.biasX) && abs(accel.biasY) > abs(accel.biasZ)){
    //Y-axis pointed up
    if(accel.biasY > 0){
      Serial.println("Y-up Detected");
      //IMU direction
      tempDirX = accel.dirX;
      tempDirY = accel.dirY;
      tempDirZ = accel.dirZ;
      accel.dirY = gyro.dirY = mag.dirY = -1 * tempDirZ;
      accel.dirZ = gyro.dirZ = mag.dirZ = tempDirY;
      //highG direction
      tempDirX = highG.dirX;
      tempDirY = highG.dirY;
      tempDirZ = highG.dirZ;
      highG.dirY = -1 * tempDirZ;
      highG.dirZ = tempDirY;
      //IMU rotation
      accel.ptrY = &accel.rawZ;
      accel.ptrZ = &accel.rawY;
      //highG rotation
      temPtr1 = highG.ptrZ;
      temPtr2 = highG.ptrY;
      highG.ptrY = temPtr1;
      highG.ptrZ = temPtr2;
      //IMU bias
      accel.biasY -= g;
      //highG bias
      if((char)EEPROM.read(37)=='Y'){highG.biasX -= (int8_t)EEPROM.read(36)*high1G;}
      if((char)EEPROM.read(39)=='Y'){highG.biasY -= (int8_t)EEPROM.read(38)*high1G;}
      if((char)EEPROM.read(41)=='Y'){highG.biasZ -= (int8_t)EEPROM.read(40)*high1G;}}
 
    //Y-axis pointed down
    else if(accel.biasY < 0){
      Serial.println("Y-Down Detected");
      //IMU direction
      tempDirX = accel.dirX;
      tempDirY = accel.dirY;
      tempDirZ = accel.dirZ;
      accel.dirY = gyro.dirY = mag.dirY = tempDirZ;
      accel.dirZ = gyro.dirZ = mag.dirZ = -1 * tempDirY;
      //highG direction
      tempDirX = highG.dirX;
      tempDirY = highG.dirY;
      tempDirZ = highG.dirZ;
      highG.dirY = tempDirZ;
      highG.dirZ = -1 * tempDirY;
      //IMU rotation
      accel.ptrY = &accel.rawZ;
      accel.ptrZ = &accel.rawY;
      //highG rotation
      temPtr1 = highG.ptrZ;
      temPtr2 = highG.ptrY;
      highG.ptrY = temPtr1;
      highG.ptrZ = temPtr2;
      //IMU bias
      accel.biasY += g;
      //highG bias
      if((char)EEPROM.read(37)=='Y'){highG.biasX += (int8_t)EEPROM.read(36)*high1G;}
      if((char)EEPROM.read(39)=='Y'){highG.biasY += (int8_t)EEPROM.read(38)*high1G;}
      if((char)EEPROM.read(41)=='Y'){highG.biasZ += (int8_t)EEPROM.read(40)*high1G;}}
      
    //write to EEPROM
    k = 24;
    //IMU
    accel.orientX = mag.orientX = gyro.orientX = 'X';
    accel.orientY = mag.orientY = gyro.orientY = 'Z';
    accel.orientZ = mag.orientZ = gyro.orientZ = 'Y';
    EEPROM.update(k, accel.dirX); k++;
    EEPROM.update(k, accel.orientX); k++;
    EEPROM.update(k, accel.dirY); k++;
    EEPROM.update(k, accel.orientY); k++;
    EEPROM.update(k, accel.dirZ); k++;
    EEPROM.update(k, accel.orientZ); k++;
    //highG
    highG.orientX = (char)EEPROM.read(37);
    highG.orientY = (char)EEPROM.read(41);
    highG.orientZ = (char)EEPROM.read(39);
    EEPROM.update(k, highG.dirX); k++;
    EEPROM.update(k, highG.orientX);  k++;
    EEPROM.update(k, highG.dirY); k++;
    EEPROM.update(k, highG.orientY);  k++;
    EEPROM.update(k, highG.dirZ); k++;
    EEPROM.update(k, highG.orientZ);}

  //display values from EEPROM
  Serial.print(F("IMU.X: "));Serial.print((accel.dirX == 1) ? '+' : '-');Serial.println(accel.orientX);
  Serial.print(F("IMU.Y: "));Serial.print((accel.dirY == 1) ? '+' : '-');Serial.println(accel.orientY);
  Serial.print(F("IMU.Z: "));Serial.print((accel.dirZ == 1) ? '+' : '-');Serial.println(accel.orientZ);
  Serial.print(F("highG.X: "));Serial.print((highG.dirX == 1) ? '+' : '-');Serial.println(highG.orientX);
  Serial.print(F("highG.Y: "));Serial.print((highG.dirY == 1) ? '+' : '-');Serial.println(highG.orientY);
  Serial.print(F("highG.Z: "));Serial.print((highG.dirZ == 1) ? '+' : '-');Serial.println(highG.orientZ);
  }//end void

