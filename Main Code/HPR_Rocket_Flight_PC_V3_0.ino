//HPR Rocket Flight Computer
//Original sketch by Bryan Sparkman
//This is built for the Teensy3.5 board
//-----------Change Log------------
//V3_0 Final Version of the 3d Gen Initial Code
//Compatible Sentences: GPGGA, GPRMC, GNGGA, GNRMC
//--------FEATURES----------
//1500Hz 3-axis digital 24G and 100G accelerometer data logging
//1500Hz 3-axis digital 2000dps gyroscope data logging
//1500Hz of flight events
//1500Hz of integrated speed, altitude, rotation, continuity
//20Hz of digital barometric data logging
//20Hz of telemetry output (time, event, speed, altitude, accel, GPS, baro, rotation)
//10Hz of magnetic data logging
//8Hz of GPS data logging
//Mach immune events
//Sensor Fusion based apogee event
//Barometric based main deploy event
//Optional Apogee delay
//Optional Two-Stage mode
//Audible Continuity report at startup
//Audible Post-flight status report
//Audible Battery Voltage report at startup
//Separate file for each flight
//Optional test mode for bench testing
//Built-in self-calibration mode
//Reads user flight profile from SD card
//-------FUTURE UPGRADES----------
//User adjustable radio frequency, bandwidth, & modulation
//Magnetic sensor fusion
//Ground-station Adjustable Radio Frequency
//Auto-changing accelerometer scale for better precision
//Auto-sensing board orientation (X or Z axis up)
//Code compatibility across all sensors and hardware configurations
//------KNOWN PROBLEMS------
//need SD reading error catch
//--------PINOUTS----------
//COMMUNICATION PINS:
//I2C Bus: SDA - 3, SCL - 4
//SPI Bus: CS - 10, MOSI - 11, MISO - 12, SCK -13

//OUTPUT PINS:
//Separation Charge Fire: Pin 28
//Igniter Fire: Pin 31
//Apogee Charge Fire: Pin 22 
//Main Deploy Fire: Pin 20 
//Beeper: Pin 32

//INPUT PINS:
//Separation Charge Continuity: Pin 29 
//Igniter Continuity: Pin 30
//Apogee Charge Continuity: Pin 23
//Main Deploy Continuity: Pin 21

//EEPROM ALLOCATION:
//0 - 5: maximum altitude of last flight
//6 - 7: accelBiasX
//8 - 9: accelBiasY
//10-11: accelBiasZ
//12-13: HighGXbias
//14-15: HighGYbias
//16-17: HighGZbias
//18-19: HighGxDiffBias
//20   : Version Indicator

//-------CODE START--------
#include <SdFat.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <EEPROM.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <ADC.h>

//Radio setup
//#define RF95_FREQ     433.250
#define RFM95_RST     37//OK
#define RFM95_CS      10//OK
#define RFM95_IRQ     32//OK
RH_RF95 rf95(RFM95_CS, RFM95_IRQ);

//Teensy 3.5 Hardware Serial for GPS
HardwareSerial HWSERIAL(Serial1);

// GPS Setup
TinyGPSPlus GPS;

//SDIO Setup
SdFatSdioEX SD;
File outputFile;
File settingsFile;

//ADC Setup
ADC *adc = new ADC(); // adc object;

//GLOBAL VARIABLES
//-----------------------------------------
//Set code version
//-----------------------------------------
const float codeVersion = 2.20;
//-----------------------------------------
//Set defaults for user defined variables
//-----------------------------------------
boolean testMode = false;
boolean calibrationMode = false;
boolean silentMode = false; //true turns off beeper
boolean twoStage = false;
char rocketName[20] = ""; //Maximum of 20 characters
char callSign[7]= "";
int max_ang = 45; //degrees
boolean magSwitchEnable = false;
boolean radioTXmode = true;//false turns off radio transmissions
byte TXpwr = 13;
float TXfreq = 433.250;
int gTrigger = 3415; //2.5G trigger
unsigned long detectLiftoffTime = 500000UL; //0.5s
unsigned long apogeeDelay = 1000000UL; //1.0s apogee delay
int Alt_threshold = 120; //120m = 400ft
unsigned long sustainerFireDelay = 1000000UL; //1.0s
unsigned long separation_delay = 500000UL; //0.5s
float mainDeployAlt = 153;//Up to 458m for main deploy
unsigned long rcd_time = 900000000UL; //15min
unsigned long fireTime = 500000UL;//0.5s
//-----------------------------------------
//GPIO pin mapping
//-----------------------------------------
/*const byte    sepFpin = 34;
const byte    ignFpin = 38;
const byte    apogeeFpin = 26;
const byte    mainFpin = 16;
      byte    beepPin = 23;
const byte    sepCpin = 35;
const byte    ignCpin = 27;
const byte    apogeeCpin = 39;
const byte    mainCpin = 17;
const byte    buttonGnd = 3;
const byte    buttonRead = 6;
const byte    magSwitchPin = 21;*/
//-----------------------------------------
//GPIO pin mapping
//-----------------------------------------
byte nullCpin = 56;
byte nullFpin = 57;
byte sepFpin = 57;
byte ignFpin = 57;
byte apogeeFpin = 57;
byte mainFpin = 57;
byte sepCpin = 56;
byte ignCpin = 56;
byte apogeeCpin = 56;
byte mainCpin = 56;
byte airStartCpin = 56;
byte airStartFpin = 56;
byte beepPin = 23;//OK
byte buttonGnd = 2;//OK
byte buttonRead = 5;//OK
const byte magSwitchPin = 33;//OK
char pyro1 = 'B';
const byte pyro1Cpin = 39;//OK
const byte pyro1Fpin = 38;//OK
char pyro2 = 'I';
const byte pyro2Cpin = 15;//OK
const byte pyro2Fpin = 14;//OK
char pyro3 = 'A';
const byte pyro3Cpin = 20;//OK
const byte pyro3Fpin = 19;//OK
char pyro4 = 'M';
const byte pyro4Cpin = 22;//OK
const byte pyro4Fpin = 21;//OK
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
boolean sustainerIgnition = false;
boolean sustainerBurnout = false;
boolean onGround = true;
uint8_t radioEvent = 0;
int16_t radioAccelAlt;
int16_t radioBaroAlt;
int16_t radioGPSalt;
int16_t radioInt;
int16_t radioMaxG = 0;
int16_t radioAccelNow;
int16_t radioVel = 0;
float radioPeakVel = 0.0;
//-----------------------------------------
//flight events
//-----------------------------------------
boolean preLiftoff = true;
boolean liftoff = false;
boolean boosterBurnout = false;
boolean boosterBurnoutCheck = false;
boolean boosterSeparation = false;
boolean sustainerFireCheck = false;
boolean sustainerFire = false;
boolean apogee = false;
boolean apogeeFire = false;
boolean apogeeSeparation = false;
boolean mainDeploy = false;
boolean touchdown = false;
boolean timeOut = false;
boolean fileClose = false;
boolean rotation_OK = true;
boolean Alt_excd = false;
boolean beep = false;
boolean pyroFire = false;
boolean contApogee = false;
boolean contMain = false;
boolean contStage = false;
boolean contSep = false;
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
//-----------------------------------------
//digital accelerometer variables
//-----------------------------------------
int g = 1366;
int accelBiasX = 0; 
int accelBiasY = 0;
int accelBiasZ = 0;
const float convertG = 66.93989; //66.93989 = HighGgain / digitalGain = 0.049/0.000732
long accelX0sum = 0;
long accelY0sum = 0;
long accelZ0sum = 0;
int accelX0 = 0;
int accelY0 = 0;
int accelZ0 = 0;
int accelX;
int accelY;
int accelZ;
float accelNow;
float maxG = 0.0;
float accelGain = 0.000735;
byte accelAddress;
byte accelRegister;
//-----------------------------------------
//High-G accelerometer variables
//-----------------------------------------
int highG = 109;
float highGgain = 0.013;
long highGsum = 0L;
int highGfilter[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t highGx;
uint16_t highGy;
uint16_t highGz;
uint16_t highGtest;
int16_t highGxDiff;
int highGxBias = 0;
int highGyBias = 0;
int highGzBias = 0;
int highGxDiffBias = 0;
uint16_t highGx0 = 0;
uint16_t highGy0 = 0;
uint16_t highGz0 = 0;
int16_t highGx0Diff = 0;
byte filterPosn = 0;
float filterAccel;
long highGsumX0 = 0L;
long highGsumY0 = 0L;
long highGsumZ0 = 0L;
long highGsumX0diff = 0L;
const float A2D = 12.15139442; //12.15139442 = analogGain / digitalGain = 0.00915/0.000753
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
const unsigned long timeBtwnBMP = 50000; //sample once every 50ms
const unsigned long bmpMeasureTime = 45000; //need 43200us for measurement oversampling
unsigned long bmpMeasureStart = 0UL;
unsigned long lastBMP = 0UL;
byte bmp_case = 1;
unsigned long bmp_counter = 0UL;
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
float baroVel5[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float baroVelSum = 0.0;
float baroVel = 0.0;
byte baroVelPosn = 0;
//-----------------------------------------
//Magnetometer Variables
//-----------------------------------------
unsigned long magTrigger = 100000UL;
unsigned long magCounter = 0UL;
int magX0 = 0;
int magY0 = 0;
int magZ0 = 0;
long magX0sum = 0;
long magY0sum = 0;
long magZ0sum = 0;
int magX;
int magY;
int magZ;
byte magAddress;
byte magRegister;
//-----------------------------------------
//gyro variables
//-----------------------------------------
int gyroBiasX = 0;
int gyroBiasY = 0;
int gyroBiasZ = 0;
int gyroX;
int gyroY;
int gyroZ;
long dx = 0L;
long dy = 0L;
long dz = 0L;
float yawY0;
float pitchX0;
int pitchX;
int yawY;
int rollZ = 0;
const float gyroDegLSB = 0.07; //degrees per LSB
const long oneDeg = 14285714L; //(long)((float)1000000/(float)gyroDegLSB);//oneDeg = mln/0.070 = 14285714L;
const long oneTenthDeg = oneDeg/10;//oneTenthDeg = oneDeg/10 = 1428571L
const float degRad = 57.296; //degrees per radian
const float mlnth = 0.000001;
//-----------------------------------------
//DCM Integration variables
//-----------------------------------------
float cosX = 1.0;
float sinX = 0.0;
float PrevCosX = 1.0;
float PrevSinX = 0.0;
float tanPitch;
float tanYaw;
int counterSign = 1;
boolean calcOffVert = false;
int offVert = 0;
boolean rotationFault = false;
byte gyroAddress;
byte gyroRegister;
//-----------------------------------------
//Quaternion Integration variables
//-----------------------------------------
boolean quatEnable = false;
float Quat[5] = {0.0, 1.0, 0.0, 0.0, 0.0};
float ddx;
float ddy;
float ddz;
unsigned long lastRotn = 0UL;
unsigned long rotnRate = 10000UL;//100 updates per second
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
//-----------------------------------------
//Firing variables
//-----------------------------------------
byte firePin = 0;
unsigned long timeFireBegin;
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
int voltage = 0;
boolean reportCode = true;//true = report max altitude, false = report max velocity
byte postFlightCode = 0;
const char cs = ',';
const byte num7 = 7;
const byte num6 = 6;
const byte num5 = 5;
const byte num4 = 4;
const byte num0 = 0;
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
boolean configGPSdefaults = false;
boolean configGPSflight = false;
unsigned long timeLastGPS = 0UL;
uint16_t fixCount = 0;
boolean GPSpsm = false;
//-----------------------------------------
//Stabilization Variables
//-----------------------------------------
boolean stableRotn = false;
boolean stableVert = false;
//-----------------------------------------
//debug
//-----------------------------------------
long debugStart;
long debugTime;

void setup(void) {
  
  //Start Harware Serial communication
  HWSERIAL.begin(9600);
  
  //Start I2C communication
  Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, 400000);

  //Start SPI communication
  SPI.begin();

  //Start SDIO Comms with the SD card
  SD.begin();

  //check if the test mode button is being held
  pinMode(buttonRead, INPUT_PULLUP);    //test mode button
  pinMode(buttonGnd, OUTPUT);           //test mode button
  digitalWrite(buttonGnd, LOW);
  delay(50);
  if(digitalRead(buttonRead) == LOW){testMode = true; Serial.println(F("Test Mode Confirmed"));}

  //Start Sensors
  if(testMode){Serial.println(F("Starting Sensors..."));}
  boolean status_MPL3115A2 = beginMPL3115A2();
  boolean status_LSM9DS1 = beginLSM9DS1();
  boolean status_H3LIS331DL = beginH3LIS331DL(2);
  boolean status_ADXL377 = beginADXL377();

  //Start the radio
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  
  //Radio manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  //Initialize the radio
  boolean status_RFM96W = rf95.init();
  if(!status_RFM96W){digitalWrite(beepPin, HIGH);
    delay(200);
    digitalWrite(beepPin, LOW);
    delay(200);
    digitalWrite(beepPin, HIGH); 
    delay(200);
    digitalWrite(beepPin, LOW);
    delay(1000);}
  rf95.setFrequency(TXfreq);
  radioDelay = RDpreLiftoff;

  //Read the battery votage from the main parachute continuity pin
  pinMode(A15, INPUT);
  voltage = analogRead(A15)*(3.3*3.2*10/65536);
  delay(50);
  
  //Set the mode of the output pins
  pinMode(nullCpin, INPUT);
  pinMode(nullFpin, INPUT);
  pinMode(pyro1Cpin, INPUT);           
  pinMode(pyro2Cpin, INPUT);          
  pinMode(pyro3Cpin, INPUT);         
  pinMode(pyro4Cpin, INPUT);           
  pinMode(pyro1Fpin, OUTPUT);             
  pinMode(pyro2Fpin, OUTPUT);            
  pinMode(pyro3Fpin, OUTPUT);
  pinMode(pyro4Fpin, OUTPUT);   
  pinMode(beepPin, OUTPUT);             //beeper
  pinMode(magSwitchPin, INPUT_PULLUP);  //mag switch pin
    
  if(!status_LSM9DS1 && testMode){Serial.println(F("LSM9DS1 not detected!"));}
  if(status_LSM9DS1 && testMode){Serial.println(F("LSM9DS1 OK!"));}
  if(!status_MPL3115A2 && testMode){Serial.println(F("MPL3115A2 not detected!"));}
  if(status_MPL3115A2 && testMode){Serial.println(F("MPL3115A2 OK!"));}
  if(!status_ADXL377 && testMode){Serial.println(F("ADXL377 not detected!"));}
  if(status_ADXL377 && testMode){Serial.println(F("ADXL377 OK!"));}
  if(!status_H3LIS331DL && testMode){Serial.println(F("H3LIS331DL not detected!"));}
  if(status_H3LIS331DL && testMode){Serial.println(F("H3LIS331DL OK!"));}
  if(!status_RFM96W && testMode){Serial.println(F("RFM96W not detected!"));}
  if(status_RFM96W && testMode){Serial.println(F("RFM96W OK!"));}
  
 if(testMode){Serial.println(F("Reading User Settings from SD Card"));}
  
  //Open the settings file
  settingsFile.open("Settings.txt", O_READ);

  //Read in the user defined variables
  parseNextVariable(false);n=0;
  while (dataString[n]!='\0'){rocketName[n] = dataString[n];n++;}rocketName[n]='\0';n=0;
  parseNextVariable(false);
  while (dataString[n]!='\0'){callSign[n] = dataString[n];n++;}callSign[n]='\0';n=0;
  magSwitchEnable = (boolean)(parseNextVariable(true));
  radioTXmode = (boolean)parseNextVariable(true);
  TXpwr = (byte)(parseNextVariable(true));
  TXfreq = (float)(parseNextVariable(true));
  silentMode = (boolean)(parseNextVariable(true));
  setupTime = (unsigned long)(parseNextVariable(true)*1000UL);
  gTrigger = (int)(parseNextVariable(true)*g);
  detectLiftoffTime = (unsigned long)(parseNextVariable(true)*1000000UL);
  apogeeDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  mainDeployAlt = (float)(parseNextVariable(true)/3.2808);
  rcd_time = (unsigned long)(parseNextVariable(true)*1000000UL);
  fireTime = (unsigned long) (parseNextVariable(true)*1000000UL);
  parseNextVariable(false); pyro4 = dataString[0];
  parseNextVariable(false); pyro3 = dataString[0];
  parseNextVariable(false); pyro2 = dataString[0];
  parseNextVariable(false); pyro1 = dataString[0];
  twoStage = (boolean)parseNextVariable(true);
  sustainerFireDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  separation_delay = (unsigned long)(parseNextVariable(true)*1000000UL);
  Alt_threshold = (int)(parseNextVariable(true)/3.2808);
  max_ang = (int)parseNextVariable(true)*10;
  quatEnable = (boolean)parseNextVariable(true);

  //close the settings file
  settingsFile.close();

  //configure pyro outupts
  configPyro(pyro1, pyro1Cpin, pyro1Fpin);
  configPyro(pyro2, pyro2Cpin, pyro2Fpin);
  configPyro(pyro3, pyro3Cpin, pyro3Fpin);
  configPyro(pyro4, pyro4Cpin, pyro4Fpin);

  if(testMode){
  Serial.print(F("Pyro 4 Function: ")); Serial.println(pyro4);
  Serial.print(F("Pyro 3 Function: ")); Serial.println(pyro3);
  Serial.print(F("Pyro 2 Function: ")); Serial.println(pyro2);
  Serial.print(F("Pyro 1 Function: ")); Serial.println(pyro1);}
  
  //safety override of manual variables
  if (gTrigger < 1.5 * g) {gTrigger = 1.5 * g;} //min 1.5G trigger
  if (gTrigger > 5 * g) {gTrigger = 5 * g;} //max 5G trigger
  if (detectLiftoffTime < 100000UL) {detectLiftoffTime = 100000UL;} //.1s min gTrigger detection
  if (detectLiftoffTime > 1000000UL) {detectLiftoffTime = 1000000UL;} //1s max gTrigger detection
  if (apogeeDelay > 5000000UL) {apogeeDelay = 5000000UL;} //5s max apogee delay
  if (fireTime > 1000000UL) {fireTime = 1000000UL;} //1s max firing length
  if (mainDeployAlt > 458){mainDeployAlt = 458;}//max of 1500 ft
  if (mainDeployAlt < 30) {mainDeployAlt = 30;}//minimum of 100ft
  if (sustainerFireDelay > 8000000UL){sustainerFireDelay = 8000000UL;}//maximum 8s 2nd stage ignition delay
  if (separation_delay > 3000000UL){separation_delay = 3000000UL;}//max 3s booster separation delay after burnout
  if (Alt_threshold < 91){Alt_threshold = 91;}//minimum 100ft threshold
  if (max_ang > 450){max_ang = 450;}//maximum 90 degree off vertical
  if (rcd_time < 300000000UL){rcd_time = 300000000UL;}//min 5min of recording time
  if (fireTime < 200000UL){fireTime = 200000UL;}//min 0.2s of firing time
  if (fireTime > 1000000UL){fireTime = 1000000UL;}//max 1.0s of firing time
  if (setupTime > 60000UL) {setupTime = 60000UL;}//max 60 seconds from power-on to preflight start
  if (setupTime < 3000UL) {setupTime = 3000UL;}//min 3 seconds of setup time
  if (TXpwr > 23){TXpwr = 23;}
  if (TXpwr < 1){TXpwr = 1;}

  //check for silent mode
  if(testMode && silentMode){beepPin = 57; Serial.println(F("Silent Mode Confirmed"));}
    
  //check for disabling of the telemetry
  if(testMode && !radioTXmode){Serial.println(F("Telemetry OFF!"));}
  if(silentMode && !radioTXmode){beepPin = 13; pinMode(beepPin, OUTPUT);}
  
  //signal if in test-mode
  if (testMode){

    Serial.println(F("Signaling Test Mode"));
    beep_counter = 0;
    beep_delay = long_beep_delay;
    while(beep_counter < 8){
      
      timeClock = micros();

      //Look for the user to release the button
      if(digitalRead(buttonRead) == HIGH){testMode = false;delay(10);}

      //Look for the user to put it into calibration mode
      if(digitalRead(buttonRead) == LOW && !testMode){
        calibrationMode = true;
        testMode = true;
        beep_counter = 8;
        digitalWrite(beepPin, LOW);
        beep = false;}

      //starts the beep
      if (!beep && timeClock - timeLastBeep > beep_delay){
          digitalWrite(beepPin, HIGH);
          timeBeepStart = timeClock;
          beep = true;
          beep_counter++;}
      
      //stops the beep
      if(beep && (timeClock - timeBeepStart > 500000UL)){
        digitalWrite(beepPin, LOW);
        timeBeepStart = 0UL;
        timeLastBeep = timeClock;
        beep = false;}
      }//end while

    //Reset variables
    beep_counter = 0;
    timeBeepStart = 0UL;
    timeLastBeep = 0UL;
    timeClock = 0UL;
    if(!calibrationMode){testMode = true;}}//end testMode

  //calibration mode
  if(calibrationMode){
    
    Serial.println(F("Calibration Mode Confirmed. Please orient altimeter"));
    
    for (byte i = 1; i < 20; i++){
      digitalWrite(beepPin, HIGH);
      delay(250);
      digitalWrite(beepPin, LOW);
      delay(250);}

    digitalWrite(beepPin, HIGH);

    Serial.println(F("Calibrating - Hold altimeter still"));

    for (byte i = 1; i < 101; i++){
      getAccel();
      Serial.println(accelX);
      getADXL377(false);
      getADXL377(true);
      accelX0sum += (accelX - g);
      accelY0sum += accelY;
      accelZ0sum += accelZ;
      highGsumX0 += (highGx - highG);
      highGsumY0 += highGy;
      highGsumZ0 += highGz;
      highGsumX0diff += (highGxDiff - highG);
      delay(300);}
      
    //calculate the bias
    accelBiasX = (int)(accelX0sum / 100);
    accelBiasY = (int)(accelY0sum / 100);
    accelBiasZ = (int)(accelZ0sum / 100);
    highGxBias = (int)(highGsumX0 / 100);
    highGyBias = (int)(highGsumY0 / 100);
    highGzBias = (int)(highGsumZ0 / 100);
    highGxDiffBias = (int)(highGsumX0diff / 100);

    //Store in EEPROM    
    writeCalibration(accelBiasX, 6);
    writeCalibration(accelBiasY, 8);
    writeCalibration(accelBiasZ, 10);
    writeCalibration(highGxBias, 12);
    writeCalibration(highGyBias, 14);
    writeCalibration(highGzBias, 16);
    writeCalibration(highGxDiffBias, 18);
    
    digitalWrite(beepPin, LOW);
    Serial.println(F("Calibration complete!  Calculated values are:"));
    Serial.print(F("accelBiasX: "));Serial.println(accelBiasX);
    Serial.print(F("accelBiasY: "));Serial.println(accelBiasY);
    Serial.print(F("accelBiasZ: "));Serial.println(accelBiasZ);
    Serial.print(F("highGxBiasX: "));Serial.println(highGxBias);
    Serial.print(F("highGyBiasY: "));Serial.println(highGyBias);
    Serial.print(F("highGzBiasZ: "));Serial.println(highGzBias);
    Serial.print(F("highGxDiffBias: "));Serial.println(highGxDiffBias);
    }//end calibration mode

  //read the bias from EEPROM  
  calUnion.calByte[0]=EEPROM.read(6); calUnion.calByte[1]=EEPROM.read(7);
  accelBiasX = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(8); calUnion.calByte[1]=EEPROM.read(9);
  accelBiasY = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(10); calUnion.calByte[1]=EEPROM.read(11);
  accelBiasZ = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(12); calUnion.calByte[1]=EEPROM.read(13);
  highGxBias = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(14); calUnion.calByte[1]=EEPROM.read(15);
  highGyBias = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(16); calUnion.calByte[1]=EEPROM.read(17);
  highGzBias = calUnion.calValue;
  calUnion.calByte[0]=EEPROM.read(18); calUnion.calByte[1]=EEPROM.read(19);
  highGxDiffBias = calUnion.calValue;
  
  //output the read values
  if(testMode){
    Serial.println(F("Calibrataion values read from EEPROM:"));
    Serial.print(F("accelBiasX: "));Serial.println(accelBiasX);
    Serial.print(F("accelBiasY: "));Serial.println(accelBiasY);
    Serial.print(F("accelBiasZ: "));Serial.println(accelBiasZ);
    Serial.print(F("highGxBiasX: "));Serial.println(highGxBias);
    Serial.print(F("highGyBiasY: "));Serial.println(highGyBias);
    Serial.print(F("highGzBiasZ: "));Serial.println(highGzBias);
    Serial.print(F("highGxDiffBias: "));Serial.println(highGxDiffBias);
    }

  //restart the highG accelerometer at the higher rate
  if(status_H3LIS331DL){beginH3LIS331DL(1);}

  //Set the radio output power
  rf95.setTxPower(TXpwr, false);//23 max setting; 20mW=13dBm, 30mW=15dBm, 50mW=17dBm, 100mW=20dBm
  
  //Overrides User Settings for bench test mode
  if (testMode) {
    //rf95.setTxPower(13, false);//10% power, or 20mW
    detectLiftoffTime = 10000UL; //0.01s
    setupTime = 3000UL; //3s startup time
    apogeeDelay = 1000000UL; //1s apogee delay
    rcd_time = 15000000UL; //15s record time
    gTrigger = (int)(1.5*g); //1.5G trigger
    maxAltitude = 11101/3.2808;
    maxVelocity = 202/3.2808;
    RDpreLiftoff = 1000000UL;
    RDpostFlight = 1000000UL;
    radioDelay = RDpreLiftoff;
    magSwitchEnable = false;
    //g += (abs(accelBiasX) + 50);
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

  if(testMode){Serial.print(F("Creating new SD card file: FLIGHT"));}
  
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
  outputFile.print(rocketName);
  outputFile.print(F(" Code V"));
  outputFile.print(codeVersion);
  outputFile.println(F(",accelX,accelY,accelZ,gyroX,gyroY,gyroZ,rotnZ,rotnY,rotnX,offVert,intVel,intAlt,fltEvents,pyroCont,pyroFire,pyroPin,highGx,baroAlt,baroPress,baroTemp,battVolt,magX,magY,magZ,gpsLat,gpsLon,gpsSpeed,gpsAlt,gpsAngle,gpsSatellites,radioPacketNum"));
  outputFile.sync();

  if(testMode){
    if(n<10){Serial.print('0');Serial.println(n);}
    else{Serial.println(n);}}

  //check continuity
  if (digitalRead(apogeeCpin) == HIGH) {contApogee = true;}
  if (digitalRead(mainCpin) == HIGH) {contMain = true;}
  if (digitalRead(sepCpin) == HIGH) {contSep = true;}
  if (digitalRead(ignCpin) == HIGH) {contStage = true;}

  //Report single-stage pre-flight status
  if (!twoStage){
    if (contMain && contApogee) {beepCode = 3;}
    else if (contMain){beepCode = 2;}
    else if (contApogee) {beepCode = 1;}
    else {beepCode = 4;}
    postFlightCode = 1;}

  //Report two-stage pre-flight status
  if (twoStage){
    if (contSep && contStage && contApogee && contMain) {beepCode = 4;}
    else {
      contError = true;
      if (!contSep) {beepCode = 1;}
      else if(!contStage) {beepCode = 2;}
      else if (!contApogee) {beepCode = 3;}
      else if (!contMain){beepCode = 5;}}}

  if(testMode){Serial.print(F("Reporting continuity: "));Serial.println(beepCode);}
  
  //set the beep delay and preflight beep code
  beep_delay = long_beep_delay;
  
  //if the magnetic switch is enabled, beep the continuity code until the magnet is sensed
  if(magSwitchEnable){
    byte ii = 0;
    boolean magTrigger = false;
    getMag();
    while(!magTrigger){
      ii=0;
      while(!magTrigger && ii < beepCode){
        digitalWrite(beepPin, HIGH);
        delay(250);
        digitalWrite(beepPin, LOW);
        delay(250);
        getMag();
        Serial.print(magX);Serial.print(',');Serial.print(magY);Serial.print(',');Serial.println(magZ);
        if(abs(magX) > 30000 || abs(magY) > 30000 || abs(magZ) > 30000){magTrigger = true;}
        ii++;}//end inner loop
      ii=0;
      while(!magTrigger && ii < 10){delay(100);ii++;}
      getMag();
      if(abs(magX) > 30000 || abs(magY) > 30000 || abs(magZ) > 30000){magTrigger = true;}
      }//end outer loop
      }//end if magswitch

  if(testMode){Serial.println(F("Hold Rocket Vertical"));}
  //wait for the rocket to be installed vertically
  digitalWrite(beepPin, HIGH);
  delay(setupTime);
  digitalWrite(beepPin, LOW);
  delay(500);

  if(testMode){Serial.println(F("Sampling Gyro"));}
  //sample the sensors 100 times over 3 seconds to determine the offsets and initial values
  highGxDiffBias = 0;
  for (byte i = 1; i < 101; i++) { 
    //get a gyro event
    getGyro();
    getAccel();
    getADXL377(false);
    getADXL377(true);
    getMag();
  
    //add up the gyro samples
    gyroBiasX += gyroX;
    gyroBiasY += gyroY;
    gyroBiasZ += gyroZ;

    //add up the accelerometer samples
    accelX0sum += accelX - accelBiasX;
    accelY0sum += accelY - accelBiasY;
    accelZ0sum += accelZ - accelBiasZ;   

    //add up the analog accelerometer samples
    highGx0 += highGx - highGxBias;
    highGx0Diff += highGxDiff - highGxDiffBias;
    
    //add up the magnetometer samples
    magX0 += magX;
    magY0 += magY;
    magZ0 += magZ;

    //sample over a period of 3 seconds
    delay(30);}

  //Divide by 100 to set the average of 100 samples
  gyroBiasX /= 100;
  gyroBiasY /= 100;
  gyroBiasZ /= 100;
  (int)(highGx0 /= 100);
  (int)(highGx0Diff /= 100);
  accelX0 = (int)(accelX0sum / 100);
  accelY0 = (int)(accelY0sum / 100);
  accelZ0 = (int)(accelZ0sum / 100);
  magX0 = (int)(magX0sum / 100);
  magY0 = (int)(magY0sum / 100);
  magZ0 = (int)(magZ0sum / 100);
  if(testMode){Serial.println(F("Sampling complete"));}

  //Calibrate the analog accelerometer to the digital one
  highGxBias = highGx0 - (int)((float)accelX0 / (float)A2D);
  highGxDiffBias = highGx0Diff - (int)((float)accelX0 / (float)A2D) - 27;
  
  //Compute the acceleromter based rotation angle
  if (accelZ0 >= 0) {yawY0 = asin(min(1, (float)accelZ0 / (float)g)) * degRad;}
  else {yawY0 = asin(max(-1, (float)accelZ0 / (float)g)) * degRad;}

  if (accelY0 >= 0) {pitchX0 = asin(min(1, (float)accelY0 / (float)g)) * degRad;}
  else {pitchX0 = asin(max(-1, (float)accelY0 / (float)g)) * degRad;}

  if(quatEnable){
    //update quaternion
    getQuatRotn(0, yawY0/degRad, pitchX0/degRad);}
  else{ 
    //Initialize the rotation angles
    yawY = int(yawY0*(float)10);
    pitchX = int(pitchX0*(float)10);}
  if(testMode){Serial.println(F("Rotation Computation Complete"));}
  offVert = (int)atan(pow(tan(pitchX0/degRad),2) + pow(tan(yawY0/degRad),2)) * degRad * 10;

  //restore the GPS factory defaults
  //restoreGPSdefaults();
  configGPSdefaults = true; 
  
  //Reset the G-trigger
  gTrigger -= accelBiasX;

  //set the booster burp check time
  boosterBurpTime = min(1000000UL, separation_delay-10000UL);
  
   //Read main deploy setting into its beep array
  parseBeep(long(10*int(mainDeployAlt*.32808)), maxVelDigits, num4);
  if(testMode){Serial.print(F("Reporting Main Deploy Settings: "));Serial.println((int)(mainDeployAlt*3.2808));}
  //Beep out the main deployment altitude setting
  while (maxVelDigits[velDigits-1]==0){velDigits--;}  
  for(byte i = velDigits; i > 0; i--){
    delay(800);
    for(byte j = maxVelDigits[i-1]; j > 0; j--){
      digitalWrite(beepPin, HIGH);
      delay(100);
      digitalWrite(beepPin, LOW);
      delay(100);}}
  velDigits = 4;
  delay(2000);

  byte k = EEPROM.read(0);
  //Write initial values into EEPROM
  if(k == (byte)255){
    if(testMode){Serial.print(F("Initial Use Detected: Writing Initial EEPROM values for prior flight"));}
    for(byte j = 0; j <6; j++){EEPROM.write(j,j);}}
  for(byte j = 0; j <6; j++){EEPROM.write(j,j);}

  //Beep out the last flight's altitude
  if(testMode){Serial.print(F("Reporting last flight: "));}
  for(byte i=0;i<6;i++){maxAltDigits[i]=EEPROM.read(i);}
  while (maxAltDigits[altDigits-1]==0 && altDigits > 0){altDigits--;}  
  for(byte i = altDigits; i > 0; i--){
    delay(800);
    if(testMode){Serial.print(maxAltDigits[i-1]);}
    for(byte j = maxAltDigits[i-1]; j > 0; j--){
      digitalWrite(beepPin, HIGH);
      delay(100);
      digitalWrite(beepPin, LOW);
      delay(100);}}
  altDigits = 6;
  delay(2000);

  //Beep out the battery voltage
  if(testMode){Serial.println(" "); Serial.print(F("Reporting Battery Voltage: "));Serial.println((float)(voltage*0.1));}
  parseBeep(voltage, voltageDigits, 2);
  for(byte i = 0; i < 2; i++){
    delay(800);
    for(byte j = voltageDigits[1-i]; j > 0; j--){
      digitalWrite(beepPin, HIGH);
      delay(100);
      digitalWrite(beepPin, LOW);
      delay(100);}}
  delay(2000);

  if(testMode){
    Serial.println("Setup Complete.  Awaiting simulated launch.");
    Serial.print("Beginning GPS NMEA Serial Output and Continuity Reporting: ");
    Serial.println(beepCode);
    delay(3000);}
}//end setup

void loop(void){

  //Sample the Accelerometer & Gyro w/ timestamps
  getAccelGyro();
 
  //Get a barometric event if needed
  if(micros()-lastBMP >= timeBtwnBMP){
      prevBaroAlt = Alt;
      readMPLbaro();
      lastBMP = micros();
      prevBaroTime = baroTime;
      baroTime = lastBMP;
      if(Alt > maxAltitude && !apogee){maxAltitude = Alt;}
      if(preLiftoff){baseAlt = Alt;}
      newBMP=true;}
      
  //look for a shutdown command and if seen, stop all progress for a hard reset
  if (preLiftoff && magSwitchEnable && digitalRead(magSwitchPin) == LOW){
      n=1;
      while(n==1){digitalWrite(beepPin,HIGH);delay(1000);}}
   
  if (!liftoff && accelX > gTrigger && !touchdown && !timeOut) {
    //timeGyroClock = timeClockPrev; //initializes timeGyro to an appropriate value
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

    //Get High-G Accelerometer Data
    getADXL377(true);

    //Update master gyro timing variables
    gdt = long(timeGyroClock - timeGyroClockPrev);
    timeGyro += (unsigned long)gdt;

    //update master timing variables
    dt = timeClock - timeClockPrev;
    timeCurrent += dt;

    //See if a new altitude reading is available
    if(newBMP){

      Alt = Alt - baseAlt;
      if(Alt > maxAltitude && !apogee){maxAltitude = Alt;}
      //Barometric apogee trigger
      baroApogee = int(Alt) - baroLast5[baroApogeePosn];
      baroLast5[baroApogeePosn] = int(Alt);
      baroApogeePosn++;
      if(baroApogeePosn == 5){baroApogeePosn = 0;}
      //Baro touchdown trigger
      if (mainDeploy && baroApogee == 0) {baroTouchdown ++;}
      else {baroTouchdown = 0;}
      //barometric velocity
      baroVelSum -= baroVel5[baroVelPosn];
      baroVel5[baroVelPosn] = (Alt - prevBaroAlt)/((float)(baroTime - prevBaroTime)*mlnth);
      baroVelSum += baroVel5[baroVelPosn];
      baroVel = baroVelSum * 0.2;
      baroVelPosn++;
      if(baroVelPosn > 4){baroVelPosn = 0;}
      radioVel = (int16_t)baroVel;}

    //Get magnetometer data
    magCounter += dt;
    if (magCounter >= magTrigger){
      getMag();
      magCounter = 0;}

    //Update the moving average of 15 high-G accelerometer points to significantly reduce noise
    highGsum -= highGfilter[filterPosn];
    highGfilter[filterPosn] = highGxDiff;
    highGsum += highGfilter[filterPosn];
    filterPosn++;
    if(filterPosn == 15){filterPosn = 0;}
    filterAccel = (float)highGsum * 0.066666667;

    //Compute the current g-load. Use the high-G accelerometer if the standard one is pegged
    if(abs(accelX) < 31500){accelNow = (float)(accelX - g) * 0.0071783945;} //0.007178395 = gain * G =  0.000753 *9.80655
    else{accelNow = (filterAccel - (float)highG) *  0.0897299325;} //0.0897299325 = gain * G = 0.00915 *9.80655
        
    //Integrate velocity, altitude, and rotation data prior to apogee
    if(!apogee || testMode){
  
      //Capture the max acceleration
      if (accelNow > maxG){maxG = accelNow;}
      
      //calculate the new acceleration based velocity
      //this makes the apogee event mach immune
      accelVel += accelNow * (float)dt * mlnth;

      //compute the smoothed velocity to send over the radio
      if(accelVel  < 300 &&  accelVel > radioPeakVel){radioPeakVel = accelVel;}
      if(!boosterBurnout || (accelVel > 300)){radioVel = (int16_t)accelVel;}
      else if(!apogee){float frac = accelVel/radioPeakVel; radioVel = (int16_t)((frac)*accelVel + (1.0 - frac)*baroVel);}
      
      //update maximum velocity if it exceeds the previous value
      if(accelVel > maxVelocity){maxVelocity = accelVel;}
    
      //calculate the new acceleration based altitude
      accelAlt += accelVel * (float)dt * mlnth;
      if(!Alt_excd && (accelAlt > Alt_threshold || testMode)){Alt_excd = true;}

      if(quatEnable){
        //get the quaternion rotation
        //caluclate the partial rotation
        const float deg2rad = 0.00122173;
        dx += gyroZ * gdt;
        dy += gyroY * gdt;
        dz -= gyroX * gdt;

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
      if(!quatEnable){getRotnDCM2D();}
      
      }//end if !apogee
  
    //Check for timeout
    if (!timeOut && !pyroFire && timeCurrent > rcd_time) {
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
      if (timeCurrent > detectLiftoffTime) {checkFalseTrigger = false;}
      if (accelX < gTrigger && accelVel < 15.5F) {
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
        filterAccel = 0;
        highGsum = 0;
        for(byte i=0; i<10; i++){highGfilter[i]=0;}
        baroTouchdown = 0;
        accelVel = 0;
        accelAlt = 0;
        packetnum = 1;
        radioDelay = RDpreLiftoff;
        lastTX = 0UL;}
    }//end checkFalseTrigger

    //check for booster burnout: if the x acceleration is negative
    if (!boosterBurnout && liftoff && accelX <= 0) {
      boosterBurnout = true;
      radioEvent = 2;
      boosterBurnoutCheck = true;
      timeLastEvent = timeCurrent;}
      
    //check for booster motor burp for 1 second after burnout is detected
    if (boosterBurnoutCheck){
      if(timeCurrent - timeLastEvent > boosterBurpTime){boosterBurnoutCheck = false;}
      else if (boosterBurnout && !testMode && accelX > 0){boosterBurnout = false; boosterBurnoutCheck = false; radioEvent = 1;}}

    //Fire separation charge if burnout is detected and time is past the separation delay
    if (!boosterSeparation && twoStage && liftoff && boosterBurnout && !checkFalseTrigger && timeCurrent - timeLastEvent > separation_delay) {
      boosterSeparation = true;
      radioEvent = 3;
      timeLastEvent = timeCurrent;
      firePin = sepFpin;
      timeFireBegin = timeCurrent;
      pyroFire = true;
      //Fire separation charge
      digitalWrite(firePin, HIGH);}

    //Fire second stage
    if (twoStage && !sustainerFireCheck && (!apogee || testMode) && liftoff && boosterBurnout && boosterSeparation && !pyroFire && timeCurrent - timeLastEvent > sustainerFireDelay) {
      sustainerFireCheck = true;
      postFlightCode = 1;
      timeLastEvent = timeCurrent;
      //Check for staging inhibit and fire if OK
      if (Alt_excd && rotation_OK) {
        sustainerFire = true;
        firePin = ignFpin;
        timeFireBegin = timeCurrent;
        radioEvent = 4;
        pyroFire = true;
        //Fire second stage
        digitalWrite(firePin, HIGH);}
      else if (!rotation_OK && !Alt_excd){postFlightCode = 4; radioEvent = 12;}
      else if (!rotation_OK) {postFlightCode = 3; radioEvent = 10;}
      else if (!Alt_excd) {postFlightCode = 2; radioEvent = 11;}}

    // Check for sustainer ignition
    if(twoStage && !apogee && !sustainerIgnition && sustainerFire && accelNow > 10.0){radioEvent = 13; sustainerIgnition = true; timeLastEvent = timeCurrent;}
    
    //Check for sustainer burnout
    if(twoStage && !apogee && !sustainerBurnout && sustainerIgnition && accelX < 0 && timeCurrent - timeLastEvent > 100000UL){radioEvent = 14; sustainerBurnout = true; timeLastEvent = timeCurrent;}
    
    //Check for apogee if the accelerometer velocity < 0 or the baroApogee is less than zero
    if (!apogee && boosterBurnout && !boosterBurnoutCheck && !pyroFire && (accelVel < 0 || (baroApogee < 0 && accelVel < 70 && Alt < 9000))) {
      apogee = true;
      timeLastEvent = timeCurrent;}

    //Fire apgogee charge if the current time > apogeeTime + apogeeDelay
    if (!apogeeFire && apogee && timeCurrent - timeLastEvent >= apogeeDelay) {
      apogeeFire = true;
      if(!apogeeSeparation){radioEvent = 5;}
      timeLastEvent = timeCurrent;
      digitalWrite(firePin, LOW);
      firePin = apogeeFpin;
      timeFireBegin = timeCurrent;
      pyroFire = true;
      //Fire apogee charge
      digitalWrite(firePin, HIGH);}
      
    //Write the data to the card 3s after apogeeFire and mainDeploy in case of crash or powerloss
    if(apogeeFire && !syncCard && !testMode && timeCurrent - timeLastEvent >= 3000000UL){outputFile.sync();syncCard = true;}

    //Detect separation after apogee
    if(apogeeFire && !mainDeploy && accelX > 4*g && timeCurrent - timeLastEvent <= 2000000UL){apogeeSeparation = true; radioEvent = 6;}

    //Fire main chute charge if the baro altitude is lower than the threshold and at least 1s has passed since apogee
    if (!mainDeploy && apogee && Alt < mainDeployAlt && timeCurrent - timeLastEvent >= 1000000UL) {
      mainDeploy = true;
      timeLastEvent = timeCurrent;
      radioEvent = 7;
      digitalWrite(firePin, LOW);
      firePin = mainFpin;
      timeFireBegin = timeCurrent;
      pyroFire = true;
      //Fire main charge
      digitalWrite(firePin, HIGH);
      //reset the sync boolean so the card syncs again 3s after main deploy
      syncCard = false;}

    //Detect deployment of the mains
    if(mainDeploy && micros()-timeLastEvent > 50000UL && micros()-timeLastEvent < 3000000UL && accelNow > 50.0){radioEvent = 15;}
      
    //Stop firing after 500 miliseconds
    if (pyroFire && timeCurrent - timeFireBegin > fireTime) {
      digitalWrite(firePin, LOW);
      pyroFire = false;}

    //Check for touchdown
    if (!touchdown && mainDeploy && !pyroFire && !testMode && baroTouchdown > touchdownTrigger && Alt < 46) {
      touchdown = true;
      onGround = true;
      timeLastEvent = micros();
      radioEvent = 8;
      touchdownHour = GPS.time.hour();
      touchdownMin = GPS.time.minute();
      touchdownSec = GPS.time.second();
      touchdownMili = GPS.time.centisecond();}

    //check continuity
    if (digitalRead(apogeeCpin) == HIGH) {contApogee = true;}
    else{contApogee = false;}
    if (digitalRead(mainCpin) == HIGH) {contMain = true;}
    else{contMain = false;}
    if (digitalRead(sepCpin) == HIGH) {contSep = true;}
    else{contSep = false;}
    if (digitalRead(ignCpin) == HIGH) {contStage = true;}
    else{contStage = false;}

    //read the main battery voltage
    if(newBMP){voltage = adc->analogRead(A15);}

    //build the packet of 4 samples: 13 bytes per sample, 12 bytes GPS & pktnum, 13 x 4 + 12 = 64 bytes flight data
    if(radioTXmode && timeCurrent - timeLastSample > sampDelay){
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
      radioAccelNow = (int16_t)(accelNow * 33.41406087); //33.41406087 = 32768 / 9.80665 / 100
      dataPacket[pktPosn] = lowByte(radioAccelNow);pktPosn++;//12
      dataPacket[pktPosn] = highByte(radioAccelNow);pktPosn++;}//13
      
    //GPS Data collected once per packet then transmit
    if(sampNum >= 4){
      packetnum++;
      dataPacket[pktPosn] = lowByte(packetnum); pktPosn++;//53
      dataPacket[pktPosn] = highByte(packetnum); pktPosn++;//54
      if(gpsTransmit){
        gpsTransmit=false;
        radioGPSalt = (int16_t)(GPS.altitude.meters() - baseAlt);
        dataPacket[pktPosn] = lowByte(radioGPSalt);pktPosn++;//55
        dataPacket[pktPosn] = highByte(radioGPSalt);pktPosn++;//56
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
    //writeLongData(timeGyro - timeCurrent);
    //LSM9DS1 Accel Data
    writeIntData(accelX);
    writeIntData(accelY);
    writeIntData(accelZ);
    //LSM9DS1 Gyro Data
    writeIntData(gyroX);
    writeIntData(gyroY);
    writeIntData(gyroZ);
    //Integrated Rotation Values
    writeIntData(rollZ);
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
    dataString[strPosn] = cs;
    strPosn++;
    //Continuity Data
    writeBoolData(contSep);
    writeBoolData(contStage);
    writeBoolData(contApogee);
    writeBoolData(contMain);
    dataString[strPosn] = cs;
    strPosn++;
    //Pyro Firing Data
    writeBoolData(pyroFire);
    dataString[strPosn] = cs;
    strPosn++;
    writeIntData(firePin);
    //ADXL377 Accelerometer Data
    writeIntData(highGxDiff);
    //BMP180 Altitude data
    if (newBMP) {
      writeFloatData(Alt, 2);
      writeFloatData(pressure, 2);
      writeFloatData(temperature, 2);
      writeIntData(voltage);
      newBMP=false;}
    else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    //LSM9DS1 Magnetometer Data
    if (magCounter == 0){
      writeIntData(magX);
      writeIntData(magY);
      writeIntData(magZ);}
    else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    //GPS Data
    if(gpsWrite){
      dataString[strPosn]=gpsLat;strPosn++;
      writeFloatData(gpsLatitude,6);
      dataString[strPosn]=gpsLon;strPosn++;
      writeFloatData(gpsLongitude,6);
      writeFloatData((float)GPS.speed.mph(),2);
      writeFloatData((float)GPS.altitude.meters(),2);
      writeFloatData((float)GPS.course.deg(),2);
      writeIntData((int)GPS.satellites.value());
      gpsWrite=false;}
    else{
      dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;
      dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
    //update the radio packet number
    if (radioTX){writeIntData(packetnum);radioTX = false;writeIntData(radioVel);}
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
      outputFile.println(F("Max Baro Alt,Max GPS Alt,Max Speed,Max Gs,baseAlt,initial Y ang,initial Z ang,accelX0,accelY0,accelZ0,highGx0,highGx0Diff,magX0,magY0,magZ0,gyroBiasX,gyroBiasY,gyroBiasZ,accelBiasX,accelBiasY,accelBiasZ,highGxBias,highGxDiffBias"));
      writeLongData((unsigned long)(maxAltitude*3.2808));
      writeLongData((unsigned long)(maxGPSalt*3.2808));
      writeLongData((unsigned long)(maxVelocity*3.2808));
      writeFloatData(maxG, 2);
      writeFloatData(baseAlt, 2);
      writeFloatData(yawY0, 2);
      writeFloatData(pitchX0, 2);
      writeIntData(accelX0);
      writeIntData(accelY0);
      writeIntData(accelZ0);
      writeIntData(highGx0);
      writeIntData(highGx0Diff);
      writeIntData(magX0);
      writeIntData(magY0);
      writeIntData(magZ0);
      writeIntData(gyroBiasX);
      writeIntData(gyroBiasY);
      writeIntData(gyroBiasZ);
      writeIntData(accelBiasX);
      writeIntData(accelBiasY);
      writeIntData(accelBiasZ);
      writeIntData(highGxBias);
      writeIntData(highGxDiffBias);
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
      dataString[strPosn] = '\r';
      strPosn++;
      dataString[strPosn] = '\n';
      strPosn++;
      dataString[strPosn] = '\0';
      outputFile.write(dataString, strPosn);
      
      //write out the settings for the flight
      strPosn = 0;
      outputFile.println(F("Rocket Name, callsign, magSwitchEnable, TXenable, TXpwr, startupTime, gTrigger, detectLiftoffTime, apogeeDelay, mainDeployAlt, rcdTime, fireTime, pyro4-1, 2Stage, ignitionDelay, sepDelay, altThreshold, maxAng, quatEnable, seaLevelPressure, batteryVoltage"));
      outputFile.print(rocketName);outputFile.print(cs);
      outputFile.print(callSign);outputFile.print(cs);
      writeBoolData(magSwitchEnable);strPosn++;dataString[strPosn] = cs;strPosn++;
      writeBoolData(radioTXmode);strPosn++;dataString[strPosn] = cs;strPosn++;
      writeIntData(TXpwr);
      writeFloatData(TXfreq);
      writeFloatData(setupTime*mlnth,1);
      writeFloatData((float)(gTrigger/g),1);
      writeFloatData(detectLiftoffTime*mlnth,1);
      writeFloatData(apogeeDelay*mlnth,1);
      writeIntData((int)(10*int(mainDeployAlt*.32808)));
      writeFloatData(rcd_time*mlnth,0);
      writeFloatData(fireTime*mlnth,1);
      dataString[strPosn]=pyro4;strPosn++;
      dataString[strPosn]=pyro3;strPosn++;
      dataString[strPosn]=pyro2;strPosn++;
      dataString[strPosn]=pyro1;strPosn++;dataString[strPosn]=cs;strPosn++;
      if(twoStage){writeIntData(1);} else{writeIntData(0);}
      writeFloatData(sustainerFireDelay*mlnth,1);
      writeFloatData(separation_delay*mlnth,1);
      writeIntData((int)(10*int(Alt_threshold*.32808)));
      writeIntData(max_ang/10);
      writeFloatData(seaLevelPressure,2);
      writeFloatData(voltage,2);
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
      parseBeep(long(maxAltitude*3.2808), maxAltDigits, num6);
      //Read max velocity into its beep array
      parseBeep(long(maxVelocity*3.2808), maxVelDigits, num4);
      //reset n, which we'll use to cycle through the reporting digits
      while (maxAltDigits[altDigits-1]==0){altDigits--;}
      while (maxVelDigits[velDigits-1]==0){velDigits--;}  
      n=altDigits;
      reportCode = true;
      //store the maximum altitude in EEPROM
      if(!testMode){for(byte i=0;i<6;i++){EEPROM.update(i,maxAltDigits[i]);}}
    }//end of timeout/touchdown check    
    
  }//end of liftoff flag
  
  //Code to start the beep
  if ((preLiftoff || fileClose) && !beep && timeClock - timeLastBeep > beep_delay)  {
    digitalWrite(beepPin, HIGH);
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
    digitalWrite(beepPin, LOW);
    timeBeepStart = 0UL;
    timeLastBeep = timeClock;
    beep = false;}

  //Telemetry ground code
  if(radioTXmode && onGround && micros() - lastTX > radioDelay){
    if(preLiftoff){//37 byte packet
      pktPosn=0;
      dataPacket[pktPosn]=radioEvent; pktPosn++;
      dataPacket[pktPosn]=gpsFix; pktPosn++;
      if(twoStage){dataPacket[pktPosn]=beepCode; pktPosn++;}
      else{dataPacket[pktPosn]=(beepCode + 5); pktPosn++;}
      for (byte j = 0; j < sizeof(rocketName); j++){
        dataPacket[pktPosn] = rocketName[j];
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
      radioMaxG = (int)(maxG * 327.68);
      radioInt = radioMaxG;
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
  //If we had a GPS fix, but haven't seen one for more than 30s, config defaults
  if(!configGPSdefaults && micros() - timeLastGPS > 10000000UL){
    restoreGPSdefaults();
    configGPSdefaults = true; 
    gpsFix = 0;
    fixCount = 0;
    configGPSflight = false;}
  //5 seconds after touchdown put GPS into Power Save Mode (PSM)
  if( !GPSpsm && (touchdown || timeOut) && micros() - timeLastEvent > 5000000UL){GPSpowerSaveMode();GPSpsm = true;}
  //Read from serial
  while(HWSERIAL.available() > 0){
    char c = HWSERIAL.read();
    GPS.encode(c);
    if(testMode && !liftoff){Serial.print(c);}}
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
          if(GPS.altitude.meters() > 0){
            //Correct sea level pressure with running average of 5 samples
            //GPS altitude running average
            GPSposn++;
            if(GPSposn > 4){GPSposn = 0;}
            GPSaltSum = GPSaltSum + GPS.altitude.meters() - GPSavgAlt[GPSposn];
            GPSavgAlt[GPSposn] = GPS.altitude.meters();
            baseGPSalt = GPSaltSum*0.2;
            //barometric pressure running average
            pressurePosn++;
            if(pressurePosn > 4){pressurePosn = 0;}
            pressureSum = pressureSum + pressure - pressureAvg5[pressurePosn];
            pressureAvg5[pressurePosn] = pressure;
            pressureAvg = pressureSum*0.2;
            //sea level correction
            seaLevelPressure = pressureAvg / pow((44330 - baseGPSalt)/44330, 5.254861);
            //correct baseAlt
            if(GPSavgAlt[4] != 0){
              baseAlt = pressureToAltitude(seaLevelPressure, pressureAvg);
              //if(testMode){Serial.print("GPS: "); Serial.print(GPS.altitude.meters()); Serial.print(", BMP: "); Serial.println(baseAlt);}
              if(isnan(baseAlt)){baseAlt = pressureToAltitude(1013.25, pressureAvg); seaLevelPressure = 1013.25;}}}
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
  if (dataBool) {dataString[strPosn] = '1';}
  else {dataString[strPosn] = '0';}
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

  void configPyro(char pyro, byte pyroCont, byte pyroFire){

  switch(pyro){

    case 'M':
      mainCpin = pyroCont;
      mainFpin = pyroFire;
      break;

    case 'A':
      apogeeCpin = pyroCont;
      apogeeFpin = pyroFire;
      break;

    case 'I':
      ignCpin = pyroCont;
      ignFpin = pyroFire;
      break;

    case 'B':
      sepCpin = pyroCont;
      sepFpin = pyroFire;
      break;

    case 'N':
      //default is set to unconnected pin
      break;
      
    case 'S':
      airStartCpin = pyroCont;
      airStartFpin = pyroFire;
      break;
}}

void writeCalibration(int16_t inValue, byte eepromStart){

    calUnion.calValue = inValue;
    EEPROM.write(eepromStart, calUnion.calByte[0]);
    EEPROM.write(eepromStart + 1, calUnion.calByte[1]);

}

