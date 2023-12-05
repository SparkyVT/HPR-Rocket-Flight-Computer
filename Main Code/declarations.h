//Header file for all major declared variables
//-----------------------------------------------
//Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
//29 Nov 23: Broke out a separate header file to make reading the main file easier
//-----------------------------------------
//EEPROM Locations
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
    int gpsLogFile = 187;
    int fireTime = 188;//4
    int pyro4Func = 192;
    int pyro3Func = 193;
    int pyro2Func = 194;
    int pyro1Func = 195;
    int TXenable = 196;
    int TXpwr = 197;
    int TXfreq = 198;//4
    int FHSS = 202;
    int boosterSeparationDelay = 203;//4
    int sustainerFireDelay = 207;//4
    int airStart1Event = 211;
    int airStart1Delay = 212;//4
    int airStart2Event = 216;
    int airStart2Delay = 217;//2
    int altThreshold = 219;//2
    int maxAngle = 221;
    int stableRotn = 222;
    int stableVert = 223;
    int flyback = 224;
    int serialDebug = 225;
    int lastFile = 226;//2
    int baseAlt = 228;//4
    int lastEvent = 232;
    //-------------------------------------------------------------
    //active stabilization parameters
    //-------------------------------------------------------------
    int servo1trim = 233;
    int servo2trim = 234;
    int servo3trim = 235;
    int servo4trim = 236;
    int servo5trim = 237;
    int servo6trim = 238;
    int servo7trim = 239;
    int servo8trim = 240;
    int Kp = 241;//4
    int Ki = 245;//4
    int Kd = 249;//4
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
  byte GNSS = 0;
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
  char gnssBusType;
  byte gnssBusNum;
  char sdBusType;
  byte sdBusNum;
  bool statusAccel = false;
  bool statusGyro = false;
  bool statusHighG = false;
  bool statusBaro = false;
  bool statusMag = false;
  bool statusRadio = false;
  bool pyroPWM = false;
} sensors;
//-----------------------------------------
//User Settings
//-----------------------------------------
struct {
  bool testMode = false;
  bool calibrationMode = false;
  //--------flight settings----------------
  char rocketName[20] = ""; //Maximum of 20 characters
  char HWid[7] = "";
  char fltProfile = 'S';
  char units = 'S';
  char reportStyle = 'P';
  bool GPSfile = false;
  unsigned long setupTime = 5000000UL;
  float mainDeployAlt = 153;//Up to 458m for main deploy
  unsigned long rcdTime = 900000000UL; //15min
  unsigned long apogeeDelay = 1000000UL; //1.0s apogee delay
  bool silentMode = false; //true turns off beeper
  bool magSwitchEnable = false;
  byte inflightRecover = 0;
  bool GPSlog = false;
  byte serialDebug = 1;
  //--------pyro settings----------------
  unsigned long fireTime = 500000UL;//0.5s
  char pyro4Func = 'M';
  char pyro3Func = 'A';
  char pyro2Func = 'N';
  char pyro1Func = 'N';
  //--------telemetry settings----------------
  char callSign[7]= ""; 
  bool TXenable = true;//false turns off radio transmissions
  byte TXpwr = 13;
  float TXfreq = 433.250;
  bool FHSS = false;
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
  bool stableRotn = false;
  bool stableVert = false;  
  //----------flyback option--------------------
  bool flyBack = false;
} settings;
//-----------------------------------------
//Sensor variables
//-----------------------------------------
byte rawData[14];
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
//pyro variables
//-----------------------------------------
typedef struct{
  char func;
  bool fireStatus = false;
  bool contStatus = false;
  unsigned long fireStart;
  uint8_t contPin;
  uint8_t firePin;
} pyroMap;
pyroMap pyro1;
pyroMap pyro2;
pyroMap pyro3;
pyroMap pyro4;
//-----------------------------------------
//flight events
//-----------------------------------------
struct  eventList{
  bool preLiftoff = true;
  bool inFlight = false;
  bool postFlight = false;
  bool liftoff = false;
  bool falseLiftoffCheck = true;
  bool boosterBurnout = false;
  bool boosterBurnoutCheck = false;
  bool boosterSeparation = false;
  bool sustainerFireCheck = false;
  bool sustainerFire = false;
  bool sustainerIgnition = false;
  bool sustainerBurnout = false;
  bool airStart1Check = false;
  bool airStart1Fire = false;
  bool airStart1Ignition = false;
  bool airStart1BurnoutCheck = false;
  bool airStart1Burnout = false;
  bool airStart2Check = false;
  bool airStart2Fire = false;
  bool airStart2Ignition = false;
  bool airStart2BurnoutCheck = false;
  bool airStart2Burnout = false;
  bool apogee = false;
  bool apogeeFire = false;
  bool apogeeSeparation = false;
  bool mainDeploy = false;
  bool touchdown = false;
  bool timeOut = false;
  };
struct eventList events;
struct eventList resetEvents;
//-----------------------------------------
//Master timing variables
//-----------------------------------------
struct timerList{
  unsigned long liftoff = 0UL;
  unsigned long detectLiftoffTime = 500000UL; //0.5s
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
};
struct timerList fltTime;
struct timerList resetFltTime;
//-----------------------------------------
//GPIO pin mapping
//-----------------------------------------
struct{
  uint8_t nullCont;
  uint8_t nullFire;
  uint8_t beep;
  uint8_t testGnd;
  uint8_t testRead;
  uint8_t mag;
  uint8_t batt;
  uint8_t pyro1Cont;
  uint8_t pyro1Fire;
  uint8_t pyro2Cont;
  uint8_t pyro2Fire;
  uint8_t pyro3Cont;
  uint8_t pyro3Fire;
  uint8_t pyro4Cont;
  uint8_t pyro4Fire;
  uint8_t radioCS;
  uint8_t radioIRQ;
  uint8_t radioRST;
  uint8_t radioEN;
  uint8_t servo1;
  uint8_t servo2;
  uint8_t servo3;
  uint8_t servo4;
  uint8_t servo5;
  uint8_t servo6;
  uint8_t servo7;
  uint8_t servo8;
  uint8_t accelCS;
  uint8_t gyroCS;
  uint8_t magCS;
  uint8_t highG_CS;
  uint8_t baroCS;
  uint8_t SD_CS;
  uint8_t firePin = 0;
} pins;
//-----------------------------------------
//radio variables
//-----------------------------------------
uint8_t dataPacket[256];
bool liftoffSync = false;
unsigned long TXstartTime;
bool syncFreq = false;
uint8_t pktPosn = 0;
uint8_t pktSize = 0;
uint16_t sampNum = 0;
uint8_t packetSamples = 4;
bool gpsTransmit = false;
bool TX = false;
bool SDradioTX = false;
uint8_t radioMode;
volatile boolean clearTX = false;
volatile boolean buildPkt = false;
volatile boolean syncFlag = false;
bool sendPkt = false;
struct {
  uint32_t preLiftoff = 1000000UL;
  uint32_t inflight = 200000UL;
  uint32_t postFlight = 10000000UL; 
  uint32_t samplesPerPkt = 4UL;
} pktInterval;
struct{
  int16_t packetnum = 0;
  uint8_t event = 0;
  uint16_t fltTime = 0;
  int16_t baseAlt = 0;
  int16_t alt = 0;
  int16_t accel = 0;
  int16_t vel = 0;
  int16_t roll = 0;
  int16_t offVert = 0;
  int16_t maxAlt = 0;
  int16_t maxVel = 0;
  int16_t maxGPSalt = 0;
  int16_t maxG = 0;
  int16_t GPSalt = 0;
  int16_t satNum = 0;
  bool pktCallsign = false;  
} radio;
//-----------------------------------------
//continuity Booleans
//-----------------------------------------
struct{
  bool apogee;
  bool main;
  bool boosterSep;
  bool upperStage;
  bool airStart1;
  bool airStart2;
  bool noFunc;
  bool error;
  uint8_t reportCode;
  uint8_t beepCode;
  } cont;
//-----------------------------------------
//Non-Event Booleans
//-----------------------------------------
bool rotnOK = true;
bool altOK = false;
bool beep = false;
bool pyroFire = false;
bool fileClose = false;
unsigned long boosterBurpTime = 1000000UL;
float unitConvert = 3.2808F;
//-----------------------------------------
//digital accelerometer variables
//-----------------------------------------
int g = 1366;
float accelNow;
float maxG = 0.0;
int gTrigger = 3415; //2.5G trigger
//-----------------------------------------
//High-G accelerometer variables
//-----------------------------------------
int high1G = 63;
bool filterFull = false;
long highGsum = 0L;
int highGfilter[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t sizeHighGfilter = 30;
uint8_t filterPosn = 0;
float highGsmooth;
float adcConvert = 0.000015259;
uint16_t ADCmidValue = 32768;
union {
   int16_t calValue; 
   uint8_t calByte[2];
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
  bool newSamp = false;
  bool newTemp = false;
  float seaLevelPressure = 1013.25;
  float tempOffset;
  float pressOffset;
} baro; 
//-----------------------------------------
//Baro Reporting Variables
//-----------------------------------------
uint8_t baroTouchdown = 0;
uint8_t touchdownTrigger = 5;
float pressureAvg = 0;
float pressureAvg5[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float pressureSum = 0.0;
uint8_t pressurePosn = 0;
//-----------------------------------------
//Magnetometer Variables
//-----------------------------------------
int16_t magTrigger = 0;
uint8_t magCalibrateMode = 0;
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
const float mlnth = 0.000001;
int offVert = 0;
bool rotationFault = false;
//-----------------------------------------
//Rotation timing variables
//-----------------------------------------
unsigned long lastRotn = 0UL;//time of the last call to update the rotation
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
uint32_t clearRailTime = 250000UL;
//-----------------------------------------
//beeper variables
//-----------------------------------------
uint8_t beep_counter = 0;
uint8_t beepPosn = 0;
unsigned long beep_delay = 100000UL;
int beepCode = 0;
unsigned long beep_len = 100000UL;
unsigned long timeBeepStart;
unsigned long timeLastBeep;
bool beepAlt = false;
bool beepVel = false;
bool beepCont = true;
bool beepAlarm = false;
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
bool syncApogee = false;
bool syncMains = false;
const byte decPts = 2;
const byte base = 10;
char dataString[1024];
uint8_t maxAltDigits[6];
uint8_t maxVelDigits[4];
uint8_t voltageDigits[2];
uint8_t altDigits = 6;
uint8_t velDigits = 4;
uint8_t n = 1;
float voltage = 0.0F;
bool writeVolt = false;
unsigned long voltRate = 33333UL;
uint16_t voltReading;
unsigned long lastVolt = 0UL;
bool reportCode = true;//true = report max altitude, false = report max velocity
uint8_t postFlightCode = 0;
const char cs = ',';
//-----------------------------------------
//GNSS Variables
//-----------------------------------------
struct eventData{
  float latitude;
  float longitude;
  float alt = 0.0;
  int year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t mili;
};
struct {
  float alt = 0.0;
  float maxAlt = 0.0;
  float baseAlt = 0.0;
  float vel = 0.0F;
  float latitude;
  float longitude;
  float avgAlt[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float altBuff[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  uint32_t timeBuff[5] = {0UL, 0UL, 0UL, 0UL, 0UL};
  bool bufferFull = false;
  float altSum = 0.0;
  uint8_t bufferPosn = 0;
  uint8_t altPosn = 0;
  eventData liftoff;
  eventData apogee;
  eventData touchdown;
  uint8_t fix = 0;
  uint16_t fixCount = 0;
  unsigned long timeLastFix= 0UL;
  bool SDwrite = false;
  bool configDefaults = false;
  bool configFlight = false;
  bool configPwrSave = false;
} gnss;
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
boolean GPSecho = false;
boolean radioDebug = false;
boolean TXdataFile = true;
//--------------------------------------------
//Radio event codes to eliminate magic numbers
//--------------------------------------------
enum flightCodes{
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
enum radioCodes{
  SleepMode     =  0x00,
  StandbyMode   =  0x01,
  TXmode        =  0x03,
  RXmode        =  0x05,
  CADmode       =  0x07,
  LoRaMode      =  0x80, 
  LowFrqMode    =  0x08, //LowFrq mode enables registers below 860MHz
  writeMask     =  0x80};
uint8_t radioFnctn = SleepMode;