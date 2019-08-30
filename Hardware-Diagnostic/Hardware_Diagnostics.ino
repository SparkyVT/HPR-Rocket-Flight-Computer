  //-------CODE START--------
  #include <SdFat.h>
  #include <i2c_t3.h>
  #include <SPI.h>
  #include <EEPROM.h>
  #include <RH_RF95.h>
  #include <TinyGPS++.h>

  //Radio setup
  #define RF95_FREQ     433.250
  #define RFM95_RST     20
  #define RFM95_CS      19
  #define RFM95_IRQ     2
  #define RFM95_EN      22
  RH_RF95 rf95(RFM95_CS, RFM95_IRQ);

  //Teensy 3.5 Hardware Serial for GPS
  HardwareSerial HWSERIAL(Serial1);

  // GPS Setup
  TinyGPSPlus GPS;

  //SDIO Setup
  SdFatSdioEX SD;
  File outputFile;
  File settingsFile;

boolean testMode = true;
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
int highGxBias = 0;
int highGyBias = 0;
int highGzBias = 0;
const float convertG = 66.93989; //66.93989 = HighGgain / digitalGain = 0.049/0.000732
int32_t accelX0sum = 0L;
int32_t accelY0sum = 0L;
int32_t accelZ0sum = 0L;
int accelX0 = 0;
int accelY0 = 0;
int accelZ0 = 0;
int accelX;
int accelY;
int accelZ;
float accelNow;
float maxG = 0.0;
byte accelAddress;
byte accelRegister;
//-----------------------------------------
//High-G accelerometer variables
//-----------------------------------------
int highG = 20;
long highGsum = 0L;
int highGfilter[10] = {0,0,0,0,0,0,0,0,0,0};
int16_t highGx;
int16_t highGy;
int16_t highGz;
int16_t highGx0 = 0;
int16_t highGy0 = 0;
int16_t highGz0 = 0;
byte filterPosn = 0;
float filterAccel;
long highGsumX0 = 0L;
long highGsumY0 = 0L;
long highGsumZ0 = 0L;
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
byte baroSensor = 1;
//-----------------------------------------
//Magnetometer Variables
//-----------------------------------------
unsigned long magTrigger = 100000UL;
unsigned long magCounter = 0UL;
int magX0 = 0;
int magY0 = 0;
int magZ0 = 0;
long magX0sum = 0L;
long magY0sum = 0L;
long magZ0sum = 0L;
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

void setup() {


  //Start communication
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_7_8, I2C_PULLUP_EXT, 400000);
  SPI.begin();
  HWSERIAL.begin(9600);

  //Start sensors and SD card
  SD.begin();
  boolean status_LSM9DS1 = beginLSM9DS1();
  boolean status_MPL3115A2 = beginMPL3115A2();
  boolean status_H3LIS331DL = beginH3LIS331DL(2);  
  boolean status_BMP280 = beginBMP280();
  
  //Enable the radio
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  
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
  rf95.setFrequency(RF95_FREQ);
  radioDelay = RDpreLiftoff;

  if(!status_LSM9DS1){Serial.println(F("LSM9DS1 not detected!"));}
  if(status_LSM9DS1){Serial.println(F("LSM9DS1 OK!"));}
  if(!status_BMP280){Serial.println(F("BMP280 not detected!"));}
  if(status_BMP280){Serial.println(F("BMP280 OK!"));}
  if(!status_MPL3115A2){Serial.println(F(" MPL3115A2 not detected!"));}
  if(status_MPL3115A2){Serial.println(F("MPL3115A2 OK!"));baroSensor = 2;}
  if(!status_H3LIS331DL){Serial.println(F("H3LIS331DL not detected!"));}
  if(status_H3LIS331DL){Serial.println(F("H3LIS331DL OK!"));}
  if(!status_RFM96W){Serial.println(F("RFM96W not detected!"));}
  if(status_RFM96W){Serial.println(F("RFM96W OK!"));}
}

void loop() {
  getGyro();
  getAccel();
  getHighG();
  getMag();
  if(baroSensor == 1){bmpGetReading();}
  if(baroSensor == 2){readMPLbaro();}

  //Read GPS from serial
  while(HWSERIAL.available() > 0){
    char c = HWSERIAL.read();
    GPS.encode(c);
    Serial.print(c);}
  Serial.println(" ");

  Serial.print("Accel Method 1: ");Serial.print(accelX);Serial.print(",");Serial.print(accelY);Serial.print(",");Serial.println(accelZ);
  Serial.print("Gyro Method 1: ");Serial.print(gyroX);Serial.print(",");Serial.print(gyroY);Serial.print(",");Serial.println(gyroZ);
  getAccelGyro();
  Serial.print("Accel Method 2: ");Serial.print(accelX);Serial.print(",");Serial.print(accelY);Serial.print(",");Serial.println(accelZ);
  Serial.print("Gyro Method 2: ");Serial.print(gyroX);Serial.print(",");Serial.print(gyroY);Serial.print(",");Serial.println(gyroZ);
  Serial.print("Mag: ");Serial.print(magX);Serial.print(",");Serial.print(magY);Serial.print(",");Serial.println(magZ);
  Serial.print("HighG: ");Serial.print(highGx);Serial.print(",");Serial.print(highGy);Serial.print(",");Serial.println(highGz);
  Serial.print("Baro: ");Serial.print(temperature);Serial.print(",");Serial.print(pressure);Serial.print(",");Serial.println(Alt);
  rf95.setModeIdle();rf95.send((uint8_t *)dataPacket, 80); Serial.println("Data Packet Sent");
  
  delay(1000);
}
