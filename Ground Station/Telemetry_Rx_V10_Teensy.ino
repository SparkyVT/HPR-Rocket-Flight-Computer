//V7 uses only a single microprocessor
//V8 uses an optimized data stream
//V9 overhauls the whole system
//V10 eliminates the RadioHead library

//TO DO:
//0) Read user data from SD card
//1) Create BLE Service
//2) Pin correction for LCD - OK
//3) Radio 1 controls - OK
//4) Radio 2 controls - OK
//5) GPS Services - OK
//6) LSM9DS1 orientation
//7) LSM9DS1 magnetic
//8) Include Software SPI - OK
//9) Create user menu interface
//10) Delineate FHSS with a booster

#include <SdFat.h>
#include <SerLCD.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>

//Hardware Serial for GPS
HardwareSerial *HWSERIAL;

//Radio control pins
#define radio1RST     14//ok
#define radio1CS      15//ok
#define radio1IRQ     17//ok
#define radio2RST     16//ok
#define radio2CS      20//ok
#define radio2IRQ     22//ok

//void preflightPacket(byte dataPacket[]);
//void inflightPacket(byte dataPacket[]);
//void postflightPacket(byte dataPacket[]);

char serialBuffer[256];
int serialPosn = 0;

byte dataPkt[80];
volatile boolean processIRQ = false;
uint32_t timeLastPkt = 0UL;
uint32_t debugTime;
uint8_t pktRssi;
float radio1Freq = 433.500;
float radio2Freq = 433.250;

//RegOpMode
enum {
  SleepMode     =  0x00,
  StandbyMode   =  0x01,
  TXmode        =  0x03,
  RXmode        =  0x05,
  CADmode       =  0x07,
  LoRaMode      =  0x80, 
  LowFrqMode    =  0x08, //LowFrq mode enables registers below 860MHz
  writeMask     =  0x80};

byte radio1Fnctn = RXmode;
byte radio2Fnctn = RXmode;

const uint8_t SOFT_MISO_PIN  = 2;
const uint8_t SOFT_MOSI_PIN  = 1;
const uint8_t SOFT_SCK_PIN   = 3;
const uint8_t SD_CS_PIN      = 0; 

// SdFat software SPI template
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)

SdFs SD;
FsFile sustainerFile;
FsFile boosterFile;

//LCD Setup
SerLCD lcd;

// GPS Setup
TinyGPSPlus GPS;

//---------------------------
//Code Control Option variables
//---------------------------
boolean displayStandard = true;
boolean FHSS = false;
boolean debugSerial = true;
float unitConvert = 3.28084F;
boolean LCD = true;
boolean GPSdebug = false;
//---------------------------
//Data Packet variables
//---------------------------
uint8_t len;
uint8_t len1;
uint8_t len2;
byte dataPacket1[66];
byte dataPacket2[66];
char dataString[256];
boolean SDstatus = false;
char sustainerFileName[13] = "FLIGHT01.txt";
char boosterFileName[14] = "BOOSTER01.txt";
boolean sustainerFileOpen = false;
boolean boosterFileOpen = false;
boolean sustainerFileCreated = false;
boolean boosterFileCreated = false;
byte n = 0;
byte fileNum = 1;
byte pktPosn = 0;
int16_t packetnum = 0;
unsigned long timer = 0UL;
boolean radio1status = false;
boolean radio2status = false;
boolean sendPacket = true;
boolean testMode = false;
unsigned long testStart = 0UL;
unsigned long lastRX = 0UL;
unsigned long debugStart = 0UL;
unsigned long debugStop = 0UL;
unsigned long colorStart;
int battVolt;
boolean SDinit = false;
boolean LCDinit = true;
boolean parseSustainer = false;
boolean parseBooster = false;
uint32_t timeLastNMEA = 0UL;
bool ledLight = false;
//---------------------------
//preflight output variables
//---------------------------
uint8_t event;
int strPosn = 0;
boolean signalEst = false;
boolean dataProcessed = false;
boolean sustainerPreFlightWrite = true;
boolean boosterPreFlightWrite = true;
boolean sustainerPostFlightWrite = true;
boolean boosterPostFlightWrite = true;
boolean boosterFileReady = false;
unsigned long lostSignalTime = 2000000UL;
byte j=0;
//---------------------------
//preflight output variables
//---------------------------
char rocketName[20]="";
int baseAlt=0;
int baseGPSalt = 0;
byte contCode;
int satNum = 0;
boolean fileOpen = true;
//---------------------------
//inflight output variables
//---------------------------
uint16_t sampleTime;
int16_t signalStrength=0;
int16_t velocity = 0;
int16_t Alt=0;
int16_t spin=0;
int16_t offVert=0;
int16_t accel=0;
int16_t packetNum = 0;
boolean apogee = false;
unsigned long lastSustainerRX = 0UL;
unsigned long lastBoosterRX = 0UL;
//---------------------------
//Postflight output variables
//---------------------------
int maxAltitude=0;
int maxVelocity=0;
int maxG=0;
int maxGPSalt=0;
//GPS output variables
byte GPSlock;
int GPSalt=0;
int prevGPSalt;
char charGPSlat;
float GPSlatitude;
char charGPSlon;
float GPSlongitude;
float lastGPSlat;
float lastGPSlon;
float prevGPSlat = 0.0;
float prevGPSlon = 0.0;
unsigned long lastGPSfix = 0UL;
//---------------------------
//Radio Variables
//---------------------------
volatile boolean radio1Pkt = false;
volatile boolean radio2Pkt = false;
union {
   float GPScoord;
   unsigned long radioTime;
   byte unionByte[4];
} radioUnion;
union {
   int16_t unionInt;
   byte unionByte[2]; 
} radioInt;
byte chnl1 = 0;
byte chnl2 = 0;
byte hailChnl_1 = 0;
byte hailChnl_2 = 0;
byte nextChnl;
byte nextChnl2;
boolean syncFreq = false;
float freq1;
float freq2;
unsigned long lastHopTime = 0UL;
byte chnlUsed = 0;
boolean band433 = false;
byte radioFnctn;
//---------------------------
//LCD Variables
//---------------------------
byte whiteIntensity = 125;
byte redIntensity = 255;
byte greenIntensity = 255;
byte blueIntensity = 255;
byte flightPhase = 0;
//---------------------------
//BlueTooth Variables
//---------------------------
boolean blueTooth = true;
//---------------------------
//Event Table
//---------------------------
const String event_00 = "Preflight";
const String event_01 = "Liftoff";
const String event_02 = "Booster Burnout";
const String event_03 = "Apogee Detected";
const String event_04 = "Firing Apogee Pyro";
const String event_05 = "Separation Detected!";
const String event_06 = "Firing Mains";
const String event_07 = "Under Chute";
const String event_08 = "Ejecting Booster";
const String event_09 = "Firing 2nd Stage";
const String event_10 = "2nd Stage Ignition";
const String event_11 = "2nd Stage Burnout";
const String event_12 = "Firing Airstart1";
const String event_13 = "Airstart1 Ignition";
const String event_14 = "Airstart1 Burnout";
const String event_15 = "Firing Airstart2";
const String event_16 = "Airstart2 Ignition";
const String event_17 = "Airstart2 Burnout";
const String event_18 = "NoFire: Rotn Limit";
const String event_19 = "NoFire: Alt Limit";
const String event_20 = "NoFire: Rotn/Alt Lmt";
const String event_21 = "Booster Apogee";
const String event_22 = "Booster Apogee Fire";
const String event_23 = "Booster Separation!";
const String event_24 = "Booster Main Deploy";
const String event_25 = "Booster Under Chute";
const String event_26 = "Time Limit Exceeded";
const String event_27 = "Touchdown!";
const String event_28 = "Power Loss! Restart!";
const String event_29 = "Booster Touchdown!";
const String event_30 = "Booster Preflight";
const String event_31 = "Booster Time Limit";
const String event_32 = "Booster Pwr Restart";

const String  eventTable[] = {
  event_00, event_01, event_02, event_03, event_04,
  event_05, event_06, event_07, event_08, event_09,
  event_10, event_11, event_12, event_13, event_14,
  event_15, event_16, event_17, event_18, event_19,
  event_20, event_21, event_22, event_23, event_24,
  event_25, event_26, event_27, event_28, event_29,
  event_30, event_31, event_32};

byte greenEvents[] = {5,6,7};
byte redEvents[] = {18, 19, 20, 26, 28};

//---------------------------
//Pyro Code Table
//---------------------------
const String cont_0 = "No Pyros Detected!";
const String cont_1 = "No Continuity Pyro 1";
const String cont_2 = "No Continuity Pyro 2";
const String cont_3 = "No Continuity Pyro 3";
const String cont_4 = "No Continuity Pyro 4";
const String cont_5 = "All 3 Pyros Detected";
const String cont_6 = "All 4 Pyros Detected";
const String cont_7 = "Pyro Apogee Only";
const String cont_8 = "Pyro Mains Only";
const String cont_9 = "Pyro Mains & Apogee";

const String pyroTable[] = {
  cont_0, cont_1, cont_2, cont_3, cont_4, 
  cont_5, cont_6, cont_7, cont_8, cont_9};
  
byte greenCont[5] = {5, 6, 7, 8, 9};
byte redCont[5]   = {0, 1, 2, 3, 4};

//---------------------------
//433 MHz Frequency List
//---------------------------
float radioFreq[9] = {433.250, 433.500, 433.625, 433.750, 433.875, 434.000, 434.125, 434.250, 434.400};

//---------------------------
//915 MHz Frequency List
//---------------------------
byte FHSSchnl[64] = {
  54,   48,   58,   27,   63,   43,   41,    1,   9,  20,   15,   37,   35,   55,   39,   31,
  38,   57,   11,   42,   24,   30,   28,   23,   0,  53,   45,   50,   10,   14,   18,   26, 
  13,    8,   56,   17,   40,   49,   61,   21,   6,   5,   44,   52,   51,   59,   33,    7, 
  32,   25,    4,    3,   36,   12,   22,   46,   62, 47,    2,   60,   29,   16,   19,   34};

byte radioCS;

void setup() {

  //start SPI
  SPI.begin();
    
  //Primary Serial Port
  Serial.begin(9600);
  Serial2.begin(9600);
  
  //Read the battery voltage
  pinMode(A0, INPUT);
  battVolt = 0;
  for(byte i = 0; i < 5; i++){battVolt += analogRead(A10);delay(10);}
  battVolt/=5;

  //Set interrupts
  pinMode(radio1IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(radio1IRQ), clearIRQ1, RISING);
  pinMode(radio2IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(radio2IRQ), clearIRQ2, RISING);
  
  //Radio1 setup
  delay(500);
  radio1status = radioBegin(radio1CS, radio1RST);
  if(!radio1status){Serial.println(F("Radio1 Failed"));}
  else{Serial.println(F("Radio1 OK!"));}
  
  //Radio2 setup
  delay(50);
  radio2status = radioBegin(radio2CS, radio2RST);
  if(!radio2status){Serial.println(F("Radio2 Failed"));}
  else{Serial.println(F("Radio2 OK!"));}

  if(!radio1status){radio1status = radioBegin(radio1CS, radio1RST);}

  //read the band from EEPROM
  if((byte)EEPROM.read(63) == 1){band433 = true;}
  
  //read the channel from EEPROM
  if((byte)EEPROM.read(30) == 255){EEPROM.write(30, 0);}
  chnl1 = (byte)EEPROM.read(30);
  if((byte)EEPROM.read(61) == 255){EEPROM.write(30, 0);}
  chnl2 = (byte)EEPROM.read(61); 
  
  //set the frequency
  //freq1 = getFreq(chnl1);
  //freq2 = getFreq(chnl2);
  radioCS = radio1CS;
  if(setRadioFreq(radio1Freq)){Serial.print("Radio1 on ");Serial.print(radio1Freq, 3);Serial.println(" MHz");}
  else{Serial.println("Radio1 Set Frequency Failed");}
  radioSetMode(RXmode);
  radioCS = radio2CS;
  if(setRadioFreq(radio2Freq)){Serial.print("Radio2 on ");Serial.print(radio2Freq, 3);Serial.println(" MHz");}
  else{Serial.println("Radio2 Set Frequency Failed");}
  radioSetMode(RXmode);
  
  //Set the flag and interrupts if FHSS is used
  if((byte)EEPROM.read(63)==1){
    FHSS = true;
    hailChnl_1 = chnl1;
    hailChnl_2 = chnl2;}
    
  // Initialise the LCD and SD card
  SDinit = SD.begin(SD_CONFIG);
  if(!SDinit){Serial.println(F("SD card failed"));}
  else{Serial.println(F("SD Card OK!"));}
      
  /* EEPROM Allocations:
  Posn 00-19: Sustainer Name
  Posn 20-23: Latitude Float
  Posn    24: Latitude Char
  Posn 25-28: Longitude Float
  Posn    29: Longitude Char
  Posn    30: Radio1 Freq
  Posn 31-50: Booster Name
  Posn 51-54: Latitude Float
  Posn    55: Latitude Char
  Posn 56-59: Longitude Float
  Posn    60: Longitude Char
  Posn    61: Radio2 Freq
  Posn    62: Units
  Posn    63: FHSS
  Posn    64: Radio Band
  Posn    65: BlueTooth Enable
  */

  //Read EEPROM for the sustainer last GPS coordinates
  for(byte i = 0; i < sizeof(rocketName); i++){rocketName[i] = EEPROM.read(i);}
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(20+i);}
  lastGPSlat = radioUnion.GPScoord;
  charGPSlat = EEPROM.read(24);
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(25+i);}
  lastGPSlon = radioUnion.GPScoord;
  charGPSlon = EEPROM.read(29);

  //Set display units
  if((byte)EEPROM.read(62)==1){unitConvert = 3.28084;}

  //Set enabling of BlueTooth
  if((byte)EEPROM.read(65)==1){blueTooth = true;}

  //configure GPS
  configGPS();
  
  //Display the startup screen
  Serial.println("starting LCD");
  startupLCD();
  
  len1 = sizeof(dataPacket1);
  len2 = sizeof(dataPacket2);}//end setup

void loop() {

  boolean msgRX = false;
  boolean pktRX = false;

  //Serial Print Debug
  if(micros()-timer > 1000000UL && micros() - timeLastNMEA > 500000){
    Serial.println(F("Waiting for Packet"));
    timer = micros();}

  //check radio1
  if(radio1Pkt){

    lastRX = micros();

    //get the packet from the radio
    len = len1;
    radioCS = radio1CS;
    radioRecvPkt(dataPacket1);

    //get signal strength
    signalStrength = pktRssi;
    
    //set flags
    event = (byte)dataPacket1[0];
    if(debugSerial){
      Serial.print("Packet on radio1: ");Serial.println(event);
      Serial.print("Length: ");Serial.println(len);}
    //determine if it is a sustainer or booster packet
    if(event < 21){parseSustainer = true;}
    else if(event == 26 || event == 27 || event == 28){parseSustainer = true;}
    else{parseBooster = true;}


    //Create the SD file
    if(parseSustainer && !sustainerFileCreated && SDinit){createSustainerFile();}
    if(parseBooster   && !boosterFileCreated   && SDinit){createBoosterFile();}

    //preflight packet
    if(event == 0 || event == 30){
      //parse the preFlight packet
      preflightPacket(dataPacket1);
      //signal established
      signalEst = true;
      //set lost signal time
      lostSignalTime = 6000000UL;
      //LCD
      preflightLCD();}

    //inflight packet
    else if(event < 26 && event != 28){
      //set lost signal time
      lostSignalTime = 2000000UL;
      
      //write the last pre-flight packet to the SD card
      if (SDinit && sustainerPreFlightWrite && parseSustainer){writePreflightData();}
      if (SDinit && boosterPreFlightWrite   && parseBooster){writePreflightData();}
      
      //parse the packet
      inflightPacket(dataPacket1);
      
      //display to LCD
      inflightLCD();}

    //postflight packet
    else if(event == 26 || event == 27 || event == 29 || event == 31){
      //set lost signal time
      lostSignalTime = 11000000UL;
      //parse the packet
      postflightPacket(dataPacket1);
      //display to LCD
      postflightLCD();}

    //error packet
    else{errorLCD();}

    parseSustainer = false;
    parseBooster = false;
    radio1Pkt = false;
    }//end check Radio1

  //check radio2
  if(radio2Pkt){

    lastRX = micros();

    //get the packet from the radio
    len = len2;
    radioCS = radio2CS;
    radioRecvPkt(dataPacket2);

    //get signal strength
    signalStrength = pktRssi;
    
    //set flags
    event = (byte)dataPacket2[0];
    if(debugSerial){
      Serial.print("Packet on radio2: ");Serial.println(event);
      Serial.print("Length: ");Serial.println(len);}
    //determine if it is a sustainer or booster packet
    if(event < 21){parseSustainer = true;}
    else if(event == 26 || event == 27 || event == 28){parseSustainer = true;}
    else{parseBooster = true;}


    //Create the SD file
    if(parseSustainer && !sustainerFileCreated && SDinit){createSustainerFile();}
    if(parseBooster   && !boosterFileCreated   && SDinit){createBoosterFile();}

    //preflight packet
    if(event == 0 || event == 30){
      //parse the preFlight packet
      preflightPacket(dataPacket2);
      //signal established
      signalEst = true;
      //set lost signal time
      lostSignalTime = 6000000UL;
      //LCD
      preflightLCD();}

    //inflight packet
    else if(event < 26 && event != 28){
      //set lost signal time
      lostSignalTime = 2000000UL;
      
      //write the last pre-flight packet to the SD card
      if (SDinit && sustainerPreFlightWrite && parseSustainer){writePreflightData();}
      if (SDinit && boosterPreFlightWrite   && parseBooster){writePreflightData();}
      
      //parse the packet
      inflightPacket(dataPacket2);
      
      //display to LCD
      inflightLCD();}

    //postflight packet
    else if(event == 26 || event == 27 || event == 29 || event == 31){
      //set lost signal time
      lostSignalTime = 11000000UL;
      //parse the packet
      postflightPacket(dataPacket2);
      //display to LCD
      postflightLCD();}

    //error packet
    else{errorLCD();}

    parseSustainer = false;
    parseBooster = false;
    radio2Pkt = false;
    }//end check Radio2
      
  //display lost signal status
  if(micros() - lastRX > lostSignalTime && signalEst){

    if(debugSerial){Serial.println(F("Signal Lost!"));}

    /* EEPROM Allocations:
    Posn 00-19: Sustainer Name
    Posn 20-23: Latitude Float
    Posn    24: Latitude Char
    Posn 25-28: Longitude Float
    Posn    29: Longitude Char
    Posn    30: Radio1 Freq
    Posn 31-50: Booster Name
    Posn 51-54: Latitude Float
    Posn    55: Latitude Char
    Posn 56-59: Longitude Float
    Posn    60: Longitude Char
    Posn    61: Radio2 Freq
    Posn    62: Units
    Posn    63: FHSS
    Posn    64: Radio Band
    Posn    65: BlueTooth Enable
    */
    
    //Write the last GPS coordinates to EEPROM for later display
    for(byte i = 0; i < sizeof(rocketName); i++){EEPROM.update(i,rocketName[i]);}
    radioUnion.GPScoord = lastGPSlat;
    for(byte i = 0; i < 4; i++){EEPROM.update(i+20,radioUnion.unionByte[i]);}
    EEPROM.update(24, charGPSlat);
    radioUnion.GPScoord = lastGPSlon;
    for(byte i = 0; i < 4; i++){EEPROM.update(i+25,radioUnion.unionByte[i]);}
    EEPROM.update(29, charGPSlon);
    
    //sync the SD card
    if(SDinit && sustainerFileOpen){sustainerFile.sync();}

    //display last coordinates on LCD
    if(LCD){signalLostLCD();}
    
    signalEst = false;}

  //GPS
  while(Serial2.available() > 0){
    if(!msgRX){msgRX = true;}
    char c = Serial2.read();
    GPS.encode(c);
    if(GPSdebug){serialBuffer[serialPosn] = c; serialPosn++;}}
  if(GPSdebug && msgRX){serialBuffer[serialPosn] = '\0'; Serial.print(serialBuffer); msgRX = false; serialPosn = 0;timeLastNMEA = micros();}

  //check LED
  if(ledLight && micros()-colorStart > 2000000){
    byte bright = whiteIntensity;
    lcd.setBacklight(bright, bright, bright); //Set backlight to white
    ledLight = false;}
    
  delay(50);
}//end loop

void clearIRQ1(){
  //cli();
  radio1Pkt = true;
  //sei();
  }

void clearIRQ2(){
  //cli();
  radio2Pkt = true;
  //sei();
  }
