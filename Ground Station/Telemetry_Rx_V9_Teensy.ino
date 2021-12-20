//V7 uses only a single microprocessor
//V8 uses an optimized data stream
//V9 overhauls the whole system

//TO DO:
//0) Read user data from SD card
//1) Create BLE Service
//2) Pin correction for LCD - OK
//3) Radio 1 controls - OK
//4) Radio 2 controls - OK
//5) GPS Services
//6) LSM9DS1 orientation
//7) LSM9DS1 magnetic
//8) Include Software SPI - OK
//9) Create user menu interface
//10) Delineate FHSS with a booster

#include <RH_RF95.h>
#include <SdFat.h>
#include <Wire.h>
//#include <i2c_t3.h>
#include <SerLCD.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <SPI.h>

//Radio control pins
#define radio1RST     14//ok
#define radio1CS      15//ok
#define radio1IRQ     17//ok
#define radio2RST     16//ok
#define radio2CS      20//ok
#define radio2IRQ     22//ok
#define radio1Freq    433.500
#define radio2Freq    433.250

void preflightPacket(byte dataPacket[]);
void inflightPacket(byte dataPacket[]);
void postflightPacket(byte dataPacket[]);

char serialBuffer[256];
int serialPosn = 0;

//Radiohead RFM95
RH_RF95 radio1(radio1CS, radio1IRQ);
RH_RF95 radio2(radio2CS, radio2IRQ);

// Software SPI pins
#define ENABLE_SOFTWARE_SPI_CLASS 1
#define SOFT_MISO_PIN   2
#define SOFT_MOSI_PIN   1 
#define SOFT_SCK_PIN    3 
#define SOFT_CS_PIN     0 

// SdFat software SPI template
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> SD;
File sustainerFile;
File boosterFile;

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
boolean GPSdebug = true;
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
const char event_00[] PROGMEM = "Preflight";
const char event_01[] PROGMEM = "Liftoff";
const char event_02[] PROGMEM = "Booster Burnout";
const char event_03[] PROGMEM = "Apogee Detected";
const char event_04[] PROGMEM = "Firing Apogee Pyro";
const char event_05[] PROGMEM = "Separation Detected!";
const char event_06[] PROGMEM = "Firing Mains";
const char event_07[] PROGMEM = "Under Chute";
const char event_08[] PROGMEM = "Ejecting Booster";
const char event_09[] PROGMEM = "Firing 2nd Stage";
const char event_10[] PROGMEM = "2nd Stage Ignition";
const char event_11[] PROGMEM = "2nd Stage Burnout";
const char event_12[] PROGMEM = "Firing Airstart1";
const char event_13[] PROGMEM = "Airstart1 Ignition";
const char event_14[] PROGMEM = "Airstart1 Burnout";
const char event_15[] PROGMEM = "Firing Airstart2";
const char event_16[] PROGMEM = "Airstart2 Ignition";
const char event_17[] PROGMEM = "Airstart2 Burnout";
const char event_18[] PROGMEM = "NoFire: Rotn Limit";
const char event_19[] PROGMEM = "NoFire: Alt Limit";
const char event_20[] PROGMEM = "NoFire: Rotn/Alt Lmt";
const char event_21[] PROGMEM = "Booster Apogee";
const char event_22[] PROGMEM = "Booster Apogee Fire";
const char event_23[] PROGMEM = "Booster Separation!";
const char event_24[] PROGMEM = "Booster Main Deploy";
const char event_25[] PROGMEM = "Booster Under Chute";
const char event_26[] PROGMEM = "Time Limit Exceeded";
const char event_27[] PROGMEM = "Touchdown!";
const char event_28[] PROGMEM = "Power Loss! Restart!";
const char event_29[] PROGMEM = "Booster Touchdown!";
const char event_30[] PROGMEM = "Booster Preflight";
const char event_31[] PROGMEM = "Booster Time Limit";
const char event_32[] PROGMEM = "Booster Pwr Restart";

const char *const eventTable[] PROGMEM = {
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
const char cont_0[] PROGMEM = "No Pyros Detected!";
const char cont_1[] PROGMEM = "No Continuity Pyro 1";
const char cont_2[] PROGMEM = "No Continuity Pyro 2";
const char cont_3[] PROGMEM = "No Continuity Pyro 3";
const char cont_4[] PROGMEM = "No Continuity Pyro 4";
const char cont_5[] PROGMEM = "All 3 Pyros Detected";
const char cont_6[] PROGMEM = "All 4 Pyros Detected";
const char cont_7[] PROGMEM = "Pyro Apogee Only";
const char cont_8[] PROGMEM = "Pyro Mains Only";
const char cont_9[] PROGMEM = "Pyro Mains & Apogee";

byte greenCont[5] = {5, 6, 7, 8, 9};
byte redCont[5]   = {0, 1, 2, 3, 4};

const char *const pyroTable[] PROGMEM = {
  cont_0, cont_1, cont_2, cont_3, cont_4, 
  cont_5, cont_6, cont_7, cont_8, cont_9};

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

void setup() {

  //Primary Serial Port
  Serial.begin(9600);
  Serial2.begin(9600);
  
  //Read the battery voltage
  pinMode(A0, INPUT);
  battVolt = 0;
  for(byte i = 0; i < 5; i++){battVolt += analogRead(A10);delay(10);}
  battVolt/=5;

  //radio manual reset
  pinMode(radio1RST, OUTPUT);
  pinMode(radio2RST, OUTPUT);
  digitalWrite(radio1RST, HIGH);
  digitalWrite(radio2RST, HIGH);
  delay(100);
  digitalWrite(radio1RST, LOW);
  digitalWrite(radio2RST, LOW);
  delay(100);
  digitalWrite(radio1RST, HIGH);
  digitalWrite(radio2RST, HIGH);
  delay(500);

  //Radiohead RFM95 setup
  radio1status = radio1.init();
  delay(50);
  radio2status = radio2.init();
  if(!radio1status){radio1status = radio1.init();}
  if(!radio1status){Serial.println(F("Radio1 Failed"));}
  else{Serial.println(F("Radio1 OK!"));}
  if(!radio2status){Serial.println(F("Radio2 Failed"));}
  else{Serial.println(F("Radio2 OK!"));}

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
  
  if(radio1.setFrequency(radio1Freq)){Serial.print("Radio1 on ");Serial.print(radio1Freq, 3);Serial.println(" MHz");}
  else{Serial.println("Radio1 Set Frequency Failed");}
  if(radio2.setFrequency(radio2Freq)){Serial.print("Radio2 on ");Serial.print(radio2Freq, 3);Serial.println(" MHz");}
  else{Serial.println("Radio2 Set Frequency Failed");}

  //Set the flag and interrupts if FHSS is used
  if((byte)EEPROM.read(63)==1){
    FHSS = true;
//    pinMode(radio1IRQ, INPUT_PULLUP);
//    pinMode(radio2IRQ, INPUT_PULLUP);
    hailChnl_1 = chnl1;
    hailChnl_2 = chnl2;}
    
  // Initialise the LCD and SD card
  SDinit = SD.begin(SOFT_CS_PIN);
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
  if((byte)EEPROM.read(62)==1){unitConvert = 3.2808;}

  //Set enabling of BlueTooth
  if((byte)EEPROM.read(65)==1){blueTooth = true;}

  //configure GPS
  configGPS();
  
  //Display the startup screen
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
  if(radio1.available()){

    lastRX = micros();

    //get the packet from the radio
    len = len1;
    radio1.recv(dataPacket1, &len);

    //get signal strength
    signalStrength = radio1.lastRssi();
    
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
      postflightPacket(dataPacket2);
      //display to LCD
      postflightLCD();}

    //error packet
    else{errorLCD();}

    parseSustainer = false;
    parseBooster = false;
    }//end check Radio1

  //check radio2
  if(radio2.available()){

    lastRX = micros();

    //get the packet from the radio
    len = len2;
    radio2.recv(dataPacket2, &len);

    //get signal strength
    signalStrength = radio2.lastRssi();
    
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
