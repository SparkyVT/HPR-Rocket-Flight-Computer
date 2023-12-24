//V7 uses only a single microprocessor
//V8 uses an optimized data stream
//V9 overhauls the whole system
//V10 eliminates the RadioHead library
//V10_1 finalizes functionality to use two radios in tandem

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

char serialBuffer[256];
int serialPosn = 0;

byte dataPkt[80];
volatile boolean processIRQ = false;
uint32_t timeLastPkt = 0UL;
uint32_t debugTime;
int16_t pktRssi;
float radio1Freq = 433.500;
float radio2Freq = 902.300;

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
FsFile settingsFile;

//LCD Setup
SerLCD lcd;

// GPS Setup
TinyGPSPlus GPS;

//Timer for the radio
IntervalTimer radioTimer;

//---------------------------
//User Settings
//---------------------------
struct{
  boolean radio1enable = true;
  boolean radio2enable = true;
  float radio1freq;
  float radio2freq;
  boolean radio1FHSS = false;
  boolean radio2FHSS = false;
  byte radio1Pwr = 13;
  byte radio2Pwr = 13;
  boolean displayMetric = false;
  byte debugSerial = 1;
  char callSign[6] = "KK4ELF";
  boolean LCD = false;
  byte LCDbrightness = 75;
  byte LCDcontrast = 50;
  boolean LCDcolorEvents = false;
  boolean blueTooth = true;
  boolean enableSD = true;
  boolean enableGPS = true;
} settings;
//---------------------------
//Radio Control variables
//---------------------------
typedef struct{
  float frq;
  byte num;
  boolean enable = true;
  boolean FHSS = false;
  boolean status = false;
  byte TXpwr = 13;
  byte dataPacket[256];
  boolean signalEst = false;
  uint32_t lastRX = 0UL;
  uint8_t cs;
  uint8_t rst;
  uint8_t hailChnl = 0;
  uint8_t chnl1;
  uint8_t nextChnl;
  uint8_t nextChnl2;
  uint8_t chnlUsed = 0;
  boolean syncFreq = true;
  uint32_t lastHopTime = 0UL;
} radio;
radio radio1;
radio radio2;
radio *activeRadio;
//---------------------------
//Data Packet variables
//---------------------------
uint8_t len;
char dataString[512];
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
boolean GPSdebug = false;
boolean debugSerial = true;
float unitConvert = 3.28084F;
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

uint8_t eventPosn = 0;

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

  //Start SD card
  SDinit = SD.begin(SD_CONFIG);
  if(!SDinit){Serial.println(F("SD card failed"));}
  else{Serial.println(F("SD Card OK!"));}
  //Read user settings  
  readRadioSettingsSD();
  
  //set the radio pointers
  radio1.num = 1;
  radio2.num = 2;
  radio1.cs = radio1CS;
  radio2.cs = radio2CS;
  radio1.rst = radio1RST;
  radio2.rst = radio2RST;

  //set radio pins
  pinMode(radio1.cs, OUTPUT);
  pinMode(radio1.rst, OUTPUT);
  pinMode(radio2.cs, OUTPUT);
  pinMode(radio2.rst, OUTPUT);
  digitalWrite(radio1.cs, HIGH);
  digitalWrite(radio1.rst, HIGH);
  digitalWrite(radio2.cs, HIGH);
  digitalWrite(radio2.rst, HIGH);
  
  //Radio1 setup
  delay(500);
  activeRadio = &radio1;
  activeRadio->status = radioBegin();
  if(!activeRadio->status){Serial.print("Radio ");Serial.print(activeRadio->num);Serial.println(" Failed");}
  else{Serial.print("Radio");Serial.print(activeRadio->num);Serial.println(" OK!");}
  //if the radio is active
  if(activeRadio->status){
    //set frequency
    if(setRadioFreq(activeRadio->frq)){Serial.print("Radio");Serial.print(activeRadio->num);Serial.print(" on ");Serial.print(activeRadio->frq, 3);Serial.println(" MHz");}
    else{Serial.print("Radio");Serial.print(activeRadio->num);Serial.println(" Set Frequency Failed");}
    //set to recieve mode
    radioSetMode(RXmode);}
  
  //Radio2 setup
  delay(500);
  activeRadio = &radio2;
  activeRadio->status = radioBegin();
  if(!activeRadio->status){Serial.print("Radio ");Serial.print(activeRadio->num);Serial.println(" Failed");}
  else{Serial.print("Radio");Serial.print(activeRadio->num);Serial.println(" OK!");}
  //if the radio is active
  if(activeRadio->status){
    //set frequency
    if(setRadioFreq(activeRadio->frq)){Serial.print("Radio");Serial.print(activeRadio->num);Serial.print(" on ");Serial.print(activeRadio->frq, 3);Serial.println(" MHz");}
    else{Serial.print("Radio");Serial.print(activeRadio->num);Serial.println(" Set Frequency Failed");}
    //set to recieve mode
    radioSetMode(RXmode);}

  //set active radio based on user settings
  if(radio1.enable & !radio2.enable){activeRadio = &radio1;}
  else if(!radio1.enable & radio2.enable){activeRadio = &radio2;}
  else{activeRadio = &radio1;}

  /* EEPROM Allocations:
  Posn 00-19: Sustainer Name
  Posn 20-23: Latitude Float
  Posn    24: Latitude Char
  Posn 25-28: Longitude Float
  Posn    29: Longitude Char
  Posn 30-49: Booster Name
  Posn 50-53: Latitude Float
  Posn    54: Latitude Char
  Posn 55-58: Longitude Float
  Posn    59: Longitude Char
  */

  //Read EEPROM for the sustainer last GPS coordinates
  for(byte i = 0; i < sizeof(rocketName); i++){rocketName[i] = EEPROM.read(i);}
  //for(byte i = 0; i<8;i++){EEPROM.write(20+i, 0);}
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(20+i);}
  lastGPSlat = radioUnion.GPScoord;
  charGPSlat = EEPROM.read(24);
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(25+i);}
  lastGPSlon = radioUnion.GPScoord;
  charGPSlon = EEPROM.read(29);

  //configure GPS
  //configGPS();
  
  //Display the startup screen
  Serial.println("starting LCD");
  startupLCD();

}//end setup

void loop() {

  boolean msgRX = false;
  boolean pktRX = false;
  boolean goodPacket = false;

  //Serial Print Debug
  if(micros()-timer > 1000000UL && micros() - timeLastNMEA > 500000){
    timer = micros();
    Serial.print(F("Waiting for Packet, frequency "));
    Serial.print(readRadioFreq(), 3);
    Serial.println(" MHz");}

  
  //check radios for recieved packet with interrupt
  if(radio1Pkt || radio2Pkt){

    //select the radio that recieved a packet, if both do radio1 first
    if(radio1Pkt){activeRadio = &radio1;}
    else{activeRadio = &radio2;}

    //set the timers
    lastRX = micros();
    activeRadio->lastRX = lastRX;
    timer = lastRX;

    //get the packet from the radio
    radioRecvPkt(activeRadio->dataPacket);

    //get signal strength
    signalStrength = pktRssi;

    //check the callsign
    goodPacket = true;
    if(activeRadio->FHSS){eventPosn = 0;}
    for(uint8_t i = 0; i < eventPosn; i++){
      if(activeRadio->dataPacket[i] != settings.callSign[i]){
        goodPacket = false;
        if(activeRadio->num == 1){radio1Pkt = false;}
        else{radio2Pkt = false;}
        break;}}}

  //if we have a good packet
  if(goodPacket){
    
    //set event flags
    event = (byte)activeRadio->dataPacket[0];
    if(settings.debugSerial){
      if(event != 255){Serial.println("-------Data Packet Received--------");}
      else{Serial.println("-------Sync Packet Received--------");}
      Serial.print("Packet on radio");Serial.print(activeRadio->num);Serial.print(", event: ");Serial.println(event);
      Serial.print("Length: ");Serial.println(len);
      Serial.print("TimeStamp: ");Serial.println(activeRadio->lastRX);}
    //determine if it is a sustainer or booster packet
    if(event < 21){parseSustainer = true;}
    else if(event == 26 || event == 27 || event == 28){parseSustainer = true;}
    else if(event != 255 && event < 33){parseBooster = true;}

    //Create the SD file
    if(parseSustainer && !sustainerFileCreated && SDinit){createSustainerFile();}
    if(parseBooster   && !boosterFileCreated   && SDinit){createBoosterFile();}
    
    //sync packet
    if(event == 255){syncPkt(activeRadio->dataPacket);}
    
    //preflight packet
    else if(event == 0 || event == 30){
      //parse the preFlight packet
      preflightPacket(activeRadio->dataPacket);
      //signal established
      signalEst = true;
      activeRadio->signalEst = true;
      //set lost signal time
      lostSignalTime = 2000000UL;
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
      inflightPacket(activeRadio->dataPacket);
      
      //display to LCD
      inflightLCD();}

    //postflight packet
    else if(event == 26 || event == 27 || event == 29 || event == 31){
      //set lost signal time
      lostSignalTime = 35000000UL;
      //parse the packet
      postflightPacket(activeRadio->dataPacket);
      //display to LCD
      postflightLCD();}
      
    //error packet
    else{errorLCD();}

    parseSustainer = false;
    parseBooster = false;
    if(activeRadio->num == 1){radio1Pkt = false;}
    else{radio2Pkt = false;}
    goodPacket = false;
    }//end check Radio2

  //On FHSS, if signal was established on the ground but a packet was missed, then move to the next channel
  if(event == 0 && radio1.FHSS && micros() - radio1.lastHopTime > 2000000UL && radio1.signalEst && radio1.chnlUsed < 3){
    Serial.println("-------Radio 1 Missed Packet--------");
    activeRadio = &radio1;
    //signal to move to the synch freq because the necessary lead time on the ground is too long to try and catch the next packet
    activeRadio->chnlUsed = 3;
    hopFreq();}
    
  //On FHSS, if signal was established in-flight but a packet was missed, then move to the next channel
  else if(radio1.FHSS && micros() - radio1.lastHopTime > 630000UL && radio1.signalEst && radio1.chnlUsed < 3){
    Serial.println("-------Radio 1 Missed Packet--------");
    activeRadio = &radio1;
    hopFreq();}
    
  if(event == 0 && radio2.FHSS && micros() - radio2.lastHopTime > 2000000UL && radio2.signalEst && radio2.chnlUsed < 3){
    Serial.println("-------Radio 1 Missed Packet--------");
    activeRadio = &radio2;
    //signal to move to the synch freq because the necessary lead time on the ground is too long to try and catch the next packet
    activeRadio->chnlUsed = 3;
    hopFreq();}
    
  else if(radio2.FHSS && micros() - radio2.lastHopTime > 630000UL && radio2.signalEst && radio2.chnlUsed < 3){
    Serial.println("-------Radio 2 Missed Packet--------");
    activeRadio = &radio2;
    hopFreq();}
      
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
    signalLostLCD();
    
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
   
}//end loop

void clearIRQ1(){
  noInterrupts();
  radio1Pkt = true;
  interrupts();}

void clearIRQ2(){
  noInterrupts();
  radio2Pkt = true;
  interrupts();}
