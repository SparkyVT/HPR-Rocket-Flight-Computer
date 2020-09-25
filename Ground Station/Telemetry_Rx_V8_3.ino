//Sketch uses 28318 bytes (98%) of program storage space. Maximum is 28672 bytes.
//Global variables use 1644 bytes of dynamic memory.
//981 lines of code
//V7_0 uses only a single microprocessor
//V8_0 uses an optimized data stream
//V8_1 adds more display events, revamps message storage, makes field changeable frequencies, and adds selectable units
//V8_2 adds the Serial outputs back into the system
//V8_3 adds 915MHz capability and allows selectable code options to enable: FHSS, LCD, SD, Serial Debug

#include <RH_RF95.h>
#include <SdFat.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

//Radio setup
#define RF95_FREQ 433.250
#define RFM95_RST     4
#define RFM95_CS      8
#define RFM95_IRQ     7

RH_RF95 rf95(RFM95_CS, RFM95_IRQ);

// Software SPI pins
#define SOFT_MISO_PIN   21
#define SOFT_MOSI_PIN   20
#define SOFT_SCK_PIN    19
#define SOFT_CS_PIN     22

// SdFat software SPI template
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> SD;
SdFile myFile;

//LCD Setup
LiquidCrystal lcd(13, 12, 11, 10, 9, 6);

//LCD Color Pins
#define REDLITE   5
#define GREENLITE 3
#define BLUELITE  2

//---------------------------
//Code Control Option variables
//---------------------------
const boolean displayStandard = false;
const boolean lcdON = false;
const boolean SDon = false;
const boolean FHSS = true;//only use for 915MHz Spread Spectrum
const boolean debugSerial = true;//only use for debugging, must turn off either LCD or SD card due to program space limits
//---------------------------
//Data Packet variables
//---------------------------
unsigned long liteStart = 0UL;
byte litePin = 5;
boolean liteUp = false;
uint8_t len;
byte dataPacket[66];
char dataString[100]= "FLIGHT01.txt";
boolean fileOpen = false;
byte n = 0;
byte pktPosn = 0;
int16_t packetnum = 0;
unsigned long timer = 0UL;
//---------------------------
//Radio & SD variables
//---------------------------
boolean radioSetup = false;
boolean testMode = false;
unsigned long testStart = 0UL;
unsigned long lastRX = 0UL;
int battVolt;
boolean SDinit = false;
byte samples = 0;
volatile boolean changeFlag = false;
float unitConvert = 1.0F;
//---------------------------
//preflight output variables
//---------------------------
uint8_t event;
int strPosn = 0;
boolean signalEst = false;
boolean dataProcessed = false;
boolean preFlightWrite = true;
boolean postFlightWrite = true;
unsigned long lostSignalTime = 2000000UL;
byte j=0;
//---------------------------
//preflight output variables
//---------------------------
char rocketName[20]="";
int baseAlt=0;
int baseGPSalt = 0;
byte contCode;
byte satNum = 0;
//---------------------------
//inflight output variables
//---------------------------
uint16_t sampleTime;
int signalStrength=0;
int velocity = 0;
int Alt=0;
int spin=0;
int offVert=0;
int accel=0;
int packetNum = 0;
boolean apogee = false;
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
union {
   float GPScoord;
   unsigned long radioTime;
   byte unionByte[4];
} radioUnion;
byte chnl = 0;
byte hailChnl = 0;
byte nextChnl;
byte nextChnl2;
boolean syncFreq = false;
float freq;
unsigned long lastHopTime = 0UL;
byte chnlUsed = 0;

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
const char event_24[] PROGMEM = "Booster Mains Deployed";
const char event_25[] PROGMEM = "Booster Under Chute";
const char event_26[] PROGMEM = "Time Limit Exceeded";
const char event_27[] PROGMEM = "Touchdown!";

const char *const eventTable[] PROGMEM = {
  event_00, event_01, event_02, event_03, event_04,
  event_05, event_06, event_07, event_08, event_09,
  event_10, event_11, event_12, event_13, event_14,
  event_15, event_16, event_17, event_18, event_19,
  event_20, event_21, event_22, event_23, event_24,
  event_25, event_26, event_27};
  
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

const char *const pyroTable[] PROGMEM = {
  cont_0, cont_1, cont_2, cont_3,
  cont_4, cont_5, cont_6, cont_7, cont_8, cont_9};

//---------------------------
//433 MHz Frequency List
//---------------------------
float radioFreq[9] = {433.250, 433.500, 433.625, 433.750, 433.875, 434.000, 434.125, 434.250, 434.400};

//const byte greenEvents[] = {1, 4, 6, 8, 9, 12, 15, 22, 24};
//const byte redEvents[] = {18, 19, 20};

void setup() {

  //Primary Serial Port
  Serial.begin(9600);
  delay(5000);
  if(debugSerial){Serial.println("Starting...");}

  //Read the battery voltage
  pinMode(A5, INPUT);
  for(byte i = 0; i < 10; i++){battVolt += analogRead(A5);delay(10);}
  battVolt/=10;
  
  //Set Radio Pin Modes
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  
  //Radio manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialise the LCD and SD card
   if(SDon){
    if(debugSerial){Serial.println(F("Starting SD"));}
    SDinit = SD.begin(SOFT_CS_PIN);}
   if(lcdON){
    if(debugSerial){Serial.println(F("Starting LCD"));}
    lcd.begin(20,4);}

  //interrupt for frequency change
  if(!FHSS){pinMode(1, INPUT_PULLUP);}
  
  //read the frequency from EEPROM
  if((byte)EEPROM.read(30) == 255){EEPROM.write(30, 0);}
  chnl = (byte)EEPROM.read(30);
  
  //Initialize the radio
  radioSetup = rf95.init();
  if(FHSS){
    chnlUsed = 3;
    hopFreq();}
  else{rf95.setFrequency(radioFreq[chnl]);}
  delay(3000);
  
  //Set display units
  if(displayStandard){unitConvert = 3.2808;}

  n=1;
  //Create and open the next file on the SD card
  if(SDon){
    while (SD.exists(dataString)) {
      n++;
      if(n<10){itoa(n, dataString + 7,10);}
      else{itoa(n, dataString + 6,10);}
      dataString[8]='.';}
    myFile.open(dataString, FILE_WRITE);
    fileOpen = true;
    dataString[0]=(char)0;
    //Print header
    myFile.println(F("Telemetry Rocket Recorder,RocketName,Continuity,GPSlock,BaseAlt,GPSalt,Latitude,Longitude"));
    myFile.sync();}
  
  /* EEPROM Allocations:
  Posn 00-19: Rocket Name
  Posn 20-23: Latitude Float
  Posn    24: Latitude Char
  Posn 25-28: Longitude Float
  Posn    29: Longitude Char
  Posn    30: Radio Freq
  */

  //Read EEPROM for the last GPS coordinates
  for(byte i = 0; i < sizeof(rocketName); i++){rocketName[i] = EEPROM.read(i);}
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(20+i);}
  lastGPSlat = radioUnion.GPScoord;
  charGPSlat = EEPROM.read(24);
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(25+i);}
  lastGPSlon = radioUnion.GPScoord;
  charGPSlon = EEPROM.read(29);

  if(lcdON){startupLCD();}

}

void loop() {

  //Serial Print Debug
  if(micros()-timer > 1000000UL){
    timer = micros();
    if(debugSerial){
      Serial.println(F("Waiting for Packet"));
      if(SDon && !SDinit){Serial.println(F("SD card failed"));}
      if(!radioSetup){Serial.println(F("Radio Failed"));}}}

  if(!FHSS && digitalRead(1) == LOW){
    chnl++;
    if(chnl >= sizeof(radioFreq)/sizeof(radioFreq[0])){chnl = 0;}
    rf95.setModeIdle();
    rf95.setFrequency(radioFreq[chnl]);
    EEPROM.write(30, (byte)chnl);
    //changeFlag = false;
    changeFreqLCD();
    lcd.setCursor(0,2);
    lcd.print(F("DONE!"));}
    
  if (rf95.available()){
    // Should be a message for us now   
    len = sizeof(dataPacket);
    
    if (rf95.recv(dataPacket, &len)){

      lastRX = timer = micros();
      event = (byte)dataPacket[0];
      
      if(debugSerial){Serial.print(F("Event: "));Serial.println((int)event);}
      
      signalStrength = rf95.lastRssi();

      //sync packet
      if(event == 255){syncPkt();}

      //preflight packet
      else if(event == 0){
        //signal established
        signalEst = true;
        //set lost signal time
        lostSignalTime = 6000000UL;
        //parse the packet
        preflightPacket();}

      //inflight packet
      else if(event < 26){  
        //two types of inflight packets: 1 sample & 4 sample
        samples = (len > 52) ? 4 : 1;
        //set lost signal time
        lostSignalTime = 2000000UL;
        //parse the packet
        inflightPacket();}

      //postflight packet
      else if(event >= 26){
        //set lost signal time
        lostSignalTime = 11000000UL;
        //parse the packet
        postflightPacket();}
        
      }}

  //if we missed a hop, go to next channel or go to sync channel
  if(FHSS && event == 0 && micros() - lastHopTime > 900000UL && signalEst && !syncFreq){if(debugSerial){Serial.print(F("----Missed Hop, moving to ch: "));} hopFreq();}
  else if(FHSS && micros() - lastHopTime > 630000UL && signalEst && !syncFreq){if(debugSerial){Serial.print(F("----Missed Hop, moving to ch: "));} hopFreq();}
  
  //display lost signal status
  if(micros() - lastRX > lostSignalTime && signalEst){
    /* EEPROM Allocations:
  Posn 00-19: Rocket Name
  Posn 20-23: Latitude Float
  Posn    24: Latitude Char
  Posn 25-28: Longitude Float
  Posn    29: Longitude Char
  Posn    30: Radio Freq
  */
    //Write the last GPS coordinates to EEPROM for later display
    for(byte i = 0; i < sizeof(rocketName); i++){EEPROM.write(i,rocketName[i]);}
    radioUnion.GPScoord = lastGPSlat;
    for(byte i = 0; i < 4; i++){EEPROM.write(i+20,radioUnion.unionByte[i]);}
    EEPROM.write(24, charGPSlat);
    radioUnion.GPScoord = lastGPSlon;
    for(byte i = 0; i < 4; i++){EEPROM.write(i+25,radioUnion.unionByte[i]);}
    EEPROM.write(29, charGPSlon);

    //sync the SD card
    if(fileOpen && SDon){myFile.sync();}

    //display last coordinates on LCD
    if(lcdON){signalLostLCD();}
    }
    
}//end loop
