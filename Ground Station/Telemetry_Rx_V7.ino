//Sketch uses 26852 bytes (93%) of program storage space. Maximum is 28672 bytes.
//Global variables use 1561 bytes of dynamic memory.  675 lines of code
//V7 uses only a single microprocessor

#include <SPI.h>
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
const uint8_t SOFT_MISO_PIN = 21;
const uint8_t SOFT_MOSI_PIN = 20;
const uint8_t SOFT_SCK_PIN  = 19;
const uint8_t SOFT_CS_PIN   = 22;

// SdFat software SPI template
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> SD;
SdFile myFile;

//LCD Setup
LiquidCrystal lcd(13, 12, 11, 10, 9, 6);

byte dataPacket[90];
char dataString[100]= "FLIGHT01.txt";
byte n = 0;
byte pktPosn = 0;
int16_t packetnum = 0;
unsigned long timer = 0UL;
boolean radioSetup = false;
boolean sendPacket = true;
boolean testMode = false;
unsigned long testStart = 0UL;
unsigned long lastRX = 0UL;
unsigned long debugStart = 0UL;
unsigned long debugStop = 0UL;
int battVolt;
boolean SDinit = false;
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
byte contCode;
//---------------------------
//inflight output variables
//---------------------------
unsigned long currentTime;
unsigned long prevTime;
float prevAlt;
int signalStrength=0;
int accelVel=0;
int accelAlt=0;
int baroAlt=0;
int angX=0;
int angY=0;
int angZ=0;
int accelX=0;
byte packetNum = 0;
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
char charGPSlat;
float GPSlatitude;
char charGPSlon;
float GPSlongitude;
float lastGPSlat;
char lastCharGPSlat;
float lastGPSlon;
char lastCharGPSlon;
union {
   float GPScoord;
   unsigned long radioTime;
   byte unionByte[4];
} radioUnion;

void setup() {

  Serial.begin(9600);

  pinMode(A5, INPUT);
  battVolt = analogRead(A5);
  
  //Set Radio Pin Modes
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  
  //Radio manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  //Initialize the radio
  radioSetup = rf95.init();
  rf95.setFrequency(RF95_FREQ);
  
  // Initialise the LCD and SD card
  SDinit = SD.begin(SOFT_CS_PIN);
  lcd.begin(20,4);

  n=1;
  //Create and open the next file on the SD card
  while (SD.exists(dataString)) {
    n++;
    if(n<10){itoa(n, dataString + 7,10);}
    else{itoa(n, dataString + 6,10);}
    dataString[8]='.';}
  myFile.open(dataString, FILE_WRITE);
  dataString[0]=(char)0;
  //Print header
  myFile.println(F("Telemetry Rocket Recorder, Rocket Name, Continuity, GPS Lock, Base Altitude, GPS Altitude, GPS Latitude, GPS Longitude"));
  myFile.sync();
  
  /* EEPROM Allocations:
  Posn 00-14: Rocket Name
  Posn 15-16: Latitude Int
  Posn 17-18: Latitide Dec
  Posn    19: Latitude Char
  Posn 20-21: Longitude Int
  Posn 22-23: Longitude Dec
  Posn    24: Longitude Char
  */
  //Read EEPROM for the last GPS coordinates
  for(byte i = 0; i < sizeof(rocketName); i++){rocketName[i] = EEPROM.read(i);}
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(20+i);}
  lastGPSlat = radioUnion.GPScoord;
  lastCharGPSlat = EEPROM.read(24);
  for(byte i = 0; i < 4; i++){radioUnion.unionByte[i] = EEPROM.read(25+i);}
  lastGPSlon = radioUnion.GPScoord;
  lastCharGPSlon = EEPROM.read(29);

  startupLCD();
  
  delay(1000);
}

void loop() {

  //Serial Print Debug
  if(micros()-timer > 1000000UL){
    Serial.println(F("Waiting for Packet"));
    if(!SDinit){Serial.println(F("SD card failed"));}
    timer = micros();
    if(!radioSetup){Serial.println(F("Radio Failed"));}
    }

  
  if (rf95.available() || (testMode && micros() - lastRX > 200000UL) ){
    // Should be a message for us now   
    uint8_t len = sizeof(dataPacket);
    Serial.println(F("Packet Recieved: "));
    
    if (rf95.recv(dataPacket, &len) || testMode){

      lastRX = micros();
      event = (byte)dataPacket[0];
      Serial.println((int)event);
      
      signalStrength = rf95.lastRssi();

      //preflight packet
      if(event == 0){
        //signal established
        signalEst = true;
        //set lost signal time
        lostSignalTime = 6000000UL;
        //parse the packet
        preflightPacket();}

      //inflight packet
      else if(event != 8 && event != 9){  
        //set lost signal time
        lostSignalTime = 2000000UL;
        //parse the packet
        inflightPacket();}

      //postflight packet
      else if(event == 8 || event == 9){
        //set lost signal time
        lostSignalTime = 11000000UL;
        //parse the packet
        postflightPacket();}
        
      }}
      
  //display lost signal status
  if(micros() - lastRX > lostSignalTime && signalEst){
    /* EEPROM Allocations:
    Posn 00-14: Rocket Name
    Posn 15-16: Latitude Int
    Posn 17-18: Latitide Dec
    Posn    19: Latitude Char
    Posn 20-21: Longitude Int
    Posn 22-23: Longitude Dec
    Posn    24: Longitude Char
    */
    //Write the last GPS coordinates to EEPROM for later display
    for(byte i = 0; i < sizeof(rocketName); i++){EEPROM.write(i,rocketName[i]);}
    radioUnion.GPScoord = lastGPSlat;
    for(byte i = 0; i < 4; i++){EEPROM.write(i+20,radioUnion.unionByte[i]);}
    EEPROM.write(24, lastCharGPSlat);
    radioUnion.GPScoord = lastGPSlon;
    for(byte i = 0; i < 4; i++){EEPROM.write(i+25,radioUnion.unionByte[i]);}
    EEPROM.write(29, lastCharGPSlon);

    //sync the SD card
    myFile.sync();

    //display last coordinates on LCD
    signalLostLCD();}
    
}//end loop

