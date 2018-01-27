//Sketch uses 11260 bytes (44%) of program storage space. Maximum is 28672 bytes.
//Global variables use 1034 bytes of dynamic memory.

//Drives Radio and slave
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>

//Radio setup
#define RF95_FREQ 433.250
#define RFM95_RST     4   // "A"
#define RFM95_CS      8   // "B"
#define RFM95_IRQ     7   // "C"
#define LED           13  // LED

RH_RF95 rf95(RFM95_CS, RFM95_IRQ);//IRQN used previously

char dataString[100] = "";
char localBuffer[20] = "";
byte n = 0;
byte i = 0;
unsigned long timer = 0UL;
boolean radioSetup = false;
boolean sendPacket = true;
byte charStart=0;
byte charStop=0;
boolean rcvGPSdata = false;
boolean testMode = false;
unsigned long testStart = 0UL;
unsigned long lastRX = 0UL;
//---------------------------
//preflight output variables
//---------------------------
char event;
char rocketName[15]="";
char contCode;
int baseAlt;
//---------------------------
//inflight output variables
//---------------------------
int intFlightTime;
byte decFlightTime;
int signalStrength;
int accelVel;
int accelAlt;
int baroAlt;
int angX;
int angY;
int angZ;
int accelX;
//---------------------------
//Postflight output variables
//---------------------------
int maxAltitude;
int maxVelocity;
int maxG;
int maxGPSalt;
//GPS output variables
char GPSlock;
int GPSalt;
char charGPSlat;
int intGPSlat;
int decGPSlat;
char charGPSlon;
int intGPSlon;
int decGPSlon;

void parsePreflightData();
void parseInFlightData();
void parsePostflightData();

void sendInflightData();
void sendPreflightData();
void sendPostflightData();

void feedBuffer();
void floatFeedBuffer();

void setup() {

  //Wire.begin();
  Serial.begin(9600);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  radioSetup = rf95.init();
  rf95.setFrequency(RF95_FREQ);

  testStart = micros();
}

void loop() {

if(micros()-timer > 1000000UL){Serial.println("Waiting for Packet");
  timer = micros();
  if(!radioSetup){Serial.println("Radio Failed");}}
if (rf95.available() || (testMode && micros() - lastRX > 1000000UL) )
  {
    // Should be a message for us now   
    uint8_t len = sizeof(dataString);
    String flightData;
    sendPacket = true;
    Serial.println("Packet Recieved: ");
    
    if (rf95.recv(dataString, &len) || testMode){

      if(testMode){
        
        if(micros()-testStart < 5000000UL){flightData = "0,Redbird,1,1,409,569,N3836.2783,W8955.0225";}
        else if (micros() - testStart < 7000000UL){flightData = "1,0.00,100,200,250,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 9000000UL){flightData = "2,2.10,400,400,-50,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 11000000UL){flightData = "3,4.05,301,600,-50,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 13000000UL){flightData = "4,6.00,200,700,750,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 15000000UL){flightData = "5,8.09,100,800,850,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 17000000UL){flightData = "6,10.00,0,900,950,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 19000000UL){flightData = "7,20.20,0,300,350,5,900,-6,130,146,N3836.2922,W8955.0342,";}
        else if (micros() - testStart < 23000000UL){flightData = "8,11101,202,290,567,1,500,N3836.2961,W8955.0400,";}
        else {sendPacket = false;}
        
        flightData.toCharArray(dataString, sizeof(dataString));
        Serial.println("Test Packet Sent");}

      signalStrength = rf95.lastRssi();
      
      n++;
      Serial.println(dataString);

      lastRX = micros();
      
      charStart = 0;
      event = dataString[charStart];
      charStop = 1;

      //Serial.println(contCode);
      //Parse data and send to the slave
      if(event == '0' && sendPacket){parsePreflightData();sendPreflightData();}
      else if (event != '8' && event != '9' && sendPacket){parseInflightData();sendInflightData();}
      else if ((event == '8' || event == '9') && sendPacket){parsePostflightData();sendPostflightData();}
    }}}

void parsePreflightData(){
  //Parse the data
  Serial.print("Event: ");Serial.println(event);
  
  //--------------
  //rocket name
  //char rocketName[15];
  charStart=charStop+1;
  charStop=charStart;
  while(dataString[charStop]!=','){charStop++;}
  memset(rocketName, 0, sizeof(rocketName));
  memcpy(rocketName, dataString+charStart, charStop-charStart);
  Serial.print("rocketName: ");Serial.print(rocketName);Serial.println(" ");
  
  //Continuity Code
  //char contCode;
  charStart = charStop+1;
  charStop = charStart+1;
  contCode = dataString[charStart];
  Serial.print("contCode: ");Serial.print(contCode);Serial.println(" ");
  
  //GPS lock
  //char GPSlock;
  charStart = charStop+1;
  charStop = charStart+1;
  GPSlock = dataString[charStart];
  Serial.print("GPSlock: ");Serial.print(GPSlock);Serial.println(" ");
  
  //Base Altitude
  //int baseAlt;
  feedBuffer();
  baseAlt = atoi(localBuffer);
  Serial.print("baseAlt: ");Serial.print(baseAlt);Serial.println(" ");
  
  //reset GPS data
  GPSalt = 0;
  charGPSlat = '\0';
  intGPSlat = 0;
  decGPSlat = 0;
  charGPSlon = '\0';
  intGPSlon = 0;
  decGPSlon = 0;
  
  //Only parse GPS data if it was transmitted
  if(dataString[charStop+1] != ','){
    
    //base GPS altitude
    //int GPSalt;
    feedBuffer();
    GPSalt = atoi(localBuffer);

    //GPS latitude
    //char GPSlat;
    charStart = charStop+1;
    charStop = charStart;
    charGPSlat = dataString[charStart];
    //int intGPSlat;
    feedBuffer();
    intGPSlat = atoi(localBuffer);
    //int decGPSlat;
    feedBuffer();
    decGPSlat = atoi(localBuffer);
    
    //GPS longitude
    //char GPSlon;
    charStart = charStop+1;
    charStop = charStart;
    charGPSlon = dataString[charStart];
    //int intGPSlon;
    feedBuffer();
    intGPSlon = atoi(localBuffer);
    //int decGPSlon;
    feedBuffer();
    decGPSlon = atoi(localBuffer);}}

void parseInflightData(){
  //Parse the data
  //--------------
  //Timestamp
  //int intFlightTime;
  //int decFlightTime;
  feedBuffer();
  intFlightTime = atoi(localBuffer);
  feedBuffer();
  decFlightTime = (byte)(atoi(localBuffer));
  
  //Accelerometer Integrated Velocity
  //int accelVel;
  feedBuffer();
  accelVel = atoi(localBuffer);
  
  //Accelerometer Integrated Altitude
  //int accelAlt;
  feedBuffer();
  accelAlt = atoi(localBuffer);
  
  //Barometric Altitude
  //int baroAlt;
  feedBuffer();
  baroAlt = atoi(localBuffer);

  //Integrated X Rotation
  //int angX;
  feedBuffer();
  angX = atoi(localBuffer);

  //Integrated Y Rotation
  //int angY;
  feedBuffer();
  angY = atoi(localBuffer);

  //Integrated Z Rotation
  //int angZ;
  feedBuffer();
  angZ = atoi(localBuffer);

  //Accelerometer Reading
  //int accelX;
  feedBuffer();
  accelX = atoi(localBuffer);

  //reset GPS data
  GPSalt = 0;
  charGPSlat = '\0';
  intGPSlat = 0;
  decGPSlat = 0;
  charGPSlon = '\0';
  intGPSlon = 0;
  decGPSlon = 0;
  
  //Only parse GPS data if it was transmitted
  if(dataString[charStop+1] != ','){
    
    //base GPS altitude
    //int GPSalt;
    feedBuffer();
    GPSalt = atoi(localBuffer);

    //GPS latitude
    //char GPSlat;
    charStart = charStop+1;
    charStop = charStart;
    charGPSlat = dataString[charStart];
    //int intGPSlat;
    feedBuffer();
    intGPSlat = atoi(localBuffer);
    //int decGPSlat;
    feedBuffer();
    decGPSlat = atoi(localBuffer);
    
    //GPS longitude
    //char GPSlon;
    charStart = charStop+1;
    charStop = charStart;
    charGPSlon = dataString[charStart];
    //int intGPSlon;
    feedBuffer();
    intGPSlon = atoi(localBuffer);
    //int decGPSlon;
    feedBuffer();
    decGPSlon = atoi(localBuffer);}}

void parsePostflightData(){
  //Parse the data
  //--------------
  //Maximum Barometric Altitude
  //int maxAltitude;
  feedBuffer();
  maxAltitude = atoi(localBuffer);
  
  //Maximum Integrated Velocity
  //int maxVelocity;
  feedBuffer();
  maxVelocity = atoi(localBuffer);
  
  //Maximum recorded G load
  //int maxG;
  feedBuffer();
  maxG = atoi(localBuffer);
  
  //Maximum recorded GPS altitude
  //int maxGPSalt;
  feedBuffer();
  maxGPSalt = atoi(localBuffer);

  //GPS lock
  //char GPSlock;
  charStart = charStop+1;
  charStop = charStart+1;
  GPSlock = dataString[charStart];
  
  //reset GPS data
  GPSalt = 0;
  charGPSlat = '\0';
  intGPSlat = 0;
  decGPSlat = 0;
  charGPSlon = '\0';
  intGPSlon = 0;
  decGPSlon = 0;
  
  //Only parse GPS data if it was transmitted
  if(dataString[charStop+1] != ','){

    //base GPS altitude
    //int GPSalt;
    feedBuffer();
    GPSalt = atoi(localBuffer);
    
    //GPS latitude
    //char GPSlat;
    charStart = charStop+1;
    charStop = charStart;
    charGPSlat = dataString[charStart];
    //int intGPSlat;
    feedBuffer();
    intGPSlat = atoi(localBuffer);
    //int decGPSlat;
    feedBuffer();
    decGPSlat = atoi(localBuffer);
    
    //GPS longitude
    //char GPSlon;
    charStart = charStop+1;
    charStop = charStart;
    charGPSlon = dataString[charStart];
    //int intGPSlon;
    feedBuffer();
    intGPSlon = atoi(localBuffer);
    //int decGPSlon;
    feedBuffer();
    decGPSlon = atoi(localBuffer);}}
  
void sendPreflightData(){
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(event);//1 byte
  Wire.write(contCode);//1 byte - 17 Total
  Wire.write(GPSlock);//1 byte - 18 Total
  Wire.write(rocketName, sizeof(rocketName));//15 bytes - 16 Total
  sendInt(baseAlt);//2 bytes - 20 Total
  sendInt(GPSalt);//2 bytes - 22 Total
  Wire.write(charGPSlat);//1 byte - 23 Total
  sendInt(intGPSlat);//2 bytes - 25 Total
  sendInt(decGPSlat);//2 bytes - 27 Total
  Wire.write(charGPSlon);//1 byte - 28 Total
  sendInt(intGPSlon);//2 bytes - 30 Total
  sendInt(decGPSlon);//2 bytes - 32 Total
  Wire.endTransmission();}

void sendInflightData(){
  Wire.beginTransmission(8); // transmit to slave
  Wire.write(event);//1 byte
  sendInt(intFlightTime);//2 bytes - 3 Total
  Wire.write(decFlightTime);//1 bytes - 4 Total
  sendInt(accelVel);//2 bytes - 6 Total
  sendInt(accelAlt);//2 byte - 8 Total
  sendInt(baroAlt);//2 bytes - 10 Total
  sendInt(angX);//2 bytes - 12 Total
  sendInt(angY);//2 bytes - 14 Total
  sendInt(angZ);//2 bytes - 16 Total
  sendInt(accelX);//2 bytes - 18 Total
  sendInt(signalStrength);//2 bytes - 20 Total
  sendInt(GPSalt);//2 bytes - 22 Total
  Wire.write(charGPSlat);//1 byte - 23 Total
  sendInt(intGPSlat);//2 bytes - 25 Total
  sendInt(decGPSlat);//2 bytes - 27 Total
  Wire.write(charGPSlon);//1 byte - 28 Total
  sendInt(intGPSlon);//2 bytes - 30 Total
  sendInt(decGPSlon);//2 bytes - 32 Total
  Wire.endTransmission();}
      
void sendPostflightData(){
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(event);//1 byte
  sendInt(maxAltitude);//2 bytes - 3 Total
  sendInt(maxVelocity);//2 bytes - 5 Total
  sendInt(maxG);//2 byte - 7 Total
  sendInt(maxGPSalt);//2 bytes - 9 Total
  Wire.write(GPSlock);//1 byte - 10 Total
  sendInt(GPSalt);//2 bytes - 12 Total
  Wire.write(charGPSlat);//1 byte - 13 Total
  sendInt(intGPSlat);//2 bytes - 15 Total
  sendInt(decGPSlat);//2 bytes - 17 Total
  Wire.write(charGPSlon);//1 byte - 18 Total
  sendInt(intGPSlon);//2 bytes - 20 Total
  sendInt(decGPSlon);//2 bytes - 22 Total
  Wire.endTransmission();}

void feedBuffer(){
  charStart = charStop+1;
  charStop = charStart;
  while(dataString[charStop] != ',' && dataString[charStop] != '.' && dataString[charStop] != '\0'){charStop++;}
  memset(localBuffer, 0, sizeof(localBuffer));
  memcpy(localBuffer, dataString+charStart, charStop-charStart);}

void sendInt (int dataInt){
  byte msb;
  byte lsb;
  lsb = lowByte(dataInt);
  msb = highByte(dataInt);
  Wire.write(lsb);
  Wire.write(msb);}

