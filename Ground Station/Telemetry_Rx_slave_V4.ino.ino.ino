//Drives SD card, and LCD
/*Sketch uses 16764 bytes (58%) of program storage space. Maximum is 28672 bytes.
Global variables use 1486 bytes of dynamic memory.*/

#include <LiquidCrystal.h>
#include <SdFat.h>
#include <SPI.h>
#include <Wire.h>
LiquidCrystal lcd(8, 6, 5, 4, 3, 1);
SdFat SD;
File myFile;
char dataString[13] = "FLIGHT01.txt";
boolean signalEst = false;
boolean dataProcessed = false;
boolean preFlightWrite = true;
boolean postFlightWrite = true;
unsigned long lastRX = 0UL;
unsigned long lostSignalTime = 2000000UL;
byte n=1;
byte j=0;
byte lsb;
byte msb;
//---------------------------
//preflight output variables
//---------------------------
char event;
char rocketName[20]="";
int baseAlt=0;
char contCode;
//---------------------------
//inflight output variables
//---------------------------
int intFlightTime=0;
byte decFlightTime=0;
float currentTime;
float prevTime;
float prevAlt;
int signalStrength=0;
int accelVel=0;
int accelAlt=0;
int baroAlt=0;
int angX=0;
int angY=0;
int angZ=0;
int accelX=0;
//---------------------------
//Postflight output variables
//---------------------------
int maxAltitude=0;
int maxVelocity=0;
int maxG=0;
int maxGPSalt=0;
//GPS output variables
char GPSlock;
int GPSalt=0;
char charGPSlat;
int intGPSlat=0;
int decGPSlat=0;
char charGPSlon;
int intGPSlon=0;
int decGPSlon=0;
char lastCharGPSlat;
int lastIntGPSlat;
int lastDecGPSlat;
char lastCharGPSlon;
int lastIntGPSlon;
int lastDecGPSlon;

void setup() {

  //Setup I2C Communication
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  
  // Initialise the LCD and SD card
  SD.begin(10);
  lcd.begin(20,4);

  n=1;
  //Create and open the next file on the SD card
  while (SD.exists(dataString)) {
    n++;
    if(n<10){itoa(n, dataString + 7,10);}
    else{itoa(n, dataString + 6,10);}
    dataString[8]='.';}
  myFile = SD.open(dataString, FILE_WRITE);
  dataString[0]=(char)0;
  //Print header
  myFile.println("Telemetry Rocket Recorder, Rocket Name, Continuity, GPS Lock, Base Altitude, GPS Altitude, GPS Latitude, GPS Longitude");
  myFile.sync();

  //Print to the LCD
  lcd.clear();
  lcd.print("Initializing...");
  delay(1000);
}

void receiveEvent(int howMany) {

  if(!signalEst){signalEst = true;}

  delay(1);
  event = Wire.read();
  
  if(event == '0'){
    
    //Read data
    contCode = Wire.read();//1 byte
    GPSlock = Wire.read();//1 byte
    n=0;
    while (n < 15){
      rocketName[n] = Wire.read();//15 bytes
      n++;}
    baseAlt = Wire.read();//2 bytes
    baseAlt += (Wire.read() << 8);
    GPSalt = Wire.read();//2 bytes
    GPSalt += (Wire.read() << 8);
    charGPSlat = Wire.read();//1 byte
    intGPSlat = Wire.read();//2 bytes
    intGPSlat += (Wire.read() << 8);
    decGPSlat = Wire.read();//2 bytes
    decGPSlat += (Wire.read() << 8);
    charGPSlon = Wire.read();//1 byte
    intGPSlon = Wire.read();//2 bytes
    intGPSlon += (Wire.read() << 8);
    decGPSlon = Wire.read();//2 bytes
    decGPSlon += (Wire.read() << 8);}
    
  else if(event != '8' && event != '9'){
    prevAlt = baroAlt;
    prevTime=currentTime;
    intFlightTime = Wire.read();//2 bytes
    intFlightTime += (Wire.read() << 8);
    decFlightTime = Wire.read();//1 bytes
    accelVel = Wire.read();//2 bytes
    accelVel += (Wire.read() << 8);
    accelAlt = Wire.read();//2 bytes
    accelAlt += (Wire.read() << 8);
    baroAlt = Wire.read();//2 bytes
    baroAlt += (Wire.read() << 8);
    angX = Wire.read();//2 bytes
    angX += (Wire.read() << 8);
    angY = Wire.read();//2 bytesl
    angY += (Wire.read() << 8);
    angZ = Wire.read();//2 bytes 
    angZ += (Wire.read() << 8);
    accelX = Wire.read();//2 bytes
    accelX += (Wire.read() << 8);
    signalStrength = Wire.read();//2 bytes
    signalStrength += (Wire.read() << 8);
    GPSalt = Wire.read();//2 bytes
    GPSalt += (Wire.read() << 8);
    charGPSlat = Wire.read();//1 byte
    intGPSlat = Wire.read();//2 bytes
    intGPSlat += (Wire.read() << 8);
    decGPSlat = Wire.read();//2 bytes
    decGPSlat += (Wire.read() << 8);
    charGPSlon = Wire.read();//1 byte
    intGPSlon = Wire.read();//2 bytes
    intGPSlon += (Wire.read() << 8);
    decGPSlon = Wire.read();//2 bytes
    decGPSlon += (Wire.read() << 8);
    currentTime = (float)(intFlightTime) + (float)decFlightTime / 100;}
      
  if(event == '8' || event == '9' ){
    maxAltitude = Wire.read();//2 bytes
    maxAltitude += (Wire.read() << 8);
    maxVelocity = Wire.read();//2 bytes
    maxVelocity += (Wire.read() << 8);
    maxG = Wire.read();//2 bytes
    maxG += (Wire.read() << 8);
    maxGPSalt = Wire.read();//2 bytes
    maxGPSalt += (Wire.read() << 8);
    GPSlock = Wire.read();//1 byte
    GPSalt = Wire.read();//2 bytes
    GPSalt += (Wire.read() << 8);
    charGPSlat = Wire.read();//1 byte
    intGPSlat = Wire.read();//2 bytes
    intGPSlat += (Wire.read() << 8);
    decGPSlat = Wire.read();//2 bytes
    decGPSlat += (Wire.read() << 8);
    charGPSlon = Wire.read();//1 byte
    intGPSlon = Wire.read();//2 bytes
    intGPSlon += (Wire.read() << 8);
    decGPSlon = Wire.read();//2 bytes
    decGPSlon += (Wire.read() << 8);}

  //capture the GPS data as the last good position
  if(GPSlock && charGPSlat != '\0'){
    lastCharGPSlat = charGPSlat;
    lastIntGPSlat = intGPSlat;
    lastDecGPSlat = decGPSlat;
    lastCharGPSlon = charGPSlon;
    lastIntGPSlon = intGPSlon;
    lastDecGPSlon = decGPSlon;}
  lastRX = micros();
  dataProcessed = true;
}//end receive Event

void loop(){
  byte n=0;
  byte i;
  byte charStart=0;
  byte charStop =0;

  if(dataProcessed){

 //PreFlight Code
 //-----------------------------------------------------
 if (event == '0'){
  //Print to the lcd

  //set lost signal time
  lostSignalTime = 6000000UL;
  
  //Setup Display Text
  //-------------------------------------------------
  //line 0: Display rocket name
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(rocketName);
  //line 1
  lcd.setCursor(0,1);
  if(contCode == '4'){lcd.print("All 4 Pyros Detected");}
  else if (contCode == '6'){lcd.print("Pyro Apogee Only");}
  else if (contCode == '7'){lcd.print("Pyro Mains Only");}
  else if (contCode == '8'){lcd.print("Pyro Mains & Apogee");}
  else if (contCode == '9'){lcd.print("No Pyros Detected!");}
  else{lcd.print("No Cont Pyro ");
    if(contCode == '5'){lcd.print("4");}
    else{lcd.print(contCode);}}
  //line 2
  lcd.setCursor(0,2);
  if(GPSlock == '1'){lcd.print("GPS Lock OK!");}
  else{lcd.print("Awaiting GPS Lock");}
  //line 3
  lcd.setCursor(0,3);
  lcd.print("Base Alt: ");
  lcd.print((long)(baseAlt*3.2808));
  lcd.print(" ft");

  if (preFlightWrite && GPSlock == '1'){
    myFile.print(event);myFile.print(',');
    myFile.write(rocketName,sizeof(rocketName));myFile.print(',');
    myFile.print(contCode);myFile.print(',');
    myFile.print(GPSlock);myFile.print(',');
    myFile.print(baseAlt);myFile.print(',');
    myFile.print(GPSalt);myFile.print(',');
    myFile.print(charGPSlat);myFile.print(intGPSlat);myFile.print('.');
    if(decGPSlat < 10){myFile.print("000");}
    else if(decGPSlat < 100){myFile.print("00");}
    else if(decGPSlat < 1000){myFile.print('0');}
    myFile.print(decGPSlat);myFile.print(',');
    myFile.print(charGPSlon);myFile.print(intGPSlon);myFile.print('.');
    if(decGPSlon < 10){myFile.print("000");}
    else if(decGPSlon < 100){myFile.print("00");}
    else if(decGPSlon < 1000){myFile.print('0');}
    myFile.print(decGPSlon);myFile.println(',');
    myFile.println("event, time, integratedVel, integratedAlt, baroAlt, rotnX, rotnY, rotnZ, acceleration, signal strength, gpsAlt, gpsLat, gpsLon");
    myFile.sync();
    
    preFlightWrite = false;
    }
  }//end Preflight Code
  
  //Inflight Code
  //-----------------------------------------------------------
  else if(event != '8' && event != '9'){

  //set lost signal time
  lostSignalTime = 2000000UL;
  
  //-----------------------------------
  //event - line 0
  //-----------------------------------
  lcd.clear();
  lcd.setCursor(0,0);
  switch (event){
    case '1':
      lcd.print("Liftoff!");
      break;
    case '2':
      lcd.print("Booster Burnout!");
      break;
    case '3':
      lcd.print("Booster Separation!");
      break;
    case '4':
      lcd.print("Firing 2nd Stage!");
      break;
    case '5':
      lcd.print("Apogee Detected!");
      break;
    case '6':
      lcd.print("Separation Detected!");
      break;
    case '7':
      lcd.print("Mains Deployed!");
      break;
    default: 
      lcd.print("Event Error");lcd.print(event);}
   //----------------------------------------
   //barometric altitude - line 1
   //----------------------------------------
   lcd.setCursor(0,1);
   lcd.print("Altitude: ");
   if(event == '1' || accelVel > 300){lcd.print((long)(accelAlt*3.2808));}
   else{lcd.print((long)(baroAlt*3.2808));}
   lcd.print(" ft");
  //----------------------------------------
  //integrated velocity - line 2
  //----------------------------------------
  lcd.setCursor(0,2);
  lcd.print("Speed: ");
  n = atoi(event);
  if(accelVel > 100){lcd.print((long)(accelVel*3.2808));}
  else{lcd.print((int)(((baroAlt-prevAlt)*3.2808)/(currentTime - prevTime)));}
  //else{lcd.print(currentTime);}
  lcd.print(" fps");
  //----------------------------------------
  //signal strength - line 3
  //----------------------------------------
  lcd.setCursor(0,3);
  lcd.print("Signal: ");
  lcd.print(signalStrength);
  lcd.print(" dBm");
  
  //write to SD card
  myFile.print(event);myFile.print(',');
  myFile.print(intFlightTime);myFile.print('.');
  if(decFlightTime < 10){myFile.print('0');}
  myFile.print(decFlightTime);myFile.print(',');
  myFile.print(accelVel);myFile.print(',');
  myFile.print(accelAlt);myFile.print(',');
  myFile.print(baroAlt);myFile.print(',');
  myFile.print(angX);myFile.print(',');
  myFile.print(angY);myFile.print(',');
  myFile.print(angZ);myFile.print(',');
  myFile.print(accelX);myFile.print(',');
  myFile.print(signalStrength);myFile.print(',');
  myFile.print(GPSalt);myFile.print(',');
  myFile.print(charGPSlat);myFile.print(intGPSlat);myFile.print('.');
  if(decGPSlat < 10){myFile.print("000");}
  else if(decGPSlat < 100){myFile.print("00");}
  else if(decGPSlat < 1000){myFile.print('0');}
  myFile.print(decGPSlat);myFile.print(',');
  myFile.print(charGPSlon);myFile.print(intGPSlon);myFile.print('.');
  if(decGPSlon < 10){myFile.print("000");}
  else if(decGPSlon < 100){myFile.print("00");}
  else if(decGPSlon < 1000){myFile.print('0');}
  myFile.print(decGPSlon);myFile.println(',');
  
  myFile.sync();}//end Inflight Code
  
 //Postflight Code
 //-----------------------------------------------------------
 else if(event == '8' || event == '9'){

  //set lost signal time
  lostSignalTime = 11000000UL;
  
  //----------------------------------------
  //barometric altitude - line 0
  //----------------------------------------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Max Alt: ");
  lcd.print((long)(maxAltitude*3.2808));
  lcd.print(" ft");

  //----------------------------------------
  //integrated velocity - line 1
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print("Max Speed: ");
  lcd.print((long)(maxVelocity*3.2808));
  lcd.print(" fps");

  //----------------------------------------
  //GPS Latitude - Line 2
  //----------------------------------------
  lcd.setCursor(0,2);
  lcd.print(intGPSlat);
  lcd.print('.');
  if(decGPSlat < 10){lcd.print("000");}
  else if(decGPSlat < 100){lcd.print("00");}
  else if(decGPSlat < 1000){lcd.print('0');}
  lcd.print(decGPSlat);
  lcd.print(charGPSlat);
  
  //----------------------------------------
  //GPS Latitude - Line 2
  //----------------------------------------
  lcd.setCursor(0,3);
  lcd.print(intGPSlon);
  lcd.print('.');
  if(decGPSlon < 10){lcd.print("000");}
  else if(decGPSlon < 100){lcd.print("00");}
  else if(decGPSlon < 1000){lcd.print('0');}
  lcd.print(decGPSlon);
  lcd.print(charGPSlon);
  
  //write the final data to the SD card
  boolean touchdownWrite = true;
  if(postFlightWrite && GPSlock == '1'){
    myFile.println("Rocket Name,lastEvent,maxAlt,maxVel,maxG,maxGPSalt,GPSlock,GPSalt,gpsLatitude,gpsLongitude");
    myFile.write(rocketName,sizeof(rocketName));myFile.print(',');
    myFile.print(event);myFile.print(',');
    myFile.print(maxAltitude);myFile.print(',');
    myFile.print(maxVelocity);myFile.print(',');
    myFile.print(maxG);myFile.print(',');
    myFile.print(maxGPSalt);myFile.print(',');
    myFile.print(GPSlock);myFile.print(',');
    myFile.print(GPSalt);myFile.print(',');
    myFile.print(charGPSlat);myFile.print(intGPSlat);myFile.print('.');
    if(decGPSlat < 10){myFile.print("000");}
    else if(decGPSlat < 100){myFile.print("00");}
    else if(decGPSlat < 1000){myFile.print('0');}
    myFile.print(decGPSlat);myFile.print(',');
    myFile.print(charGPSlon);myFile.print(intGPSlon);myFile.print('.');
    if(decGPSlon < 10){myFile.print("000");}
    else if(decGPSlon < 100){myFile.print("00");}
    else if(decGPSlon < 1000){myFile.print('0');}
    myFile.print(decGPSlon);myFile.print(',');
    myFile.close();
    postFlightWrite = false;}

  }//end Postflight Code
  dataProcessed = false;
  }//end dataProcessed flag

  //display lost signal status
  if(micros() - lastRX > lostSignalTime && signalEst){
    lcd.clear();
    lastRX = micros();
    //----------------------------------------
    //Message - Line 1    
    //----------------------------------------
    lcd.setCursor(0,0);
    lcd.print("Signal Lost");

    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    lcd.setCursor(0,1);
    lcd.print(lastIntGPSlat);
    lcd.print('.');
    if(lastDecGPSlat < 10){lcd.print("000");}
    else if(lastDecGPSlat < 100){lcd.print("00");}
    else if(lastDecGPSlat < 1000){lcd.print('0');}
    lcd.print(lastDecGPSlat);
    lcd.print(lastCharGPSlat);
  
    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    lcd.setCursor(0,2);
    lcd.print(lastIntGPSlon);
    lcd.print('.');
    if(lastDecGPSlon < 10){lcd.print("000");}
    else if(lastDecGPSlon < 100){lcd.print("00");}
    else if(lastDecGPSlon < 1000){lcd.print('0');}
    lcd.print(lastDecGPSlon);
    lcd.print(lastCharGPSlon);}
}//end void loop
