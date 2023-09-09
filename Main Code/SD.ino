//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//beginSD(): starts the SD card 
//restartSD(): starts the SD card after a power loss
//parseEEPROMsettingsSD(): if an EEPROM settings file exists, read it and update EEPROM
//createNextFileSD(): finds the last filename in the list and opens a new file, writes header
//reOpenSD(): reopens the last file on the SD card
//syncSD(): syncs the SD card in case of a powerlossfire
//readFlightSettingsSD(): reads the user settings from the SD card
//writeSDflightData(): primary routine to capture the flight data in ASCII text to the SD card
//writeSDfooter(): writes the post flight footer to the SD card once touchdown or timeout is detected
//writeNMEA(): writes the NMEA GPS data to a separate file
//--------------
//Functions:
//--------------
//writeIntData(): turns int to a char array
//writeFloatData(): turns floats to a char array, limit of 4 decimals (faster than writeFloatData2)
//writeFloatData2(): turns floats to a char array with no decimal limit
//writeLongData(): turns long to a char array
//writeULongData(): turns unsigned long to a char array
//writeBoolData(): turns a boolean to a char array
//updateStrPosn(): helper function to keep track of the position within the char array
//parseNextVariable(): places data from the SD card into the dataString char array
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//04 AUG 21: edited to account for variable gain settings
//15 AUG 21: variable gain settings removed
//18 NOV 21: added all flight settings to the footer
//03 JAN 22: added hardware compatibility macros for Teensy 3.2, 4.0, 4.1, and moved in functions from the main file
//21 JUN 22: minor bug fixes and tweaks
//27 NOV 22: adds an optional GPS output file containing the NMEA strings
//---------------------------------
#if defined (__MK66FX1M0__) || defined (__MK64FX512__)
  //Teensy 3.5 and 3.6
  #include <SdFat.h>
   
  //SDIO Setup: v2.X now works after fixing the RadioHead ISR problem
  SdFs SD;
  FsFile outputFile;
  FsFile settingsFile;
  FsFile gpsFile;
  
#else
  //Teensy 4.X and 3.2
  #include <SD.h>
  
  File outputFile;
  File settingsFile;
  File gpsFile;
#endif

//GPS log variable
char GPSlog[1024];

void beginSD(){

  //Built-in SDIO on Teensy3.5 or 3.6
  #if defined (__MK66FX1M0__) || defined (__MK64FX512__) 
  
    //Use SDFat library with Built-in SDIO
    if(!SD.begin(SdioConfig(FIFO_SDIO))){Serial.println(F("SD card failed!"));}
    else{Serial.println(F("SD Card OK!"));}

  //Teensy4.1 or Teensy4.0
  #elif defined (__IMXRT1062__)

    //Use the TeensyDuino modification of the SDFat library for SDIO
    if(!SD.begin(BUILTIN_SDCARD)){Serial.println(F("SD card failed!"));}
    else{Serial.println(F("SD Card OK!"));}

    //Uncomment this section if using the Teensy4.0 with the SPI bus
    /*if(pins.SD_CS != pins.nullFire){
      if(!SD.begin(pins.SD_CS)){Serial.println(F("SD card failed!"));}
      else{Serial.println(F("SD Card OK!"));}}*/
     
  //Teensy 3.2
  #elif defined (__MK20DX256__)
    
    //Use the SDFat library
    if(pins.SD_CS != pins.nullFire){
      if(!SD.begin(pins.SD_CS)){Serial.println(F("SD card failed!"));}
      else{Serial.println(F("SD Card OK!"));}}

  #endif  
}

void restartSD(){
  
  //Teensy3.5 or 3.6 with built-in SDIO
  #if defined (__MK66FX1M0__) || defined (__MK64FX512__)

    SD.begin(SdioConfig(FIFO_SDIO));//SDFat V2.1 still not working

  //Teensy 3.2, 4.0, and 4.1
  #else

    //if the SD Chip Select pin is set to the nullpin, then it must be a Teensy4.1 or Teensy4.0 using a builtin SDIO port
    if(pins.SD_CS == pins.nullFire){SD.begin();}//BUILTIN_SDCARD
    
    //if the SD Chip Select is not set to the nullpin, then it must be a Teensy3.2 or Teensy4.0 so use the chip select pin
    if(pins.SD_CS != pins.nullFire){SD.begin(pins.SD_CS);}

  #endif  
  }

void parseEEPROMsettingsSD(){
  
  //If an EEPROM settings file exists, open it and copy the values into EEPROM
  byte kk;
  int8_t ii;
  if(SD.exists("EEPROMsettings.txt")){
    Serial.println(F("EEPROM file found!  Writing initial EEPROM Settings..."));
    settingsFile = SD.open("EEPROMsettings.txt", FILE_READ);
    //-----------------------------------------------------------------
    //read the device ID codes
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.accelID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.magID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.gyroID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.highGID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.baroID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.GPSID, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.sdID, kk);
    //-----------------------------------------------------------------
    //read the device buses
    //-----------------------------------------------------------------
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.accelBusType, (char)dataString[0]); EEPROM.update(eeprom.accelBusNum, ii);//accel bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.magBusType, (char)dataString[0]); EEPROM.update(eeprom.magBusNum, ii);//mag bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.gyroBusType, (char)dataString[0]); EEPROM.update(eeprom.gyroBusNum, ii);//gyro bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.highGBusType, (char)dataString[0]); EEPROM.update(eeprom.highGBusNum, ii);//highG bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.baroBusType, (char)dataString[0]); EEPROM.update(eeprom.baroBusNum, ii);//baro bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.radioBusType, (char)dataString[0]); EEPROM.update(eeprom.radioBusNum, ii);//radio bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.gpsBusType, (char)dataString[0]); EEPROM.update(eeprom.gpsBusNum, ii);//GPS bus data
    parseNextVariable(false); ii = (uint8_t)dataString[1] - 48;Serial.print((char)dataString[0]);Serial.print(ii);Serial.print(F(", "));
    EEPROM.update(eeprom.sdBusType, (char)dataString[0]); EEPROM.update(eeprom.sdBusNum, ii);//SD bus data
    //-----------------------------------------------------------------
    //read the device orientation to the mother board
    //-----------------------------------------------------------------
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.accel2boardXsign, ii); EEPROM.update(eeprom.accel2boardXaxis, (char)dataString[1]);//accelX orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.accel2boardYsign, ii); EEPROM.update(eeprom.accel2boardYaxis, (char)dataString[1]);//accelY orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.accel2boardZsign, ii); EEPROM.update(eeprom.accel2boardZaxis, (char)dataString[1]);//accelZ orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.gyro2boardXsign, ii); EEPROM.update(eeprom.gyro2boardXaxis, (char)dataString[1]);//gyroX orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.gyro2boardYsign, ii); EEPROM.update(eeprom.gyro2boardYaxis, (char)dataString[1]);//gyroY orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.gyro2boardZsign, ii); EEPROM.update(eeprom.gyro2boardZaxis, (char)dataString[1]);//gyroZ orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.highG2boardXsign, ii); EEPROM.update(eeprom.highG2boardXaxis, (char)dataString[1]);//highGX orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.highG2boardYsign, ii); EEPROM.update(eeprom.highG2boardYaxis, (char)dataString[1]);//highGY orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.highG2boardZsign, ii); EEPROM.update(eeprom.highG2boardZaxis, (char)dataString[1]);//highGZ orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.mag2boardXsign, ii); EEPROM.update(eeprom.mag2boardXaxis, (char)dataString[1]);//magX orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.mag2boardYsign, ii); EEPROM.update(eeprom.mag2boardYaxis, (char)dataString[1]);//magY orientation
    parseNextVariable(false); ii = (dataString[0] == '-') ? -1 : 1;Serial.print((char)dataString[0]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.mag2boardZsign, ii); EEPROM.update(eeprom.mag2boardZaxis, (char)dataString[1]);//magZ orientation
    //-----------------------------------------------------------------
    //read the GPIO control pins
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro1ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro1FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro2ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro2FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro3ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro3FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro4ContPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.pyro4FirePin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.nullPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.beepPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.battReadPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.testModeGndPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.testModeRdPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioIRQpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioRstPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioEnPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.radioCSpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.accelCSpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.gyroCSpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.highGcsPin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.magCSpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.baroCSpin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.sdCSpin, kk);
    //-----------------------------------------------------------------
    //read the servo control pins
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo1pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo2pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo3pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo4pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo5pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo6pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo7pin, kk);
    kk = (byte)parseNextVariable(true);Serial.print(kk);Serial.print(F(", "));EEPROM.update(eeprom.servo8pin, kk);
    //-----------------------------------------------------------------
    //read the ham radio call sign
    //-----------------------------------------------------------------
    parseNextVariable(false);
    for(byte i = 0; i < 6; i++){EEPROM.update(eeprom.callSign+i, (char)dataString[i]);}
    Serial.print(dataString);Serial.print(F(", "));
    //-----------------------------------------------------------------
    //read whether or not to put the unit into magnetometer calibration mode
    //-----------------------------------------------------------------
    magCalibrateMode = (byte)parseNextVariable(true);//Sets Magnetometer Calibration Mode, not stored in EEPROM
    //-----------------------------------------------------------------
    //read hardware ID into EEPROM
    //-----------------------------------------------------------------
    parseNextVariable(false);
    EEPROM.update(eeprom.HWversion, (char)dataString[0]);Serial.print((char)dataString[0]);Serial.print(F(", "));
    EEPROM.update(eeprom.HWsubVersion, (char)dataString[1]);Serial.print((char)dataString[1]);Serial.print(F(", "));
    EEPROM.update(eeprom.HWunitNum, (char)dataString[2]);Serial.print((char)dataString[2]);Serial.print(F("..."));
    settingsFile.close();
    SD.remove("EEPROMsettings.txt");
    Serial.println(F("Complete!"));}
}

void createNextFileSD(){

  char fileName[20] = "FLIGHT01.txt";
  
  if(settings.testMode){Serial.print(F("Creating new SD card file: FLIGHT"));}
  n=0;
  while (SD.exists(fileName)) {
    n++;
    if(n<10){itoa(n, fileName + 7,10);}
    else{itoa(n, fileName + 6,10);}
    fileName[8]='.';}
  outputFile = SD.open(fileName, FILE_WRITE);
  //Print header
  outputFile.print(settings.rocketName);
  outputFile.print(F(" Code V"));
  outputFile.print(codeVersion);
  outputFile.print(F(",accelX,accelY,accelZ,gyroX,gyroY,gyroZ,highGx,highGy,highGz,"));
  if(settings.testMode){outputFile.print(F("cyclesBtwn,writeFlags,sampleTime,"));}
  outputFile.println(F("smoothHighGz,rollZ,yawY,pitchX,offVert,intVel,intAlt,fusionVel,fusionAlt,fltEvents,radioCode,pyroCont,pyroFire,pyroPin,baroAlt,altMoveAvg,baroVel,baroPress,baroTemp,battVolt,magX,magY,magZ,gnssLat,gnssLon,gnssSpeed,gnssAlt,gnssAngle,gnssSatellites,radioPacketNum"));
  outputFile.flush();

  //create GPS file
  if(settings.GPSlog){
    fileName[0] = 'G';
    fileName[1] = 'P';
    fileName[2] = 'S';
    fileName[3] = 'l';
    fileName[4] = 'o';
    fileName[5] = 'g';
    fileName[8] = '.';
    fileName[9] = 'n';
    fileName[10]= 'm';
    fileName[11]= 'e';
    fileName[12]= 'a';
    fileName[13]= '\0';
    gpsFile = SD.open(fileName, FILE_WRITE);
    gpsFile.println("GPS Log");}

  if(settings.testMode){
    if(n<10){Serial.print('0');Serial.println(n);}
    else{Serial.println(n);}}
}//end createNextFileSD

void reOpenSD(){
  n = 1;
  char fileName[20] = "FLIGHT01.txt";
  while (SD.exists(fileName)){
    n++;
    if(n<10){itoa(n, fileName + 7,10);}
    else{itoa(n, fileName + 6,10);}
    fileName[8]='.';}
  n--;
  if(n<10){itoa(n, fileName + 7,10);}
  else{itoa(n, fileName + 6,10);}
  fileName[8]='.';
  //outputFile = SD.open(dataString, FILE_WRITE | O_AT_END);
  outputFile = SD.open(fileName, FILE_WRITE);}

void syncSD(){
  outputFile.flush();
  if(settings.GPSlog){gpsFile.flush();}}

void readFlightSettingsSD(){
  if(settings.testMode){Serial.println(F("Reading User Settings from SD Card"));}
  //Open the settings file
  settingsFile = SD.open("Settings.txt", FILE_READ);

  //Read in the user defined variables
  parseNextVariable(false);n=0;
  //Rocket name
  while (dataString[n]!='\0'){settings.rocketName[n] = dataString[n];n++;}
  settings.rocketName[n]='\0';n=0;
  parseNextVariable(false); settings.fltProfile = dataString[0];
  parseNextVariable(false); settings.units = dataString[0]; if(settings.units == 'M'){unitConvert = 1.0F;}
  parseNextVariable(false); settings.reportStyle = dataString[0];
  settings.setupTime = (unsigned long)(parseNextVariable(true)*1000UL);
  settings.mainDeployAlt = (float)(parseNextVariable(true)/unitConvert);
  settings.apogeeDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.rcdTime = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.silentMode = (boolean)(parseNextVariable(true));
  settings.magSwitchEnable = (boolean)(parseNextVariable(true));
  settings.inflightRecover = (byte)(parseNextVariable(true));
  settings.GPSlog = (boolean)(parseNextVariable(true));
  if(settings.GPSlog){Serial.println("GPS Log Active");}
  settings.fireTime = (unsigned long) (parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.pyro4Func = dataString[0];
  parseNextVariable(false); settings.pyro3Func = dataString[0];
  parseNextVariable(false); settings.pyro2Func = dataString[0];
  parseNextVariable(false); settings.pyro1Func = dataString[0];
  settings.TXenable = (boolean)parseNextVariable(true);
  settings.TXpwr = (byte)(parseNextVariable(true));
  settings.TXfreq = (float)(parseNextVariable(true));
  settings.FHSS = (boolean)parseNextVariable(true);
  settings.boosterSeparationDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.sustainerFireDelay = (unsigned long)(parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.airStart1Event = dataString[0];
  settings.airStart1Delay = (unsigned long)(parseNextVariable(true)*1000000UL);
  parseNextVariable(false); settings.airStart2Event = dataString[0];
  settings.airStart2Delay = (unsigned long)(parseNextVariable(true)*1000000UL);
  settings.altThreshold = (int)(parseNextVariable(true)/unitConvert);
  settings.maxAngle = (int)parseNextVariable(true)*10;
  settings.stableRotn = (boolean)parseNextVariable(true);
  settings.stableVert = (boolean)parseNextVariable(true);
  settings.flyBack = (boolean)parseNextVariable(true);
  settings.serialDebug = (byte)parseNextVariable(true);
  //close the settings file
  settingsFile.close();
}//end readFlightSettingsSD

void writeSDflightData(){
  //Timestamp
  writeULongData(fltTime.timeCurrent);
  //Accel Data
  writeIntData(accel.x);
  writeIntData(accel.y);
  writeIntData(accel.z);
  //Gyro Data
  writeIntData(gyro.x);
  writeIntData(gyro.y);
  writeIntData(gyro.z);
  //HighG Accel Data
  writeIntData(highG.x);
  writeIntData(highG.y);
  writeIntData(highG.z);
  //debug output for write flags
  if(settings.testMode){
    writeIntData(cyclesBtwn);
    writeBoolData(accel.newSamp);
    writeBoolData(gyro.newSamp);
    writeBoolData(highG.newSamp);
    writeBoolData(mag.newSamp);
    writeBoolData(baro.newSamp);
    writeBoolData(baro.newTemp);
    writeBoolData(SDradioTX);
    writeBoolData(gnss.SDwrite);
    writeBoolData(writeVolt);
    dataString[strPosn] = cs;strPosn++;
    writeULongData(sampleTime);
    cyclesBtwn = 0;}
  //reset write flags
  accel.newSamp = gyro.newSamp = highG.newSamp = false;
  //Smoothed high-G data
  writeIntData((int16_t)highGsmooth);
  //Integrated Rotation Values
  writeLongData(rollZ);
  writeIntData(yawY);
  writeIntData(pitchX);
  writeIntData(offVert);
  //Integrated Speed and Altitude
  writeFloatData(accelVel, 2);
  writeFloatData(accelAlt, 2);
  //Sensor Fusion Speed and Altitude
  writeFloatData(fusionVel, 2);
  writeFloatData(fusionAlt, 2);
  //Flight Event Flags
  writeBoolData(events.liftoff);
  writeBoolData(events.boosterBurnout);
  writeBoolData(events.boosterBurnoutCheck);
  if(settings.fltProfile == '2'){
    writeBoolData(events.boosterSeparation);
    writeBoolData(events.sustainerFireCheck);
    writeBoolData(events.sustainerFire);
    writeBoolData(events.sustainerIgnition);
    writeBoolData(events.sustainerBurnout);}
  if(settings.fltProfile == 'A'){
    writeBoolData(events.airStart1Check);
    writeBoolData(events.airStart1Fire);
    writeBoolData(events.airStart1Ignition);
    writeBoolData(events.airStart1Burnout);
    writeBoolData(events.airStart2Check);
    writeBoolData(events.airStart2Fire);
    writeBoolData(events.airStart2Ignition);
    writeBoolData(events.airStart2Burnout);}
  writeBoolData(events.apogee);
  writeBoolData(events.apogeeFire);
  writeBoolData(events.mainDeploy);
  writeBoolData(events.touchdown);
  writeBoolData(events.timeOut);
  dataString[strPosn] = cs;strPosn++;
  writeIntData(radio.event);
  //Continuity Data
  dataString[strPosn] = 'C';strPosn++;
  writeBoolData(pyro1.contStatus);
  writeBoolData(pyro2.contStatus);
  writeBoolData(pyro3.contStatus);
  writeBoolData(pyro4.contStatus);
  dataString[strPosn] = cs;strPosn++;
  //Pyro Firing Data
  dataString[strPosn] = 'F';strPosn++;
  writeBoolData(pyro1.fireStatus);
  writeBoolData(pyro2.fireStatus);
  writeBoolData(pyro3.fireStatus);
  writeBoolData(pyro4.fireStatus);
  dataString[strPosn] = cs;strPosn++;
  writeIntData(pins.firePin);
  //Barometer
  if (baro.newSamp) {
    writeFloatData(baro.Alt, 2);
    writeFloatData(baro.smoothAlt, 2);
    writeFloatData(baro.Vel, 2);
    writeFloatData(baro.pressure, 2);
    baro.newSamp=false;}
  else{for(byte i = 0; i < 4; i++){dataString[strPosn]=cs;strPosn++;}}
  //Temperature
  if(baro.newTemp){writeFloatData(baro.temperature, 2);baro.newTemp=false;}
  else{dataString[strPosn]=cs;strPosn++;}
  //Battery Voltage
  if(writeVolt){writeFloatData(voltage, 2);writeVolt = false;}
  else{dataString[strPosn]=cs;strPosn++;}
  //Magnetometer Data
  if (mag.newSamp){
    writeIntData(mag.x);
    writeIntData(mag.y);
    writeIntData(mag.z);
    mag.newSamp = false;}
  else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
  //GPS Data
  if(gnss.SDwrite){
    writeFloatData(gnss.latitude,6);
    writeFloatData(gnss.longitude,6);
    writeFloatData((float)GPS.speed.mph(),2);
    writeFloatData(gnss.alt,2);
    writeFloatData((float)GPS.course.deg(),2);
    writeIntData(radio.satNum);
    gnss.SDwrite=false;}
  else{
    dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;
    dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
  //update the radio packet number
  if (SDradioTX){writeIntData(radio.packetnum);SDradioTX = false;}
  else{dataString[strPosn]=cs;strPosn++;}
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  // write the string to file and capture the time it takes to write to the SD card
  if(settings.testMode){writeStart = micros();}
  outputFile.write(dataString, strPosn);
  if(settings.testMode){writeTime = micros() - writeStart;
    if(writeTime > maxWriteTime){maxWriteTime = writeTime;}
    if(writeTime > writeThreshold){writeThreshCount++;}}
  strPosn = 0;
}//end write SD data

void writeSDfooter(){
  //Print the initial conditions
  outputFile.println(F("Max Baro Alt,Max GPS Alt,Max Speed,Max Gs,baseAlt,padTime,initial Y ang,initial X ang,accelX0,accelY0,accelZ0,highGz0,magX0,magY0,magZ0,gyroBiasX,gyroBiasY,gyroBiasZ,accelBiasX,accelBiasY,accelBiasZ,highGbiasX,highGbiasY,highGbiasZ,magBiasX,magBiasY,magBiasZ,baroPressureOffset,baroTempOffset"));
  writeULongData((unsigned long)(baro.maxAlt*unitConvert));
  writeULongData((unsigned long)(gnss.maxAlt*unitConvert));
  writeULongData((unsigned long)(maxVelocity*unitConvert));
  writeFloatData(maxG/9.80665, 2);
  writeFloatData(baro.baseAlt, 2);
  writeFloatData(((float)fltTime.padTime/(float)1000000), 2);
  writeFloatData(yawY0, 2);
  writeFloatData(pitchX0, 2);
  writeIntData(accel.x0);
  writeIntData(accel.y0);
  writeIntData(accel.z0);
  writeIntData(highG.z0);
  writeIntData(mag.x0);
  writeIntData(mag.y0);
  writeIntData(mag.z0);
  writeIntData(gyro.biasX);
  writeIntData(gyro.biasY);
  writeIntData(gyro.biasZ);
  writeIntData(accel.biasX);
  writeIntData(accel.biasY);
  writeIntData(accel.biasZ);
  writeIntData(highG.biasX);
  writeIntData(highG.biasY);
  writeIntData(highG.biasZ);
  writeIntData(mag.biasX);
  writeIntData(mag.biasY);
  writeIntData(mag.biasZ);
  writeFloatData(baro.pressOffset,2);
  writeFloatData(baro.tempOffset,1);
  //carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);
  
  //write out the launch time and locations
  strPosn = 0;
  outputFile.println(F("launch date, UTC time, launch altitude, launch latitude, launch longitude"));
  //Write out the GPS liftoff date
  outputFile.print(gnss.liftoff.day);outputFile.print("/");outputFile.print(gnss.liftoff.month);outputFile.print("/");outputFile.print(gnss.liftoff.year);outputFile.print(",");
  //Write out the GPS liftoff time
  outputFile.print(gnss.liftoff.hour);outputFile.print(":");outputFile.print(gnss.liftoff.minute);outputFile.print(":");outputFile.print((int)gnss.liftoff.second);outputFile.print(",");
  //Write out GPS launch location
  writeFloatData(gnss.baseAlt,2);
  writeFloatData2(gnss.liftoff.latitude,6);
  writeFloatData2(gnss.liftoff.longitude,6);
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);

  //Write out the GPS landing location
  strPosn = 0;
  outputFile.println(F("landing date, UTC time, landing altitude, landing latitude, landing longitude"));
  //Write out the GPS landing date
  outputFile.print(gnss.liftoff.day);outputFile.print("/");outputFile.print(gnss.liftoff.month);outputFile.print("/");outputFile.print(gnss.liftoff.year);outputFile.print(",");
  //Write out the GPS landing time
  outputFile.print(gnss.touchdown.hour);outputFile.print(":");outputFile.print(gnss.touchdown.minute);outputFile.print(":");outputFile.print((int)gnss.touchdown.second);outputFile.print(",");
  writeFloatData(gnss.touchdown.alt,2);
  writeFloatData2(gnss.touchdown.latitude,6);
  writeFloatData2(gnss.touchdown.longitude,6);
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r'; strPosn++;
  dataString[strPosn] = '\n'; strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);
  
  //write out the settings for the flight
  outputFile.print(F("Rocket Name, callsign, HWid, flightProfile, units, inflightRecover, pyro4func, pyro3func, pyro2func, pyro1func, apogeeDelay, mainDeployAlt, setupTime, rcdTime, fireTime, TXenable, TXpwr, TXfreq, FHSS, seaLevelPressure"));
  if(settings.fltProfile == '2'){outputFile.println("ignitionDelay, sepDelay, altThreshold, maxAng");}
  else if(settings.fltProfile == 'A'){outputFile.println("aistart1event, airstart1delay, airstart2event, airstart2delay, altThreshold, maxAng");}
  else{outputFile.println(' ');}
  outputFile.print(settings.rocketName);outputFile.print(cs);
  outputFile.print(settings.callSign);outputFile.print(cs);
  outputFile.print(settings.HWid);outputFile.print(cs);
  outputFile.print((char)settings.fltProfile);outputFile.print(cs);
  outputFile.print(settings.units);outputFile.print(cs);
  outputFile.print(settings.inflightRecover);outputFile.print(cs);
  outputFile.print(settings.pyro4Func);outputFile.print(cs);
  outputFile.print(settings.pyro3Func);outputFile.print(cs);
  outputFile.print(settings.pyro2Func);outputFile.print(cs);
  outputFile.print(settings.pyro1Func);outputFile.print(cs);
  strPosn = 0;
  writeFloatData(((float)settings.apogeeDelay)*mlnth,1);
  writeFloatData(((float)settings.mainDeployAlt)*unitConvert,0);
  writeFloatData(((float)settings.setupTime)/1000,0);//issue
  writeFloatData(settings.rcdTime*mlnth,0);
  writeFloatData(settings.fireTime*mlnth,1);
  writeBoolData(settings.TXenable);dataString[strPosn]=cs;strPosn++;
  writeIntData(settings.TXpwr);
  writeFloatData(settings.TXfreq, 3);
  writeBoolData(settings.FHSS);dataString[strPosn]=cs;strPosn++;
  writeFloatData(baro.seaLevelPressure,2);      
  if(settings.fltProfile == '2'){
    writeFloatData(settings.sustainerFireDelay*mlnth,1);
    writeFloatData(settings.boosterSeparationDelay*mlnth,1);
    writeIntData((int)(10*int(settings.altThreshold*(unitConvert/10))));
    writeIntData(settings.maxAngle/10);}
  if(settings.fltProfile == 'A'){
    dataString[strPosn] = settings.airStart1Event;strPosn++;
    writeFloatData(settings.airStart1Delay*mlnth, 1);
    dataString[strPosn] = settings.airStart2Event;strPosn++;
    writeFloatData(settings.airStart2Delay*mlnth, 1);
    writeIntData((int)(10*int(settings.altThreshold*(unitConvert/10))));
    writeIntData(settings.maxAngle/10);}
  //carriage return, newline, and null value
  dataString[strPosn] = '\r';
  strPosn++;
  dataString[strPosn] = '\n';
  strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);    
  strPosn=0;
  
  //close the file
  outputFile.close();
  if(settings.GPSlog){gpsFile.close();}
  fileClose = true;}//end SD footer

void writeIntData(int dataValue) {
  itoa(dataValue, dataString + strPosn, base);
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = cs;
  strPosn++;}//end void

void writeULongData(unsigned long dataValue){
  ultoa(dataValue, dataString + strPosn, base);
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = cs;
  strPosn++;}//end void

void writeLongData(long dataValue){
  ltoa(dataValue, dataString + strPosn, base);
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
  fracInt = (long)(partial*powf(10,decimals));
  if(fracInt == 0){dataString[strPosn] = '0'; strPosn++; dataString[strPosn] = cs; strPosn++;}
  else{
    decimals--;
    while(fracInt < powf(10, decimals)){dataString[strPosn]='0';strPosn++;decimals--;}
    ltoa(fracInt, dataString + strPosn, base);
    while(dataString[strPosn]!= '\0'){strPosn++;}
    dataString[strPosn]=','; strPosn++;}}
    
void writeBoolData(boolean dataBool) { 
  dataString[strPosn] = (dataBool) ? '1' : '0';
  strPosn ++;}//end void

void updateStrPosn(){
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = cs;
  strPosn++;}

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

void updateGPSlogSD(char c){

  static uint16_t logPosn = 0;

  if(logPosn < sizeof(GPSlog)){
    GPSlog[logPosn] = c;
    logPosn++;}
  else{
    gpsFile.write(GPSlog, logPosn);
    logPosn = 0;
    GPSlog[logPosn] = c;}}