 void createSustainerFile(){

  fileNum=1;
  //Create and open the next file on the SD card
  while (SD.exists(sustainerFileName)) {
    fileNum++;
    if(fileNum<10){itoa(fileNum, sustainerFileName + 7,10);}
    else{itoa(fileNum, sustainerFileName + 6,10);}
    sustainerFileName[8]='.';}
  //create the sustainer file
  if(debugSerial){Serial.print("FileName: ");Serial.println(sustainerFileName);}
  sustainerFile.open(sustainerFileName, FILE_WRITE);
  sustainerFileCreated = true;
  sustainerFileOpen = true;
  sustainerFileName[0]=(char)0;
  //Print header
  sustainerFile.println(F("Telemetry Rocket Recorder,RocketName,Continuity,GPSlock,BaseAlt,GPSalt,Latitude,Longitude"));
  sustainerFile.sync();
  if(debugSerial){Serial.println("Header Printed, Card Synced");}}
 
 void createBoosterFile(){

  //Create and open the next file on the SD card
  while (SD.exists(boosterFileName)) {
    fileNum++;
    if(fileNum<10){itoa(fileNum, boosterFileName + 7,10);}
    else{itoa(fileNum, boosterFileName + 6,10);}
    boosterFileName[8]='.';}
  //create the booster file
  if(debugSerial){Serial.print("FileName: ");Serial.println(boosterFileName);}
  boosterFile.open(boosterFileName, FILE_WRITE);
  if(fileNum<10){itoa(fileNum, boosterFileName + 8,10);}
    else{itoa(fileNum, boosterFileName + 7,10);}
  boosterFileName[9]='.';
  boosterFileCreated = true;
  boosterFileOpen = true;
  boosterFileName[0]=(char)0;
  //Print header
  boosterFile.println(F("Telemetry Rocket Recorder,RocketName,Continuity,GPSlock,BaseAlt,GPSalt,Latitude,Longitude"));
  boosterFile.sync();
  if(debugSerial){Serial.println("Header Printed, Card Synced");}}
  
 void writePreflightData(){
    strPosn = 0;
    writeIntData((int)0);
    for(byte i=0; i<sizeof(rocketName)-1; i++){dataString[strPosn]=rocketName[i];strPosn++;}
    while(dataString[strPosn-1] == ' ' || dataString[strPosn-1] == '\0'){strPosn--;}
    dataString[strPosn]=',';strPosn++;
    writeIntData(contCode);
    writeIntData(GPSlock);
    writeIntData(baseAlt);
    writeIntData(baseGPSalt);
    dataString[strPosn]=charGPSlat;strPosn++;
    writeFloatData(GPSlatitude,4);
    dataString[strPosn]=charGPSlon;strPosn++;
    writeFloatData(GPSlongitude,4);
    dataString[strPosn] = '\r';strPosn++;
    dataString[strPosn] = '\n';strPosn++;
    dataString[strPosn] = '\0';
    if(parseSustainer){
      sustainerFile.write(dataString,strPosn);
      strPosn=0;
      sustainerFile.println(F("event,time,acceleration,velocity,altitude,spin,offVert,gpsAlt,gpsLat,gpsLon,signalStrength,packetNum"));
      sustainerFile.sync();
      sustainerPreFlightWrite = false;
      if(debugSerial){Serial.println(F("Preflight Data Written"));}}
    if(parseBooster){
      boosterFile.write(dataString,strPosn);
      strPosn=0;
      boosterFile.println(F("event,time,acceleration,velocity,altitude,spin,offVert,gpsAlt,gpsLat,gpsLon,signalStrength,packetNum"));
      boosterFile.sync();
      boosterPreFlightWrite = false;}
    }

void writeInflightData(){
  strPosn = 0;
  writeIntData(event);
  //writeFloatData((float)(sampleTime) * 0.01, 2);
  writeTimeStamp(sampleTime);
  writeFloatData((float)(accel) * 0.029927521, 4);
  writeIntData(velocity);
  writeIntData(Alt);
  writeIntData(spin);
  writeFloatData((float)(offVert) * 0.1, 1);
  writeIntData(GPSalt);
  dataString[strPosn]=charGPSlat;strPosn++;
  writeFloatData(GPSlatitude, 4);
  dataString[strPosn]=charGPSlon;strPosn++;
  writeFloatData(GPSlongitude, 4);
  writeIntData(signalStrength);
  writeIntData(packetnum);
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  if(parseSustainer){sustainerFile.write(dataString, strPosn);}
  if(parseBooster){boosterFile.write(dataString, strPosn);}
  
  //sync the SD card
  if(SDinit && parseSustainer){sustainerFile.sync();}
  if(SDinit && parseBooster){boosterFile.sync();}}
  
void writePostflightData(){
  //write the final data to the SD card
  if(parseSustainer){sustainerFile.println(F("Rocket Name,lastEvent,maxAlt,maxVel,maxG,maxGPSalt,GPSlock,GPSalt,gpsLatitude,gpsLongitude"));}
  if(parseBooster){boosterFile.println(F("Rocket Name,lastEvent,maxAlt,maxVel,maxG,maxGPSalt,GPSlock,GPSalt,gpsLatitude,gpsLongitude"));}
  for(byte i=0; i<sizeof(rocketName)-1; i++){dataString[i]=rocketName[i];strPosn=i;}
  while(dataString[strPosn-1] == ' ' || dataString[strPosn-1] == '\0'){strPosn--;}
  dataString[strPosn]=',';strPosn++;
  writeIntData(event);
  writeIntData(maxAltitude);
  writeIntData(maxVelocity);
  writeFloatData((float)(maxG) * 0.029927521, 4);
  writeIntData(maxGPSalt);
  writeIntData(GPSlock);
  writeIntData(GPSalt);
  dataString[strPosn]=charGPSlat;strPosn++;
  writeFloatData(GPSlatitude, 4);
  dataString[strPosn]=charGPSlon;strPosn++;
  writeFloatData(GPSlongitude, 4);
  if(parseSustainer){
    sustainerFile.write(dataString,strPosn);
    strPosn=0;
    sustainerFile.close();
    sustainerFileOpen = false;
    sustainerPostFlightWrite = false;}
  if(parseBooster){
    boosterFile.write(dataString,strPosn);
    strPosn=0;
    boosterFile.close();
    boosterFileOpen = false;
    boosterPostFlightWrite = false;}
  if(debugSerial){Serial.println("Post Flight Data Written");}}

void writeIntData(int dataValue) {
  itoa(dataValue, dataString + strPosn, 10);
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
  itoa((int)dataValue, dataString + strPosn, 10);
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn]='.'; strPosn++;
  
  //fractional portion
  partial = dataValue - (int)(dataValue);
  fracInt = (long)(partial*pow(10,decimals));
  if(fracInt == 0){dataString[strPosn] = '0'; strPosn++; dataString[strPosn] = ','; strPosn++;}
  else{
    decimals--;
    while(fracInt < pow(10, decimals)){dataString[strPosn]='0';strPosn++;decimals--;}
    itoa(fracInt, dataString + strPosn, 10);
    while(dataString[strPosn]!= '\0'){strPosn++;}
    dataString[strPosn]=','; strPosn++;}}

void writeTimeStamp(int dataValue){

  writeIntData(dataValue);  

  //add 2 zeroes
  if(dataValue < 10){
    dataString[strPosn+2] = dataString[strPosn-1];//comma
    dataString[strPosn+1] = dataString[strPosn-2];
    dataString[strPosn] = '0';
    dataString[strPosn-1]='.';
    dataString[strPosn-2]='0';
    strPosn+=3;}

  //add 1 zero
  else if(dataValue < 100){
    dataString[strPosn+1] = dataString[strPosn-1];
    dataString[strPosn] = dataString[strPosn-2];
    dataString[strPosn-1]=dataString[strPosn-3];
    dataString[strPosn-2]='.';
    dataString[strPosn-3]='0';
    strPosn+=2;}
  
  //parse as normal
  else{
    dataString[strPosn]=dataString[strPosn-1];
    dataString[strPosn-1] = dataString[strPosn-2];
    dataString[strPosn-2] = dataString[strPosn-3];
    dataString[strPosn-3] = '.';
    strPosn++;}}

void updateStrPosn(){
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = ',';
  strPosn++;}
