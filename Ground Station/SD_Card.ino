 void writePreflightData(){
    strPosn = 0;
    writeIntData(event);
    for(byte i=0; i<sizeof(rocketName)-1; i++){dataString[strPosn]=rocketName[i];strPosn++;}dataString[strPosn]=',';strPosn++;
    writeIntData(contCode);
    writeIntData(GPSlock);
    writeIntData(baseAlt);
    writeIntData(baseGPSalt);
    dataString[strPosn]=charGPSlat;strPosn++;
    writeFloatData(GPSlatitude,2,4);
    dataString[strPosn]=charGPSlon;strPosn++;
    writeFloatData(GPSlongitude,2,4);
    dataString[strPosn] = '\r';strPosn++;
    dataString[strPosn] = '\n';strPosn++;
    dataString[strPosn] = '\0';
    myFile.write(dataString,strPosn);
    Serial.println(dataString);
    strPosn=0;
    myFile.println(F("event,time,acceleration,velocity,altitude,spin,offVert,gpsAlt,gpsLat,gpsLon,signalStrength,packetNum"));
    myFile.sync();
    preFlightWrite = false;}

void writeInflightData(){
  strPosn = 0;
  writeIntData(event);
  writeFloatData((float)(sampleTime) * 0.01,2,2);
  writeFloatData((float)(accelX) * 0.029927521,2,4);
  writeIntData(velocity);
  writeIntData(Alt);
  writeIntData(spin);
  writeFloatData((float)(offVert) * 0.1,2,1);
  writeIntData(GPSalt);
  dataString[strPosn]=charGPSlat;strPosn++;
  writeFloatData(GPSlatitude,2,4);
  dataString[strPosn]=charGPSlon;strPosn++;
  writeFloatData(GPSlongitude,2,4);
  writeIntData(signalStrength);
  writeIntData(packetnum);
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  myFile.write(dataString, strPosn);
  Serial.println(dataString);}
  
void writePostflightData(){
  //write the final data to the SD card
  myFile.println(F("Rocket Name,lastEvent,maxAlt,maxVel,maxG,maxGPSalt,GPSlock,GPSalt,gpsLatitude,gpsLongitude"));
  strPosn=0;
  for(byte i=0; i<sizeof(rocketName)-1; i++){dataString[i]=rocketName[i];strPosn=i;}dataString[strPosn]=',';strPosn++;
  writeIntData(event);
  writeIntData(maxAltitude);
  writeIntData(maxVelocity);
  writeFloatData((float)(maxG) * 0.00305176,2,4);
  writeIntData(maxGPSalt);
  writeIntData(GPSlock);
  writeIntData(GPSalt);
  dataString[strPosn]=charGPSlat;strPosn++;
  writeFloatData(GPSlatitude,2,4);
  dataString[strPosn]=charGPSlon;strPosn++;
  writeFloatData(GPSlongitude,2,4);
  myFile.write(dataString,strPosn);
  Serial.println(dataString);
  strPosn=0;
  myFile.close();
  fileOpen = false;
  postFlightWrite = false;}

void writeIntData(int dataValue) {
  itoa(dataValue, dataString + strPosn, 10);
  updateStrPosn();}//end void

void writeFloatData(float dataValue, byte dataLen, byte decimals) {
  dtostrf(dataValue, dataLen, decimals, dataString + strPosn);
  updateStrPosn();}//end void

void updateStrPosn(){
  while(dataString[strPosn]!= '\0'){strPosn++;}
  dataString[strPosn] = ',';
  strPosn++;}
