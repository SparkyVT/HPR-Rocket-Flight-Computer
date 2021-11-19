//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//writeSDflightData(): primary routine to capture the flight data in ASCII text to the SD card
//writeSDfooter(): writes the post flight footer to the SD card once touchdown or timeout is detected
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
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//04 AUG 21: edited to account for variable gain settings
//15 AUG 21: variable gain settings removed
//18 NOV 21: added all flight settings to the footer
//---------------------------------

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
  if(settings.highG3axis){writeIntData(highG.x);writeIntData(highG.y);}
  writeIntData(highG.z);
  writeIntData(highGsmooth);
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
  writeIntData(firePin);
  //Barometer
  if (newBaro) {
    writeFloatData(Alt, 2);
    writeFloatData(altMoveAvg, 2);
    writeFloatData(baroVel, 2);
    writeFloatData(pressure, 2);
    newBaro=false;}
  else{for(byte i = 0; i < 4; i++){dataString[strPosn]=cs;strPosn++;}}
  //Temperature
  if(newTemp){writeFloatData(temperature, 2);newTemp=false;}
  else{dataString[strPosn]=cs;strPosn++;}
  //Battery Voltage
  if(writeVolt){writeFloatData(voltage, 2);writeVolt = false;}
  else{dataString[strPosn]=cs;strPosn++;}
  //Magnetometer Data
  if (magCounter == 0){
    writeIntData(mag.x);
    writeIntData(mag.y);
    writeIntData(mag.z);}
  else{dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
  //GPS Data
  if(gpsWrite){
    dataString[strPosn]=gpsLat;strPosn++;
    writeFloatData(gpsLatitude,6);
    dataString[strPosn]=gpsLon;strPosn++;
    writeFloatData(gpsLongitude,6);
    writeFloatData((float)GPS.speed.mph(),2);
    writeFloatData(((float)GPS.altitude.meters()-baseAlt),2);
    writeFloatData((float)GPS.course.deg(),2);
    writeIntData(radio.satNum);
    gpsWrite=false;}
  else{
    dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;
    dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;dataString[strPosn]=cs;strPosn++;}
  //update the radio packet number
  if (radioTX){writeIntData(radio.packetnum);radioTX = false;}
  else{dataString[strPosn]=cs;strPosn++;}
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  // write the string to file
  if(settings.testMode){writeStart = micros();}
  outputFile.write(dataString, strPosn);
  if(settings.testMode){writeTime = micros() - writeStart;
    if(writeTime > maxWriteTime){maxWriteTime = writeTime;}
    if(writeTime > writeThreshold){writeThreshCount++;}}
  strPosn = 0;
}//end write SD data

void writeSDfooter(){
  //Print the initial conditions
  outputFile.println(F("Max Baro Alt,Max GPS Alt,Max Speed,Max Gs,baseAlt,padTime,initial Y ang,initial X ang,accelX0,accelY0,accelZ0,highGz0,magX0,magY0,magZ0,gyroBiasX,gyroBiasY,gyroBiasZ,accelBiasX,accelBiasY,accelBiasZ,highGbiasX,highGbiasY,highGbiasZ,magBiasX,magBiasY,magBiasZ"));
  writeULongData((unsigned long)(maxAltitude*unitConvert));
  writeULongData((unsigned long)(maxGPSalt*unitConvert));
  writeULongData((unsigned long)(maxVelocity*unitConvert));
  writeFloatData(maxG/9.80665, 2);
  writeFloatData(baseAlt, 2);
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
  //carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);
  
  //write out the launch time and locations
  strPosn = 0;
  outputFile.println(F("launch date, UTC time, launch altitude, launch latitude, launch longitude"));
  //Write out the GPS liftoff date
  outputFile.print(liftoffDay);outputFile.print("/");outputFile.print(liftoffMonth);outputFile.print("/");outputFile.print(liftoffYear);outputFile.print(",");
  //Write out the GPS liftoff time
  outputFile.print(liftoffHour);outputFile.print(":");outputFile.print(liftoffMin);outputFile.print(":");outputFile.print((int)liftoffSec);outputFile.print(",");
  //Write out GPS launch location
  writeFloatData(baseGPSalt,2);
  dataString[strPosn]=liftoffLat; strPosn++; writeFloatData2(liftoffLatitude,4);
  dataString[strPosn]=liftoffLon; strPosn++; writeFloatData2(liftoffLongitude,4);
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';strPosn++;
  dataString[strPosn] = '\n';strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);

  //Write out the GPS landing location
  strPosn = 0;
  outputFile.println(F("landing date, UTC time, landing altitude, landing latitude, landing longitude"));
  //Write out the GPS landing date
  outputFile.print(liftoffDay);outputFile.print("/");outputFile.print(liftoffMonth);outputFile.print("/");outputFile.print(liftoffYear);outputFile.print(",");
  //Write out the GPS landing time
  outputFile.print(touchdownHour);outputFile.print(":");outputFile.print(touchdownMin);outputFile.print(":");outputFile.print((int)touchdownSec);outputFile.print(",");
  writeFloatData(touchdownAlt,2);
  dataString[strPosn]=touchdownLat; strPosn++; writeFloatData2(touchdownLatitude,4);
  dataString[strPosn]=touchdownLon; strPosn++; writeFloatData2(touchdownLongitude,4);
  //end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r'; strPosn++;
  dataString[strPosn] = '\n'; strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);
  
  //write out the settings for the flight
  outputFile.print(F("Rocket Name, callsign, HWid, flightProfile, units, inflightRecover, pyro4func, pyro3func, pyro2func, pyro1func, gTrigger, detectLiftoffTime, apogeeDelay, mainDeployAlt, setupTime, rcdTime, fireTime, TXenable, TXpwr, TXfreq, FHSS, seaLevelPressure"));
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
  writeFloatData((float)settings.gTrigger/(float)g,1);
  writeFloatData(((float)settings.detectLiftoffTime)*mlnth,2);//issue
  writeFloatData(((float)settings.apogeeDelay)*mlnth,1);
  writeFloatData(((float)settings.mainDeployAlt)*unitConvert,0);
  writeFloatData(((float)settings.setupTime)/1000,0);//issue
  writeFloatData(settings.rcdTime*mlnth,0);
  writeFloatData(settings.fireTime*mlnth,1);
  writeBoolData(settings.TXenable);dataString[strPosn]=cs;strPosn++;
  writeIntData(settings.TXpwr);
  writeFloatData(settings.TXfreq, 3);
  writeBoolData(settings.FHSS);dataString[strPosn]=cs;strPosn++;
  writeFloatData(seaLevelPressure,2);      
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
  strPosn=0;}//end SD footer

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
  //limited to 4 decimal places only!  

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
