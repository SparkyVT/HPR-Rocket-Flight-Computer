//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
// accelCalibrate(): one-time calibration routine for the IMU and High-G acceleromters
// magCalibrate(): one-time calibration routine for the IMU magnetometer.  Runs separate from the acceleration calibration routine
// writeCalibration(): function to store calibration values in eeprom
// setOrientation: part of the acceleromter calibration routine - automatically determines the flight computer orientation and stores in eeprom
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//---------------------------------

void accelCalibrate(){
      
    Serial.println(F("Accelerometer Calibration Mode Confirmed. Ensure the altimeter is pointed vertical and held steady on a level surface"));
    
    for (byte i = 1; i < 20; i++){
      digitalWrite(pins.beep, HIGH);
      delay(250);
      digitalWrite(pins.beep, LOW);
      delay(250);}

    digitalWrite(pins.beep, HIGH);

    //reset the pointers and directions
    accel.ptrX = &accel.rawX;
    accel.ptrY = &accel.rawY;
    accel.ptrZ = &accel.rawZ;
    accel.ptrXdir = &accel.dirX;
    accel.ptrYdir = &accel.dirY;
    accel.ptrZdir = &accel.dirZ;
    accel.orientX = 'X';
    accel.orientY = 'Y';
    accel.orientZ = 'Z';
    accel.dirX = accel.dirY = accel.dirZ = 1;
    highG.ptrX = &highG.rawX;
    highG.ptrY = &highG.rawY;
    highG.ptrZ = &highG.rawZ;
    highG.ptrXdir = &highG.dirX;
    highG.ptrYdir = &highG.dirY;
    highG.ptrZdir = &highG.dirZ;
    highG.orientX = 'X';
    highG.orientY = 'Y';
    highG.orientZ = 'Z';
    highG.dirX = highG.dirY = highG.dirZ = 1;

    /*//Align the highG accelerometer to the IMU
    highG.dirX = (int8_t)EEPROM.read(eeprom.IMU2hiGxSign);
    if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='X'){highG.ptrX = &highG.rawX; highG.orientX = 'X';}
    if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Y'){highG.ptrX = &highG.rawY; highG.orientX = 'Y';}
    if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Z'){highG.ptrX = &highG.rawZ; highG.orientX = 'Z';}
    highG.dirY = (int8_t)EEPROM.read(eeprom.IMU2hiGySign);
    if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='X'){highG.ptrY = &highG.rawX; highG.orientY = 'X';}
    if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Y'){highG.ptrY = &highG.rawY; highG.orientY = 'Y';}
    if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Z'){highG.ptrY = &highG.rawZ; highG.orientY = 'Z';}
    highG.dirZ = (int8_t)EEPROM.read(eeprom.IMU2hiGzSign);
    if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='X'){highG.ptrZ = &highG.rawX; highG.orientZ = 'X';}
    if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Y'){highG.ptrZ = &highG.rawY; highG.orientZ = 'Y';}
    if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Z'){highG.ptrZ = &highG.rawZ; highG.orientZ = 'Z';}

    //align the IMU
    switch (sensors.accel){
  
      //LSM303 & L3GD20H
      case 1:
        //orient to common axis
        accel.dirX = mag.dirX = gyro.dirX = 1;
        accel.dirY = mag.dirY = gyro.dirY = 1;
        accel.dirZ = mag.dirZ = gyro.dirZ = 1;
        break;
        
      //LSM9DS1
      case 2: 
        //orient to common axis
        accel.dirX = mag.dirX = gyro.dirX = -1;
        accel.dirY = mag.dirY = gyro.dirY = 1;
        accel.dirZ = mag.dirZ = gyro.dirZ = 1;
        highG.dirX *= -1;
        break;}*/

    Serial.println(F("Calibrating..."));

    accel.biasX = accel.biasY = accel.biasZ = 0;
    accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
    highG.biasX = highG.biasY = highG.biasZ = 0;
    highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0; 

    int dispData = 0;
    int sampSize = 10000;
    uint32_t delayTime = 714*(1-0.8);//average cycle time * (100% - time spent in I2C comms)
    for (int i = 0; i < sampSize; i++){
      getAccel();
      getHighG(true);
      dispData++;
      if(dispData > 1000){
        Serial.print(F("Accel X,Y,Z: "));Serial.print(accel.rawX);Serial.print(',');Serial.print(accel.rawY);Serial.print(',');Serial.println(accel.rawZ);
        Serial.print(F("highG X,Y,Z: "));Serial.print(highG.rawX);Serial.print(',');Serial.print(highG.rawY);Serial.print(',');Serial.println(highG.rawZ);
        dispData = 0;}
      accel.sumX0 += accel.rawX;
      accel.sumY0 += accel.rawY;
      accel.sumZ0 += accel.rawZ;
      highG.sumX0 += highG.rawX;
      highG.sumY0 += highG.rawY;
      highG.sumZ0 += highG.rawZ;
      delayMicroseconds(delayTime);}
      
    //calculate the bias
    accel.biasX = (int)(accel.sumX0 / sampSize);
    accel.biasY = (int)(accel.sumY0 / sampSize);
    accel.biasZ = (int)(accel.sumZ0 / sampSize);
    highG.biasX = (int)(highG.sumX0 / sampSize);
    highG.biasY = (int)(highG.sumY0 / sampSize);
    highG.biasZ = (int)(highG.sumZ0 / sampSize);

    //reset the counters
    accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
    highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0;
    
    //determine the altimeter orientation
    setOrientation();

    //Store bias in EEPROM    
    writeCalibration(accel.biasX, eeprom.accelBiasX);
    writeCalibration(accel.biasY, eeprom.accelBiasY);
    writeCalibration(accel.biasZ, eeprom.accelBiasZ);
    writeCalibration(highG.biasX, eeprom.highGbiasX);
    writeCalibration(highG.biasY, eeprom.highGbiasY);
    writeCalibration(highG.biasZ, eeprom.highGbiasZ);
        
    digitalWrite(pins.beep, LOW);
    Serial.println(F("Calibration complete!  Calculated values are:"));
    Serial.print(F("accel.biasX: "));Serial.println(accel.biasX);
    Serial.print(F("accel.biasY: "));Serial.println(accel.biasY);
    Serial.print(F("accel.biasZ: "));Serial.println(accel.biasZ);
    Serial.print(F("highG.biasX: "));Serial.println(highG.biasX);
    Serial.print(F("highG.biasY: "));Serial.println(highG.biasY);
    Serial.print(F("highG.biasZ: "));Serial.println(highG.biasZ);  
    
  }//end accelerometer/gyro calibration

void magCalibrate(){
  
    //reset the magnetometer gain
    setMagGain(false);

    //Clear out the FIFO
    getMag();
    delay(100);
    getMag();
    
    Serial.println(F("Magnetometer Calibration Mode"));
    //give a 20 second audible warning
    for(byte i = 0; i < 20; i++){
      digitalWrite(pins.beep, HIGH);
      delay(250);
      digitalWrite(pins.beep, LOW);
      delay(750);}

    //initialize the offsets
    int maxMagX, maxMagY, maxMagZ;
    int minMagX, minMagY, minMagZ;
    boolean initMag = true;
    mag.biasX = mag.biasY = mag.biasZ = 0;
    
    Serial.println(F("Sampling Magnetometer"));    
    //sample at 10Hz for 30seconds
    for(int i = 0; i < 300; i++){
      getMag();
      Serial.print(mag.rawX);Serial.print(',');Serial.print(mag.rawY);Serial.print(',');Serial.println(mag.rawZ);
      if(initMag){
        maxMagX = minMagX = mag.rawX;
        maxMagY = minMagY = mag.rawY;
        maxMagZ = minMagZ = mag.rawZ;
        initMag = false;}
      if(mag.rawX > maxMagX){maxMagX = mag.rawX;}
      if(mag.rawY > maxMagY){maxMagY = mag.rawY;}
      if(mag.rawZ > maxMagZ){maxMagZ = mag.rawZ;}
      if(mag.rawX < minMagX){minMagX = mag.rawX;}
      if(mag.rawY < minMagY){minMagY = mag.rawY;}
      if(mag.rawZ < minMagZ){minMagZ = mag.rawZ;}
      delay(100);}

    //Calculate the bias
    Serial.print("minMagX: ");Serial.print(minMagX);Serial.print(", maxMagX: ");Serial.println(maxMagX);
    Serial.print("minMagY: ");Serial.print(minMagY);Serial.print(", maxMagY: ");Serial.println(maxMagY);
    Serial.print("minMagZ: ");Serial.print(minMagZ);Serial.print(", maxMagZ: ");Serial.println(maxMagZ);
    mag.biasX = (int16_t)(minMagX + (maxMagX - minMagX)/2);
    mag.biasY = (int16_t)(minMagY + (maxMagY - minMagY)/2);
    mag.biasZ = (int16_t)(minMagZ + (maxMagZ - minMagZ)/2);

    //Write to EEPROM
    writeCalibration(mag.biasX, eeprom.magBiasX);
    writeCalibration(mag.biasY, eeprom.magBiasY);
    writeCalibration(mag.biasZ, eeprom.magBiasZ);

    //set the variables to proceed to bench-test mode
    settings.testMode = true;
    settings.magSwitchEnable = false;
    
    Serial.println(F("Magnetometer Calibration Complete!"));

    //beep three times to signal calibration complete
    digitalWrite(pins.beep, HIGH);
    delay(200);
    digitalWrite(pins.beep, LOW);
    delay(200);
    digitalWrite(pins.beep, HIGH);
    delay(200);
    digitalWrite(pins.beep, LOW);
    delay(200);
    digitalWrite(pins.beep, HIGH);
    delay(200);
    digitalWrite(pins.beep, LOW);
    delay(2000);} //end magnetometer calibration

 void writeCalibration(int16_t inValue, byte eepromStart){

    calUnion.calValue = inValue;
    EEPROM.update(eepromStart, calUnion.calByte[0]);
    EEPROM.update(eepromStart + 1, calUnion.calByte[1]);}

void setOrientation(){

  byte orientCase = 0;
  
  //determine which of the 6 possible IMU orientations is present
  if(abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX) && accel.biasZ > 0){orientCase = 1;}//Main IMU Z-axis is pointed to the nose
  if(abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX) && accel.biasZ < 0){orientCase = 2;}//Main IMU Z-axis is pointed to the tail
  if(abs(accel.biasY) > abs(accel.biasZ) && abs(accel.biasY) > abs(accel.biasX) && accel.biasY > 0){orientCase = 3;}//Main IMU Y-axis is pointed to the nose
  if(abs(accel.biasY) > abs(accel.biasZ) && abs(accel.biasY) > abs(accel.biasX) && accel.biasY < 0){orientCase = 4;}//Main IMU Y-axis is pointed to the tail
  if(abs(accel.biasX) > abs(accel.biasZ) && abs(accel.biasX) > abs(accel.biasY) && accel.biasX > 0){orientCase = 5;}//Main IMU X-axis is pointed to the nose
  if(abs(accel.biasX) > abs(accel.biasZ) && abs(accel.biasX) > abs(accel.biasY) && accel.biasX < 0){orientCase = 6;}//Main IMU X-axis is pointed to the tail

  //set the orientation variables
  switch (orientCase) {

    case 1: //Main IMU Z-axis is pointed to the nose

      Serial.println("Main IMU Z-axis is pointed to the nose");
      
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Z';

      //set IMU axes direction
      accel.dirX = 1;
      accel.dirY = 1;
      accel.dirZ = 1;
      gyro.dirX = 1;
      gyro.dirY = 1;
      gyro.dirZ = 1;
      mag.dirX = 1;
      mag.dirY = 1;
      mag.dirZ = 1;

      break;

    case 2: //Main IMU Z-axis is pointed to the tail

      Serial.println("Main IMU Z-axis is pointed to the tail");
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Z';

      //set IMU axes direction
      accel.dirX = -1;
      accel.dirY = 1;
      accel.dirZ = -1;
      gyro.dirX = -1;
      gyro.dirY = 1;
      gyro.dirZ = -1;
      mag.dirX = -1;
      mag.dirY = 1;
      mag.dirZ = -1;
      
      break;

    case 3: //Main IMU Y-axis is pointed to the nose

      Serial.println("Main IMU Y-axis is pointed to the nose");
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Z';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Y';

      //set IMU axes direction
      accel.dirX = 1;
      accel.dirY = 1;
      accel.dirZ = -1;
      gyro.dirX = 1;
      gyro.dirY = 1;
      gyro.dirZ = -1;
      mag.dirX = 1;
      mag.dirY = 1;
      mag.dirZ = -1;

      break;

    case 4: //Main IMU Y-axis is pointed to the tail

      Serial.println("Main IMU Y-axis is pointed to the tail");
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Z';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Y';

      //set IMU axes direction
      accel.dirX = 1;
      accel.dirY = -1;
      accel.dirZ = 1;
      gyro.dirX = 1;
      gyro.dirY = -1;
      gyro.dirZ = 1;
      mag.dirX = 1;
      mag.dirY = -1;
      mag.dirZ = 1;
      
      break;

    case 5: //Main IMU X-axis is pointed to the nose

      Serial.println("Main IMU X-axis is pointed to the nose");
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'Z';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'X';

      //set IMU axes direction
      accel.dirX = 1;
      accel.dirY = 1;
      accel.dirZ = -1;
      gyro.dirX = 1;
      gyro.dirY = 1;
      gyro.dirZ = -1;
      mag.dirX = 1;
      mag.dirY = 1;
      mag.dirZ = -1;
      
      break;

    case 6: //Main IMU X-axis is pointed to the tail

      Serial.println("Main IMU X-axis is pointed to the tail");
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'Z';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'X';

      //set IMU axes direction
      accel.dirX = -1;
      accel.dirY = 1;
      accel.dirZ = 1;
      gyro.dirX = -1;
      gyro.dirY = 1;
      gyro.dirZ = 1;
      mag.dirX = -1;
      mag.dirY = 1;
      mag.dirZ = 1;
      
      break;

    default: //error has occured

      break;
  }//end case

  //set high-G accelerometer axes
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Z'){highG.orientZ = accel.orientZ;}//vertical axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Z'){highG.orientY = accel.orientZ;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Z'){highG.orientX = accel.orientZ;}
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Y'){highG.orientZ = accel.orientY;}//horizontal y axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Y'){highG.orientY = accel.orientY;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Y'){highG.orientX = accel.orientY;}
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='X'){highG.orientZ = accel.orientX;}//horizontal x axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='X'){highG.orientY = accel.orientX;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='X'){highG.orientX = accel.orientX;}
      
  //set high-G accelerometer direction
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Z'){highG.dirZ = (int8_t)EEPROM.read(eeprom.IMU2hiGzSign) * accel.dirZ;}//vertical axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Z'){highG.dirY = (int8_t)EEPROM.read(eeprom.IMU2hiGySign) * accel.dirZ;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Z'){highG.dirX = (int8_t)EEPROM.read(eeprom.IMU2hiGxSign) * accel.dirZ;}
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Y'){highG.dirZ = (int8_t)EEPROM.read(eeprom.IMU2hiGzSign) * accel.dirY;}//horizontal y axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Y'){highG.dirY = (int8_t)EEPROM.read(eeprom.IMU2hiGySign) * accel.dirY;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Y'){highG.dirX = (int8_t)EEPROM.read(eeprom.IMU2hiGxSign) * accel.dirY;}
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='X'){highG.dirZ = (int8_t)EEPROM.read(eeprom.IMU2hiGzSign) * accel.dirX;}//horizontal x axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='X'){highG.dirY = (int8_t)EEPROM.read(eeprom.IMU2hiGySign) * accel.dirX;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='X'){highG.dirX = (int8_t)EEPROM.read(eeprom.IMU2hiGxSign) * accel.dirX;}

  //correct IMU bias for 1G
  if(accel.orientX == 'Z'){accel.biasX -= accel.dirX * g;}
  if(accel.orientY == 'Z'){accel.biasY -= accel.dirY * g;}
  if(accel.orientZ == 'Z'){accel.biasZ -= accel.dirZ * g;}
  
  //correct high-G bias for 1G
  if(highG.orientX == 'Z'){highG.biasX -= highG.dirX * high1G;}
  if(highG.orientY == 'Z'){highG.biasY -= highG.dirY * high1G;}
  if(highG.orientZ == 'Z'){highG.biasZ -= highG.dirZ * high1G;}

  //write to EEPROM
  //IMU
  EEPROM.update(eeprom.imuXsign, accel.dirX);
  EEPROM.update(eeprom.imuXptr, accel.orientX);
  EEPROM.update(eeprom.imuYsign, accel.dirY);
  EEPROM.update(eeprom.imuYptr, accel.orientY);
  EEPROM.update(eeprom.imuZsign, accel.dirZ);
  EEPROM.update(eeprom.imuZptr, accel.orientZ);
  //highG
  EEPROM.update(eeprom.hiGxSign, highG.dirX);
  EEPROM.update(eeprom.hiGxPtr, highG.orientX);
  EEPROM.update(eeprom.hiGySign, highG.dirY);
  EEPROM.update(eeprom.hiGyPtr, highG.orientY);
  EEPROM.update(eeprom.hiGzSign, highG.dirZ);
  EEPROM.update(eeprom.hiGzPtr, highG.orientZ);

  //display values from EEPROM
  Serial.print(F("IMU.X points to Hi-G: "));Serial.print(((int8_t)EEPROM.read(eeprom.IMU2hiGxSign) == 1) ? '+' : '-');Serial.println((char)EEPROM.read(eeprom.IMU2hiGxAxis));
  Serial.print(F("IMU.Y points to Hi-G: "));Serial.print(((int8_t)EEPROM.read(eeprom.IMU2hiGySign) == 1) ? '+' : '-');Serial.println((char)EEPROM.read(eeprom.IMU2hiGyAxis));
  Serial.print(F("IMU.Z points to Hi-G: "));Serial.print(((int8_t)EEPROM.read(eeprom.IMU2hiGzSign) == 1) ? '+' : '-');Serial.println((char)EEPROM.read(eeprom.IMU2hiGzAxis));
  Serial.print(F("IMU.X is pointed to real world: "));Serial.print((accel.dirX == 1) ? '+' : '-');Serial.println(accel.orientX);
  Serial.print(F("IMU.Y is pointed to real world: "));Serial.print((accel.dirY == 1) ? '+' : '-');Serial.println(accel.orientY);
  Serial.print(F("IMU.Z is pointed to real world: "));Serial.print((accel.dirZ == 1) ? '+' : '-');Serial.println(accel.orientZ);
  Serial.print(F("highG.X is pointed to real world: "));Serial.print((highG.dirX == 1) ? '+' : '-');Serial.println(highG.orientX);
  Serial.print(F("highG.Y is pointed to real world: "));Serial.print((highG.dirY == 1) ? '+' : '-');Serial.println(highG.orientY);
  Serial.print(F("highG.Z is pointed to real world: "));Serial.print((highG.dirZ == 1) ? '+' : '-');Serial.println(highG.orientZ);
  }//end void

void baroCalibrate(){

  char serialInput;
  
  //exit the beeping
  digitalWrite(pins.beep, LOW);
  beep_counter = 8;

  //Send user message
  Serial.println(F("Barometer Calibration mode initiated"));
  
  //clear the user input
  while(Serial.available() > 0){serialInput = Serial.read();}

  //Display the user temperature instructions
  Serial.println(F("Enter the current ambient room temperature in degrees and tenths of a degree C, or enter e to exit: "));

  //wait for user input, then enter into the array
  while(Serial.available() == 0){delay(100);}
  strPosn = 0;
  while(Serial.available() > 0){
    serialInput = Serial.read();
    dataString[strPosn] = serialInput; strPosn++;
    if(serialInput == 'e'){Serial.println(F("Exiting calibration")); return;}}
  dataString[strPosn] = '\0';

  //parse the data
  float userInput = atof(dataString);
  Serial.print(F("User input: "));Serial.println(userInput, 1);
  Serial.println(F("Calibrating..."));

  //Sample Temperature Sensor
  float tempSum = 0.0F;
  baroTempOffset = 0.0F;
  for(int i = 0; i < 300; i++){
    getBaro();
    tempSum += temperature;
    delayMicroseconds(timeBtwnBaro);}
  
  baroTempOffset = (tempSum / 300) - userInput;
  Serial.println(baroTempOffset, 1);

  //Print the sample conditions
  Serial.println(F("Temperature calibration complete"));
  Serial.print(F("Sampled atmospheric temperature: ")); Serial.println(tempSum/300, 1);
  Serial.print("Temperature Offset: ");Serial.println(baroTempOffset, 1);
      
  //Display the user temperature instructions
  Serial.println(F("Enter the current barometric pressure in hPa: "));

  //wait for user input, then enter into the array
  while(Serial.available()==0){delay(100);}
  strPosn = 0;
  
  while(Serial.available() > 0){
    char input = Serial.read();
    dataString[strPosn] = input; strPosn++;}
  dataString[strPosn] = '\0';

  //parse the data
  userInput = atof(dataString);
  Serial.print(F("User input: "));Serial.println(userInput, 2);
  Serial.println(F("Calibrating..."));
  
  //Sample Barometer
  baroPressOffset = 0.0F;
  float baroSum = 0.0F;
  for(int i = 0; i < 300; i++){
    getBaro();
    baroSum += pressure;
    delayMicroseconds(timeBtwnBaro);}

  baroPressOffset = (baroSum / 300) - userInput;

  //Print the sample conditions
  Serial.println(F("Barometer calibration Complete"));
  Serial.print(F("Sampled barometric pressure: ")); Serial.println(baroSum/300, 2);
  Serial.print("Barometric Offset: ");Serial.print(baroPressOffset, 2); Serial.println(F(" hPa"));

  //write to eeprom
  floatUnion.val = baroTempOffset;
  for(byte i = 0; i < 4; i++){EEPROM.update(eeprom.baroTempOffset + i, floatUnion.Byte[i]);}
  floatUnion.val = baroPressOffset;
  for(byte i = 0; i < 4; i++){EEPROM.update(eeprom.baroPressOffset + i, floatUnion.Byte[i]);}

  delay(2000);
    
 }//End of barometer calibration

 char a;
char b;
char d;

void setCanardTrim(){

  //Send instructions to the user
  Serial.println(F("Enter the servo number and direction of rotation.  Each entry will add/subtract 1 degree of Trim."));
  Serial.println(F("Ex: 2+ will move Servo #2 one degree clockwise, 6- will move Servo #6 one degree counter clockwise"));
  Serial.println(F("Send ee to exit"));
  //clear the buffer
  while(Serial.available()>0){a = Serial.read();}

  //Set loop flags
  boolean errorFlag = false;
  boolean exitFlag = false;

  //loop and wait for the user to enter a value
  while(!exitFlag){
    if(Serial.available() == 3){

      //read data
      a = Serial.read();
      b = Serial.read();
      d = Serial.read();
      
      //exit condition
      if(a == 'e' || b == 'e'){exitFlag = true;}

      //error check
      if(a!='1' && a!='2' && a!='3' && a!='4' && a!='5' && a!='6' && a!='7' && a!='8'){errorFlag = true;}
      if(b!='+' && b!='-'){errorFlag = true;}

      //update trim settings
      if(a=='1'){servo1trim += (b=='+'? 1 : -1); Serial.print(F("Servo1 Trim: "));Serial.println(servo1trim); wiggleServo(1);}
      if(a=='2'){servo2trim += (b=='+'? 1 : -1); Serial.print(F("Servo2 Trim: "));Serial.println(servo2trim); wiggleServo(2);}
      if(a=='3'){servo3trim += (b=='+'? 1 : -1); Serial.print(F("Servo3 Trim: "));Serial.println(servo3trim); wiggleServo(3);}
      if(a=='4'){servo4trim += (b=='+'? 1 : -1); Serial.print(F("Servo4 Trim: "));Serial.println(servo4trim); wiggleServo(4);}
      if(a=='5'){servo5trim += (b=='+'? 1 : -1); Serial.print(F("Servo5 Trim: "));Serial.println(servo5trim); wiggleServo(5);}
      if(a=='6'){servo6trim += (b=='+'? 1 : -1); Serial.print(F("Servo6 Trim: "));Serial.println(servo6trim); wiggleServo(6);}
      if(a=='7'){servo7trim += (b=='+'? 1 : -1); Serial.print(F("Servo7 Trim: "));Serial.println(servo7trim); wiggleServo(7);}
      if(a=='8'){servo8trim += (b=='+'? 1 : -1); Serial.print(F("Servo8 Trim: "));Serial.println(servo8trim); wiggleServo(8);}}
      
    else if(Serial.available() > 0){
      Serial.print("Sent Bytes: ");Serial.println(Serial.available());
      while(Serial.available() > 0){a = (char)Serial.read(); Serial.print(a);}
      Serial.println(' ');
      errorFlag = true;}

    //report error message if needed
    if(!exitFlag && errorFlag){
      Serial.println(F("Error: Send a valid two character string. Ex: 3+"));
      errorFlag = false;}

    delay(100);
  }//end trim update loop

  //update the EEPROM
  EEPROM.update(eeprom.servo1trim, servo1trim);
  EEPROM.update(eeprom.servo2trim, servo2trim);
  EEPROM.update(eeprom.servo3trim, servo3trim);
  EEPROM.update(eeprom.servo4trim, servo4trim);
  EEPROM.update(eeprom.servo5trim, servo5trim);
  EEPROM.update(eeprom.servo6trim, servo6trim);
  EEPROM.update(eeprom.servo7trim, servo7trim);
  EEPROM.update(eeprom.servo8trim, servo8trim);

  //display settings to user
  Serial.println(F("FINAL SERVO TRIM SETTINGS"));
  Serial.print(F("Servo1 Trim: "));Serial.println(servo1trim);
  Serial.print(F("Servo2 Trim: "));Serial.println(servo2trim);
  Serial.print(F("Servo3 Trim: "));Serial.println(servo3trim);
  Serial.print(F("Servo4 Trim: "));Serial.println(servo4trim);
  Serial.print(F("Servo5 Trim: "));Serial.println(servo5trim);
  Serial.print(F("Servo6 Trim: "));Serial.println(servo6trim);
  Serial.print(F("Servo7 Trim: "));Serial.println(servo7trim);
  Serial.print(F("Servo8 Trim: "));Serial.println(servo8trim);
 }

void wiggleServo( byte servo){

  switch (servo){
    
  case 1:
    canardYaw1.write(80);delay(250);
    canardYaw1.write(100);delay(250);
    canardYaw1.write(90-servo1trim);
    break;
    
  case 2:
    canardYaw2.write(80);delay(250);
    canardYaw2.write(100);delay(250);
    canardYaw2.write(90-servo2trim);
    break;
    
  case 3:
    canardPitch3.write(80);delay(250);
    canardPitch3.write(100);delay(250);
    canardPitch3.write(90-servo3trim);
    break;
    
  case 4:
    canardPitch4.write(80);delay(250);
    canardPitch4.write(100);delay(250);
    canardPitch4.write(90-servo4trim);
    break;
    
  case 5:
    actionServo5.write(80);delay(250);
    actionServo5.write(100);delay(250);
    actionServo5.write(90-servo5trim);
    break;
    
  case 6:
    actionServo6.write(80);delay(250);
    actionServo6.write(100);delay(250);
    actionServo6.write(90-servo6trim);
    break;
    
  case 7:
    actionServo7.write(80);delay(250);
    actionServo7.write(100);delay(250);
    actionServo7.write(90-servo7trim);
    break;
    
  case 8:
    actionServo8.write(80);delay(250);
    actionServo8.write(100);delay(250);
    actionServo8.write(90-servo8trim);
    break;}
  
  }//end void wiggle servo
