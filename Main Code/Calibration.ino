//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
// accelCalibrate(): one-time calibration routine for the IMU and High-G acceleromters
// magCalibrate(): one-time calibration routine for the IMU magnetometer.  Runs separate from the acceleration calibration routine
// writeCalibration(): function to store calibration values in eeprom
// setOrientation: part of the acceleromter calibration routine - automatically determines the flight computer orientation and stores in eeprom
//----------------------------

void accelCalibrate(){
      
    Serial.println(F("Accelerometer Calibration Mode Confirmed. Please hold altimeter vertical and still"));
    
    for (byte i = 1; i < 20; i++){
      digitalWrite(pins.beep, HIGH);
      delay(250);
      digitalWrite(pins.beep, LOW);
      delay(250);}

    digitalWrite(pins.beep, HIGH);

    //Align the highG accelerometer to the IMU
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
        break;}

    Serial.println(F("Calibrating..."));

    accel.biasX = accel.biasY = accel.biasZ = 0;
    accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
    highG.biasX = highG.biasY = highG.biasZ = 0;
    highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0; 

    for (byte i = 1; i < 101; i++){
      getAccel();
      getHighG(true);
      Serial.print(F("Accel X,Y,Z: "));Serial.print(accel.rawX);Serial.print(',');Serial.print(accel.rawY);Serial.print(',');Serial.println(accel.rawZ);
      Serial.print(F("highG X,Y,Z: "));Serial.print(highG.rawX);Serial.print(',');Serial.print(highG.rawY);Serial.print(',');Serial.println(highG.rawZ);
      accel.sumX0 += accel.rawX;
      accel.sumY0 += accel.rawY;
      accel.sumZ0 += accel.rawZ;
      highG.sumX0 += highG.rawX;
      highG.sumY0 += highG.rawY;
      highG.sumZ0 += highG.rawZ;
      delay(300);}
      
    //calculate the bias
    accel.biasX = (int)(accel.sumX0 / 100);
    accel.biasY = (int)(accel.sumY0 / 100);
    accel.biasZ = (int)(accel.sumZ0 / 100);
    highG.biasX = (int)(highG.sumX0 / 100);
    highG.biasY = (int)(highG.sumY0 / 100);
    highG.biasZ = (int)(highG.sumZ0 / 100);

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
    Serial.print(F("highG.biasZ: "));Serial.println(highG.biasZ);}//end accelerometer calibration

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
  if(abs(orientFrameZ) > abs(orientFrameY) && abs(orientFrameZ) > abs(orientFrameX) && orientFrameZ > 0){orientCase = 1;}//Main IMU Z-axis is pointed to the nose
  if(abs(orientFrameZ) > abs(orientFrameY) && abs(orientFrameZ) > abs(orientFrameX) && orientFrameZ < 0){orientCase = 2;}//Main IMU Z-axis is pointed to the tail
  if(abs(orientFrameY) > abs(orientFrameZ) && abs(orientFrameY) > abs(orientFrameX) && orientFrameY > 0){orientCase = 3;}//Main IMU Y-axis is pointed to the nose
  if(abs(orientFrameY) > abs(orientFrameZ) && abs(orientFrameY) > abs(orientFrameX) && orientFrameY < 0){orientCase = 4;}//Main IMU Y-axis is pointed to the tail
  if(abs(orientFrameX) > abs(orientFrameZ) && abs(orientFrameX) > abs(orientFrameY) && orientFrameX > 0){orientCase = 5;}//Main IMU X-axis is pointed to the nose
  if(abs(orientFrameX) > abs(orientFrameZ) && abs(orientFrameX) > abs(orientFrameY) && orientFrameX < 0){orientCase = 6;}//Main IMU X-axis is pointed to the tail

  //set the orientation variables
  switch (orientCase) {

    case 1: //Main IMU Z-axis is pointed to the nose
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Z';

      //set IMU axes direction
      accel.dirX *= 1;
      accel.dirY *= 1;
      accel.dirZ *= 1;
      gyro.dirX *= 1;
      gyro.dirY *= 1;
      gyro.dirZ *= 1;
      mag.dirX *= 1;
      mag.dirY *= 1;
      mag.dirZ *= 1;

      break;

    case 2: //Main IMU Z-axis is pointed to the tail
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Z';

      //set IMU axes direction
      accel.dirX *= -1;
      accel.dirY *= 1;
      accel.dirZ *= -1;
      gyro.dirX *= -1;
      gyro.dirY *= 1;
      gyro.dirZ *= -1;
      mag.dirX *= -1;
      mag.dirY *= 1;
      mag.dirZ *= -1;
      
      break;

    case 3: //Main IMU Y-axis is pointed to the nose
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Z';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Y';

      //set IMU axes direction
      accel.dirX *= 1;
      accel.dirY *= 1;
      accel.dirZ *= -1;
      gyro.dirX *= 1;
      gyro.dirY *= 1;
      gyro.dirZ *= -1;
      mag.dirX *= 1;
      mag.dirY *= 1;
      mag.dirZ *= -1;

      break;

    case 4: //Main IMU Y-axis is pointed to the tail
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'X';
      accel.orientY = mag.orientY = gyro.orientY = 'Z';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'Y';

      //set IMU axes direction
      accel.dirX *= 1;
      accel.dirY *= -1;
      accel.dirZ *= 1;
      gyro.dirX *= 1;
      gyro.dirY *= -1;
      gyro.dirZ *= 1;
      mag.dirX *= 1;
      mag.dirY *= -1;
      mag.dirZ *= 1;
      
      break;

    case 5: //Main IMU X-axis is pointed to the nose
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'Z';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'X';

      //set IMU axes direction
      accel.dirX *= 1;
      accel.dirY *= 1;
      accel.dirZ *= -1;
      gyro.dirX *= 1;
      gyro.dirY *= 1;
      gyro.dirZ *= -1;
      mag.dirX *= 1;
      mag.dirY *= 1;
      mag.dirZ *= -1;
      
      break;

    case 6: //Main IMU X-axis is pointed to the tail
    
      //align IMU axes
      accel.orientX = mag.orientX = gyro.orientX = 'Z';
      accel.orientY = mag.orientY = gyro.orientY = 'Y';
      accel.orientZ = mag.orientZ = gyro.orientZ = 'X';

      //set IMU axes direction
      accel.dirX *= -1;
      accel.dirY *= 1;
      accel.dirZ *= 1;
      gyro.dirX *= -1;
      gyro.dirY *= 1;
      gyro.dirZ *= 1;
      mag.dirX *= -1;
      mag.dirY *= 1;
      mag.dirZ *= 1;
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
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Z'){highG.dirZ = (char)EEPROM.read(eeprom.IMU2hiGzSign) * accel.dirZ;}//vertical axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Z'){highG.dirY = (char)EEPROM.read(eeprom.IMU2hiGySign) * accel.dirZ;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Z'){highG.dirX = (char)EEPROM.read(eeprom.IMU2hiGxSign) * accel.dirZ;}
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='Y'){highG.dirZ = (char)EEPROM.read(eeprom.IMU2hiGzSign) * accel.dirY;}//horizontal y axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='Y'){highG.dirY = (char)EEPROM.read(eeprom.IMU2hiGySign) * accel.dirY;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='Y'){highG.dirX = (char)EEPROM.read(eeprom.IMU2hiGxSign) * accel.dirY;}
  if((char)EEPROM.read(eeprom.IMU2hiGzAxis)=='X'){highG.dirZ = (char)EEPROM.read(eeprom.IMU2hiGzSign) * accel.dirX;}//horizontal x axis
  if((char)EEPROM.read(eeprom.IMU2hiGyAxis)=='X'){highG.dirY = (char)EEPROM.read(eeprom.IMU2hiGySign) * accel.dirX;}
  if((char)EEPROM.read(eeprom.IMU2hiGxAxis)=='X'){highG.dirX = (char)EEPROM.read(eeprom.IMU2hiGxSign) * accel.dirX;}

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
  Serial.print(F("IMU 2 Hi-Gx: "));Serial.print((int8_t)EEPROM.read(eeprom.IMU2hiGxSign));Serial.println((char)EEPROM.read(eeprom.IMU2hiGxAxis));
  Serial.print(F("IMU 2 Hi-Gy: "));Serial.print((int8_t)EEPROM.read(eeprom.IMU2hiGySign));Serial.println((char)EEPROM.read(eeprom.IMU2hiGyAxis));
  Serial.print(F("IMU 2 Hi-Gz: "));Serial.print((int8_t)EEPROM.read(eeprom.IMU2hiGzSign));Serial.println((char)EEPROM.read(eeprom.IMU2hiGzAxis));
  Serial.print(F("IMU.X: "));Serial.print((accel.dirX == 1) ? '+' : '-');Serial.println(accel.orientX);
  Serial.print(F("IMU.Y: "));Serial.print((accel.dirY == 1) ? '+' : '-');Serial.println(accel.orientY);
  Serial.print(F("IMU.Z: "));Serial.print((accel.dirZ == 1) ? '+' : '-');Serial.println(accel.orientZ);
  Serial.print(F("highG.X: "));Serial.print((highG.dirX == 1) ? '+' : '-');Serial.println(highG.orientX);
  Serial.print(F("highG.Y: "));Serial.print((highG.dirY == 1) ? '+' : '-');Serial.println(highG.orientY);
  Serial.print(F("highG.Z: "));Serial.print((highG.dirZ == 1) ? '+' : '-');Serial.println(highG.orientZ);
  }//end void
