//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
// accelCalibrate(): one-time calibration routine for the IMU and High-G acceleromters
// magCalibrate(): one-time calibration routine for the IMU magnetometer.  Runs separate from the acceleration calibration routine
// writeCalibration(): function to store calibration values in eeprom
// setOrientation(): part of the acceleromter calibration routine - automatically determines the flight computer orientation and stores in eeprom
// readOrientation(): reads the orientation pointers from eeprom
// baroCalibrate(): one-time calilbration routine with user input over Serial for the barometric pressure sensor offset and temperature offset
// setCanardTrim(): one-time calibration routine with user input over Serial to set the canard trim for active stabilization & return capability
// wiggleServo(): function that moves the servo being calibrated.  Useful since the movements are only 1 degree
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//10 SEP 21: barometer calibration routine created
//30 DEC 21: updated magnetomer calibration to account for revised resetMagGain routine
//18 APR 22: major overhaul to account for independent sensors in all directions, created readOrientation
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
    gyro.ptrX = &gyro.rawX;
    gyro.ptrY = &gyro.rawY;
    gyro.ptrZ = &gyro.rawZ;
    mag.ptrX = &mag.rawX;
    mag.ptrY = &mag.rawY;
    mag.ptrZ = &mag.rawZ;
    highG.ptrX = &highG.rawX;
    highG.ptrY = &highG.rawY;
    highG.ptrZ = &highG.rawZ;
    //resetting the direction pointers isn't necessary because everything is now 1
    accel.dirX = accel.dirY = accel.dirZ = 1;
    gyro.dirX = gyro.dirY = gyro.dirZ = 1;
    mag.dirX = mag.dirY = mag.dirZ = 1;
    highG.dirX = highG.dirY = highG.dirZ = 1;

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
      getHighG();
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
    Serial.println(F("Calculated values are:"));
    Serial.print(F("accel.biasX: "));Serial.println(accel.biasX);
    Serial.print(F("accel.biasY: "));Serial.println(accel.biasY);
    Serial.print(F("accel.biasZ: "));Serial.println(accel.biasZ);
    Serial.print(F("highG.biasX: "));Serial.println(highG.biasX);
    Serial.print(F("highG.biasY: "));Serial.println(highG.biasY);
    Serial.print(F("highG.biasZ: "));Serial.println(highG.biasZ);  
    
  }//end accelerometer/gyro calibration

void magCalibrate(){
  
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

  //read from EEPROM the sensor orientations relative to the board
  accel.orientX = (char)EEPROM.read(eeprom.accel2boardXaxis);
  accel.dirX = (uint8_t)EEPROM.read(eeprom.accel2boardXsign);
  accel.orientY = (char)EEPROM.read(eeprom.accel2boardYaxis);
  accel.dirY = (uint8_t)EEPROM.read(eeprom.accel2boardYsign);
  accel.orientZ = (char)EEPROM.read(eeprom.accel2boardZaxis);
  accel.dirZ = (uint8_t)EEPROM.read(eeprom.accel2boardZsign);
  gyro.orientX = (char)EEPROM.read(eeprom.gyro2boardXaxis);
  gyro.dirX = (uint8_t)EEPROM.read(eeprom.gyro2boardXsign);
  gyro.orientY = (char)EEPROM.read(eeprom.gyro2boardYaxis);
  gyro.dirY = (uint8_t)EEPROM.read(eeprom.gyro2boardYsign);
  gyro.orientZ = (char)EEPROM.read(eeprom.gyro2boardZaxis);
  gyro.dirZ = (uint8_t)EEPROM.read(eeprom.gyro2boardZsign);
  mag.orientX = (char)EEPROM.read(eeprom.mag2boardXaxis);
  mag.dirX = (uint8_t)EEPROM.read(eeprom.mag2boardXsign);
  mag.orientY = (char)EEPROM.read(eeprom.mag2boardYaxis);
  mag.dirY = (uint8_t)EEPROM.read(eeprom.mag2boardYsign);
  mag.orientZ = (char)EEPROM.read(eeprom.mag2boardZaxis);
  mag.dirZ = (uint8_t)EEPROM.read(eeprom.mag2boardZsign);
  highG.orientX = (char)EEPROM.read(eeprom.highG2boardXaxis);
  highG.dirX = (uint8_t)EEPROM.read(eeprom.highG2boardXsign);
  highG.orientY = (char)EEPROM.read(eeprom.highG2boardYaxis);
  highG.dirY = (uint8_t)EEPROM.read(eeprom.highG2boardYsign);
  highG.orientZ = (char)EEPROM.read(eeprom.highG2boardZaxis);
  highG.dirZ = (uint8_t)EEPROM.read(eeprom.highG2boardZsign);
  
  //use the accelerometer to determine which of the 6 possible system orientations is present
  //Accelerometer Z-axis is pointed up
  if(abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX) && accel.biasZ > 0){
    if(accel.orientZ == 'Z'){orientCase = 1;}//Flight Computer Z-axis is pointed to the nose
    if(accel.orientY == 'Z'){orientCase = 3;}//Flight Computer Y-axis is pointed to the nose
    if(accel.orientX == 'Z'){orientCase = 5;}}//Flight Computer X-axis is pointed to the nose
  //Accelerometer Z-axis is pointed down
  if(abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX) && accel.biasZ < 0){
    if(accel.orientZ == 'Z'){orientCase = 2;}//Flight Computer Z-axis is pointed to the ground
    if(accel.orientY == 'Z'){orientCase = 4;}//Flight Computer Y-axis is pointed to the ground
    if(accel.orientX == 'Z'){orientCase = 6;}}//Flight Computer X-axis is pointed to the ground
  //Accelerometer Y-axis is pointed up
  if(abs(accel.biasY) > abs(accel.biasZ) && abs(accel.biasY) > abs(accel.biasX) && accel.biasY > 0){
    if(accel.orientZ == 'Y'){orientCase = 1;}//Flight Computer Z-axis is pointed to the nose
    if(accel.orientY == 'Y'){orientCase = 3;}//Flight Computer Y-axis is pointed to the nose
    if(accel.orientX == 'Y'){orientCase = 5;}}//Flight Computer X-axis is pointed to the nose
  //Accelerometer Y-axis is pointed down
  if(abs(accel.biasY) > abs(accel.biasZ) && abs(accel.biasY) > abs(accel.biasX) && accel.biasY < 0){
    if(accel.orientZ == 'Y'){orientCase = 2;}//Flight Computer Z-axis is pointed to the ground
    if(accel.orientY == 'Y'){orientCase = 4;}//Flight Computer Y-axis is pointed to the ground
    if(accel.orientX == 'Y'){orientCase = 6;}}//Flight Computer X-axis is pointed to the ground
  //Accelerometer X-axis is pointed up
  if(abs(accel.biasX) > abs(accel.biasZ) && abs(accel.biasX) > abs(accel.biasY) && accel.biasX > 0){
    if(accel.orientZ == 'X'){orientCase = 1;}//Flight Computer Z-axis is pointed to the nose
    if(accel.orientY == 'X'){orientCase = 3;}//Flight Computer Y-axis is pointed to the nose
    if(accel.orientX == 'X'){orientCase = 5;}}//Flight Computer X-axis is pointed to the nose
  //Accelerometer X-axis is pointed down
  if(abs(accel.biasX) > abs(accel.biasZ) && abs(accel.biasX) > abs(accel.biasY) && accel.biasX < 0){
    if(accel.orientZ == 'X'){orientCase = 2;}//Flight Computer Z-axis is pointed to the ground
    if(accel.orientY == 'X'){orientCase = 4;}//Flight Computer Y-axis is pointed to the ground
    if(accel.orientX == 'X'){orientCase = 6;}}//Flight Computer X-axis is pointed to the ground

  //set the orientation variables
  switch (orientCase) {

    case 1: //Flight Computer Z-axis is pointed to the nose

      Serial.println("Flight Computer Z-axis is pointed to the nose");
      
      //Do nothing, the axes are correctly aligned
      
      break;

    case 2: //Flight Computer Z-axis is pointed to the tail

      Serial.println("Flight Computer Z-axis is pointed to the tail");

      //all axes are still aligned
      
      //flip the direction of Z
      flipAxis('Z');
      
      //flip the direction of Y
      flipAxis('Y');      
      
      break;

    case 3: //Flight Computer Y-axis is pointed to the nose

      Serial.println("Flight Computer Y-axis is pointed to the nose");

      //flip the direction of Z
      flipAxis('Z');
      
      //exchange Y and Z axes
      exchangeAxes('Y','Z');
      
      break;

    case 4: //Flight Computer Y-axis is pointed to the tail

      Serial.println("Flight Computer Y-axis is pointed to the tail");

      //flip the direction of Y
      flipAxis('Y');
      
      //exchange Y and Z axes
      exchangeAxes('Y','Z');
      
      break;

    case 5: //Flight Computer X-axis is pointed to the nose

      Serial.println("Flight Computer IMU X-axis is pointed to the nose");

      //flip the direction of Z
      flipAxis('Z'); 
      
      //exchange X and Z axes
      exchangeAxes('X','Z');
    
      break;

    case 6: //Flight Computer X-axis is pointed to the tail

      Serial.println("Flight Computer X-axis is pointed to the tail");

      //flip the direction of X
      flipAxis('X');
      
      //exchange X and Z axes
      exchangeAxes('X','Z');
    
      break;

    default: //error has occured
      Serial.println("Error in determining flight computer orientation");
      break;
  }//end case

  //correct IMU bias for 1G
  if(accel.orientX == 'Z'){accel.biasX -= accel.dirX * g;}
  if(accel.orientY == 'Z'){accel.biasY -= accel.dirY * g;}
  if(accel.orientZ == 'Z'){accel.biasZ -= accel.dirZ * g;}
  
  //correct high-G bias for 1G
  if(highG.orientX == 'Z'){highG.biasX -= highG.dirX * high1G;}
  if(highG.orientY == 'Z'){highG.biasY -= highG.dirY * high1G;}
  if(highG.orientZ == 'Z'){highG.biasZ -= highG.dirZ * high1G;}

  //write to EEPROM
  //Accelerometer
  EEPROM.update(eeprom.accelXsign, accel.dirX);
  EEPROM.update(eeprom.accelXptr, accel.orientX);
  EEPROM.update(eeprom.accelYsign, accel.dirY);
  EEPROM.update(eeprom.accelYptr, accel.orientY);
  EEPROM.update(eeprom.accelZsign, accel.dirZ);
  EEPROM.update(eeprom.accelZptr, accel.orientZ);
  //Gyroscope
  EEPROM.update(eeprom.gyroXsign, gyro.dirX);
  EEPROM.update(eeprom.gyroXptr, gyro.orientX);
  EEPROM.update(eeprom.gyroYsign, gyro.dirY);
  EEPROM.update(eeprom.gyroYptr, gyro.orientY);
  EEPROM.update(eeprom.gyroZsign, gyro.dirZ);
  EEPROM.update(eeprom.gyroZptr, gyro.orientZ);
  //Magnetometer
  EEPROM.update(eeprom.magXsign, mag.dirX);
  EEPROM.update(eeprom.magXptr, mag.orientX);
  EEPROM.update(eeprom.magYsign, mag.dirY);
  EEPROM.update(eeprom.magYptr, mag.orientY);
  EEPROM.update(eeprom.magZsign, mag.dirZ);
  EEPROM.update(eeprom.magZptr, mag.orientZ);
  //High-G Accelerometer
  EEPROM.update(eeprom.highGxSign, highG.dirX);
  EEPROM.update(eeprom.highGxPtr, highG.orientX);
  EEPROM.update(eeprom.highGySign, highG.dirY);
  EEPROM.update(eeprom.highGyPtr, highG.orientY);
  EEPROM.update(eeprom.highGzSign, highG.dirZ);
  EEPROM.update(eeprom.highGzPtr, highG.orientZ);

  Serial.println("Calibration Complete!");

  }//end void

void exchangeAxes(char A, char B){

  //accelerometer
  if(accel.orientX == A){
    accel.orientX = B;
    if(accel.orientY == B){accel.orientY = A;}
    if(accel.orientZ == B){accel.orientZ = A;}}
  else if(accel.orientY == A){
    accel.orientY = B;
    if(accel.orientX == B){accel.orientX = A;}
    if(accel.orientZ == B){accel.orientZ = A;}}
  else if(accel.orientZ == A){
    accel.orientZ = B;
    if(accel.orientX == B){accel.orientX = A;}
    if(accel.orientY == B){accel.orientY = A;}}

  //gyroscope
  if(gyro.orientX == A){
    gyro.orientX = B;
    if(gyro.orientY == B){gyro.orientY = A;}
    if(gyro.orientZ == B){gyro.orientZ = A;}}
  else if(gyro.orientY == A){
    gyro.orientY = B;
    if(gyro.orientX == B){gyro.orientX = A;}
    if(gyro.orientZ == B){gyro.orientZ = A;}}
  else if(gyro.orientZ == A){
    gyro.orientZ = B;
    if(gyro.orientX == B){gyro.orientX = A;}
    if(gyro.orientY == B){gyro.orientY = A;}}

  //magnetometer
  if(mag.orientX == A){
    mag.orientX = B;
    if(mag.orientY == B){mag.orientY = A;}
    if(mag.orientZ == B){mag.orientZ = A;}}
  else if(mag.orientY == A){
    mag.orientY = B;
    if(mag.orientX == B){mag.orientX = A;}
    if(mag.orientZ == B){mag.orientZ = A;}}
  else if(mag.orientZ == A){
    mag.orientZ = B;
    if(mag.orientX == B){mag.orientX = A;}
    if(mag.orientY == B){mag.orientY = A;}}

  //high-G accelerometer
  if(highG.orientX == A){
    highG.orientX = B;
    if(highG.orientY == B){highG.orientY = A;}
    if(highG.orientZ == B){highG.orientZ = A;}}
  else if(highG.orientY == A){
    highG.orientY = B;
    if(highG.orientX == B){highG.orientX = A;}
    if(highG.orientZ == B){highG.orientZ = A;}}
  else if(highG.orientZ == A){
    highG.orientZ = B;
    if(highG.orientX == B){highG.orientX = A;}
    if(highG.orientY == B){highG.orientY = A;}}
}//end exchangeAxes

void flipAxis(char A){

  //accelerometer
  if(accel.orientX == A){accel.dirX *= -1;}
  if(accel.orientY == A){accel.dirY *= -1;}
  if(accel.orientZ == A){accel.dirZ *= -1;}
  //gyroscope
  if(gyro.orientX == A){gyro.dirX *= -1;}
  if(gyro.orientY == A){gyro.dirY *= -1;}
  if(gyro.orientZ == A){gyro.dirZ *= -1;}
  //magnetometer
  if(mag.orientX == A){mag.dirX *= -1;}
  if(mag.orientY == A){mag.dirY *= -1;}
  if(mag.orientZ == A){mag.dirZ *= -1;}
  //high-G accelerometer
  if(highG.orientX == A){highG.dirX *= -1;}
  if(highG.orientY == A){highG.dirY *= -1;}
  if(highG.orientZ == A){highG.dirZ *= -1;}
  
}//end flipAxis

void readOrientation(){

  //EEPROM stores the pointers and directions for the real-world axes as determined from calibration
  //accel.orientZ contains a character that represents the accelerometer axis that is aligned to the realworld Z-axis (rocket direction of travel)
  //thus if accel.orientZ == 'X' and accel.dirX == -1, then we know that the accelerometer X-axis is pointed towards the ground
  //*****************************************************
  //Accelerometer
  //*****************************************************
  //X-Axis
  accel.orientX = (char)EEPROM.read(eeprom.accelXptr);
  accel.dirX = (int8_t)EEPROM.read(eeprom.accelXsign);
  switch(accel.orientX){
    case 'X':
      accel.ptrX = &accel.rawX;
      accel.ptrXsign = &accel.dirX;
      break;
    case 'Y':
      accel.ptrY = &accel.rawX;
      accel.ptrYsign = &accel.dirX;
      break;
    case 'Z':
      accel.ptrZ = &accel.rawX;
      accel.ptrZsign = &accel.dirX;
      break;}
  //Y-Axis
  accel.orientY = (char)EEPROM.read(eeprom.accelYptr);
  accel.dirY = (int8_t)EEPROM.read(eeprom.accelYsign);
  switch(accel.orientY){
    case 'X':
      accel.ptrX = &accel.rawY;
      accel.ptrXsign = &accel.dirY;
      break;
    case 'Y':
      accel.ptrY = &accel.rawY;
      accel.ptrYsign = &accel.dirY;
      break;
    case 'Z':
      accel.ptrZ = &accel.rawY;
      accel.ptrZsign = &accel.dirY;
      break;}
  //Z-Axis
  accel.orientZ = (char)EEPROM.read(eeprom.accelZptr);
  accel.dirZ = (int8_t)EEPROM.read(eeprom.accelZsign);
  switch(accel.orientZ){
    case 'X':
      accel.ptrX = &accel.rawZ;
      accel.ptrXsign = &accel.dirZ;
      break;
    case 'Y':
      accel.ptrY = &accel.rawZ;
      accel.ptrYsign = &accel.dirZ;
      break;
    case 'Z':
      accel.ptrZ = &accel.rawZ;
      accel.ptrZsign = &accel.dirZ;
      break;}
      
  //*****************************************************
  //Gyroscope
  //*****************************************************
  //X-Axis
  gyro.orientX = (char)EEPROM.read(eeprom.gyroXptr);
  gyro.dirX = (int8_t)EEPROM.read(eeprom.gyroXsign);
  switch(gyro.orientX){
    case 'X':
      gyro.ptrX = &gyro.rawX;
      gyro.ptrXsign = &gyro.dirX;
      break;
    case 'Y':
      gyro.ptrY = &gyro.rawX;
      gyro.ptrYsign = &gyro.dirX;
      break;
    case 'Z':
      gyro.ptrZ = &gyro.rawX;
      gyro.ptrZsign = &gyro.dirX;
      break;}
  //Y-Axis
  gyro.orientY = (char)EEPROM.read(eeprom.gyroYptr);
  gyro.dirY = (int8_t)EEPROM.read(eeprom.gyroYsign);
  switch(gyro.orientY){
    case 'X':
      gyro.ptrX = &gyro.rawY;
      gyro.ptrXsign = &gyro.dirY;
      break;
    case 'Y':
      gyro.ptrY = &gyro.rawY;
      gyro.ptrYsign = &gyro.dirY;
      break;
    case 'Z':
      gyro.ptrZ = &gyro.rawY;
      gyro.ptrZsign = &gyro.dirY;
      break;}
  //Z-Axis
  gyro.orientZ = (char)EEPROM.read(eeprom.gyroZptr);
  gyro.dirZ = (int8_t)EEPROM.read(eeprom.gyroZsign);
  switch(gyro.orientZ){
    case 'X':
      gyro.ptrX = &gyro.rawZ;
      gyro.ptrXsign = &gyro.dirZ;
      break;
    case 'Y':
      gyro.ptrY = &gyro.rawZ;
      gyro.ptrYsign = &gyro.dirZ;
      break;
    case 'Z':
      gyro.ptrZ = &gyro.rawZ;
      gyro.ptrZsign = &gyro.dirZ;
      break;}

  //*****************************************************
  //Magnetometer
  //*****************************************************
  //X-Axis
  mag.orientX = (char)EEPROM.read(eeprom.magXptr);
  mag.dirX = (int8_t)EEPROM.read(eeprom.magXsign);
  switch(mag.orientX){
    case 'X':
      mag.ptrX = &mag.rawX;
      mag.ptrXsign = &mag.dirX;
      break;
    case 'Y':
      mag.ptrY = &mag.rawX;
      mag.ptrYsign = &mag.dirX;
      break;
    case 'Z':
      mag.ptrZ = &mag.rawX;
      mag.ptrZsign = &mag.dirX;
      break;}
  //Y-Axis
  mag.orientY = (char)EEPROM.read(eeprom.magYptr);
  mag.dirY = (int8_t)EEPROM.read(eeprom.magYsign);
  switch(mag.orientY){
    case 'X':
      mag.ptrX = &mag.rawY;
      mag.ptrXsign = &mag.dirY;
      break;
    case 'Y':
      mag.ptrY = &mag.rawY;
      mag.ptrYsign = &mag.dirY;
      break;
    case 'Z':
      mag.ptrZ = &mag.rawY;
      mag.ptrZsign = &mag.dirY;
      break;}
  //Z-Axis
  mag.orientZ = (char)EEPROM.read(eeprom.magZptr);
  mag.dirZ = (int8_t)EEPROM.read(eeprom.magZsign);
  switch(mag.orientZ){
    case 'X':
      mag.ptrX = &mag.rawZ;
      mag.ptrXsign = &mag.dirZ;
      break;
    case 'Y':
      mag.ptrY = &mag.rawZ;
      mag.ptrYsign = &mag.dirZ;
      break;
    case 'Z':
      mag.ptrZ = &mag.rawZ;
      mag.ptrZsign = &mag.dirZ;
      break;}

  //*****************************************************
  //High-G Accelerometer
  //*****************************************************
  //X-Axis
  highG.orientX = (char)EEPROM.read(eeprom.highGxPtr);
  highG.dirX = (int8_t)EEPROM.read(eeprom.highGxSign);
  switch(highG.orientX){
    case 'X':
      highG.ptrX = &highG.rawX;
      highG.ptrXsign = &highG.dirX;
      break;
    case 'Y':
      highG.ptrY = &highG.rawX;
      highG.ptrYsign = &highG.dirX;
      break;
    case 'Z':
      highG.ptrZ = &highG.rawX;
      highG.ptrZsign = &highG.dirX;
      break;}
  //Y-Axis
  highG.orientY = (char)EEPROM.read(eeprom.highGyPtr);
  highG.dirY = (int8_t)EEPROM.read(eeprom.highGySign);
  switch(highG.orientY){
    case 'X':
      highG.ptrX = &highG.rawY;
      highG.ptrXsign = &highG.dirY;
      break;
    case 'Y':
      highG.ptrY = &highG.rawY;
      highG.ptrYsign = &highG.dirY;
      break;
    case 'Z':
      highG.ptrZ = &highG.rawY;
      highG.ptrZsign = &highG.dirY;
      break;}
  //Z-Axis
  highG.orientZ = (char)EEPROM.read(eeprom.highGzPtr);
  highG.dirZ = (int8_t)EEPROM.read(eeprom.highGzSign);
  switch(highG.orientZ){
    case 'X':
      highG.ptrX = &highG.rawZ;
      highG.ptrXsign = &highG.dirZ;
      break;
    case 'Y':
      highG.ptrY = &highG.rawZ;
      highG.ptrYsign = &highG.dirZ;
      break;
    case 'Z':
      highG.ptrZ = &highG.rawZ;
      highG.ptrZsign = &highG.dirZ;
      break;}
}//end readOrientation

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
  baro.tempOffset = 0.0F;
  for(int i = 0; i < 300; i++){
    getBaro();
    if(baro.newTemp){tempSum += baro.temperature;baro.newTemp = false;Serial.println(baro.temperature, 1);}
    else{i--;}
    delayMicroseconds(baro.timeBtwnSamp);}
  
  baro.tempOffset = (tempSum / 300) - userInput;
  Serial.println(baro.tempOffset, 1);

  //Print the sample conditions
  Serial.println(F("Temperature calibration complete"));
  Serial.print(F("Sampled atmospheric temperature: ")); Serial.println(tempSum/300, 1);
  Serial.print("Temperature Offset: ");Serial.println(baro.tempOffset, 1);
      
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
  baro.pressOffset = 0.0F;
  float baroSum = 0.0F;
  for(int i = 0; i < 300; i++){
    getBaro();
    if(baro.newSamp){baroSum += baro.pressure;baro.newSamp = false;Serial.println(baro.pressure, 2);}
    else{i--;};    
    delayMicroseconds(baro.timeBtwnSamp);}

  baro.pressOffset = (baroSum / 300) - userInput;

  //Print the sample conditions
  Serial.println(F("Barometer calibration Complete"));
  Serial.print(F("Sampled barometric pressure: ")); Serial.println(baroSum/300, 2);
  Serial.print("Barometric Offset: ");Serial.print(baro.pressOffset, 2); Serial.println(F(" hPa"));

  //write to eeprom
  floatUnion.val = baro.tempOffset;
  for(byte i = 0; i < 4; i++){EEPROM.update(eeprom.baroTempOffset + i, floatUnion.Byte[i]);}
  floatUnion.val = baro.pressOffset;
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
