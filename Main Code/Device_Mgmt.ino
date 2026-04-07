//-----------------------------------------------
//Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
//22 OCT 23: Initial Build
//19 NOV 23: Increased devices supported
//29 SEP 24: Established polymorphic variables for function pointers
//***************************************************************************
//The system interfaces with the sensors through generic "begin" and "get" functions, i.e. beginAccel(), getAccel(),...
//Each function can either use an external library or use optimized drivers provided in Sensor_Drivers.ino
//The "begin" function is used to initialize the sensor and put it at the necessary gain and oupt data rates
//The "get" function is used to retrieve the raw data from the sensor.  Raw sensor data is necessary to use the built-in calilbration
//The "orient" function is used to remove bias from the sensor data and orient it to the proper axes. Called after "get"
//***************************************************************************
//----------------------------
//LIST OF FUNCTIONS
//----------------------------
//setPolyMorphs(): sets the function pointers for the devices
//orientAccel(): removes bias from raw sensor data and orients corrected data to the proper axes
//orientGyro(): removes bias from raw sensor data and orients corrected data to the proper axes
//orientMag(): removes bias from raw sensor data and orients corrected data to the proper axes
//orientHighG(): removes bias from raw sensor data and orients corrected data to the proper axes
//GNSSconfig(): configures the GNSS unit for flight conditions
//GNSSpowerSaveMode(): reduces power consumption post-flight
//dummyVoid(): executes no code when a routine or sensor is absent
//dummyBool(): executes no code and returns true when a routine or sensor is absent

void setPolyMorphs() {

  //To enable faster execution and simplicity of code, the "begin" and "get" sensor functions are pointers

  //accelerometer pointers
  switch (sensors.accel) {

    case 1:
      beginAccel = beginLSM303_A;
      getAccel = getLSM303_A;
      break;

    case 2:
      beginAccel = beginGyro = beginLSM9DS1_AG;
      getAccel = getGyro = getLSM9DS1_AG;
      break;

    case 3:
      beginAccel = beginGyro  = beginLSM6DS33;
      getAccel = getGyro = getLSM6DS33;
      break;

    case 4:
      beginAccel = beginGyro  = beginMPU6050;
      getAccel = getGyro = getMPU6050;
      break;

    case 5:
      beginAccel = beginGyro  = beginMPU9250_AG;
      getAccel = getGyro = getMPU9250_AG;
      break;
    
    case 6:
      beginAccel = beginGyro  = beginLSM6DSOX;
      getAccel = getGyro = getLSM6DSOX;
      break;

    case 7://External Library          
      beginAccel = beginExternalAccel;
      getAccel = getExternalAccel;
      break;
    
    case 8:
      beginAccel = beginGyro = beginLSM6DS3TR;
      getAccel = getGyro = getLSM6DS3TR;
      break;
      
    default:
      beginAccel = beginExternalAccel;
      getAccel = getExternalAccel;
      break;}

  //gyroscope pointers
  switch (sensors.gyro) {

    case 1:
      beginGyro = beginL3GD20H;
      getGyro = getL3GD20H;
      break;
    
    case 7://External Library
      beginGyro = beginExternalGyro;
      getGyro = getExternalGyro;
      break;

    default: //all other casesare 
      beginGyro = dummyBool;
      getGyro = dummyVoid;
      break;}

  //High-G Accelerometer Pointers
  switch (sensors.highG) {

    case 1:
      beginHighG = beginADS1115;
      getHighG = getADS1115;
      break;

    case 2:
      beginHighG = beginH3LIS331DL;
      getHighG = getH3LIS331DL;
      break;

    case 3:
      beginHighG = beginADXL377;
      getHighG = getADXL377;
      break;

    case 4:
      beginHighG = beginExternalHighG;
      getHighG = getExternalHighG;
      break;

    default:
      beginHighG = dummyBool;
      getHighG = dummyVoid;
      break;}

  //magnetometer pointers
  switch (sensors.mag) {

    case 1:
      beginMag = beginLSM303_M;
      getMag = getLSM303_M;
      break;

    case 2:
      beginMag = beginLSM9DS1_M;
      getMag = getLSM9DS1_M;
      break;

    case 3:
      beginMag = beginLIS3MDL;
      getMag = getLIS3MDL;
      break;

    case 4:
      beginMag = beginMPU9250_M;
      getMag = getMPU9250_M;
      break;
    
    case 5://External Library
      beginMag = beginExternalMag;
      getMag = getExternalMag;
      break;
      
    default:
      beginMag = dummyBool;
      getMag = dummyVoid;
      break;}

  //set barometer pointers
  switch (sensors.baro) {

    case 1:
      beginBaro = beginBMP180;
      getBaro = getBMP180;
      break;

    case 2:
      beginBaro = beginMPL3115A2;
      getBaro = getMPL3115A2;
      break;

    case 3:
      beginBaro = beginBMP280;
      getBaro = getBMP280;
      break;

    case 4:
      beginBaro = beginBMP388;
      getBaro = getBMP388;
      break;

    case 5:
      beginBaro = beginMS56XX;
      getBaro = getMS56XX;
      break;

    case 6:
      beginBaro = beginMS56XX;
      getBaro = getMS56XX;
      break;

    case 7:
      beginBaro = beginLPS25H;
      getBaro = getLPS25H;
      break;

    case 8:
      beginBaro = beginExternalBaro;
      getBaro = getExternalBaro;
      break;

    default:
      beginBaro = beginExternalBaro;
      getBaro = getExternalBaro;
      break;}

  //set radio pointers
  switch (sensors.radio){

    case 1:
      beginRadio = beginSX127X;
      setRadioFreq = setFreqSX127X;
      setRadioPWR = setPwrSX127X;
      radioSleep = sleepSX127X;
      radioSendPkt = sendPktSX127X;
      break;

    case 2:
      beginRadio = beginRFD900;
      setRadioFreq = dummyBool1;
      setRadioPWR = dummyBool2;
      radioSleep = dummyVoid;
      radioSendPkt = sendPktRFD900;
      break;

    case 3:
      beginRadio = beginExternalRadio;
      setRadioFreq = setExternalRadioFreq;
      setRadioPWR = setExternalRadioPwr;
      radioSleep = externalRadioSleep;
      radioSendPkt = sendExternalRadioDataPkt;
      break;

    default://no radio
      beginRadio = dummyBool;
      setRadioFreq = dummyBool1;
      setRadioPWR = dummyBool2;
      radioSleep = dummyVoid;
      radioSendPkt = dummyBool3;
      break;}

  //set GNSS device pointers
  switch (sensors.GNSS) {

    case 1: 
      GNSSrestorDefaults = UBLOXrestorDefaults;
      GNSSconfig = UBLOXconfig;
      GNSSpowerSave = UBLOXpowerSave;
      break;

    case 2:
      GNSSrestorDefaults = UBLOXrestorDefaults;
      GNSSconfig = UBLOXconfig;
      GNSSpowerSave = UBLOXpowerSave;
      break;

    case 3:
      GNSSrestorDefaults = UBLOXrestorDefaults;
      GNSSconfig = UBLOXconfig;
      GNSSpowerSave = UBLOXpowerSave;
      break;

    case 4:
      GNSSrestorDefaults = dummyVoid;
      GNSSconfig = ExternalGNSSconfig;
      GNSSpowerSave = externalGNSS_PSM;
      break;

    default:
      break;}

}//end setPolyMorphs()

void orientAccel() {

  //after getting the raw sensor data, we need to remove the bias and orient the raw data to the real-world axes

  //indicate new sample
  accel.newSamp = true;

  //remove bias
  accel.rawX -= accel.biasX;
  accel.rawY -= accel.biasY;
  accel.rawZ -= accel.biasZ;

  //orient sensor data
  accel.x = *accel.ptrX * *accel.ptrXsign;
  accel.y = *accel.ptrY * *accel.ptrYsign;
  accel.z = *accel.ptrZ * *accel.ptrZsign;}

void orientMag() {

  //indicate new sample
  mag.newSamp = true;

  //remove bias
  mag.rawX -= mag.biasX;
  mag.rawY -= mag.biasY;
  mag.rawZ -= mag.biasZ;

  //translate sensor data
  mag.x = *mag.ptrX * *mag.ptrXsign;
  mag.y = *mag.ptrY * *mag.ptrYsign;
  mag.z = *mag.ptrZ * *mag.ptrZsign;}

void orientGyro() {

  //indicate new sample
  gyro.newSamp = true;

  //remove bias
  gyro.rawX -= gyro.biasX;
  gyro.rawY -= gyro.biasY;
  gyro.rawZ -= gyro.biasZ;

  //orient sensor data
  gyro.x = *gyro.ptrX * *gyro.ptrXsign;
  gyro.y = *gyro.ptrY * *gyro.ptrYsign;
  gyro.z = *gyro.ptrZ * *gyro.ptrZsign;}

void orientHighG() {

  //indicate new sample
  highG.newSamp = true;

  //remove bias
  highG.rawX -= highG.biasX;
  highG.rawY -= highG.biasY;
  highG.rawZ -= highG.biasZ;

  //orient sensor data
  highG.x = *highG.ptrX * *highG.ptrXsign;
  highG.y = *highG.ptrY * *highG.ptrYsign;
  highG.z = *highG.ptrZ * *highG.ptrZsign;}  

void dummyVoid() { }
bool dummyBool() {return true;}
bool dummyBool1(float freq) {return true;}
bool dummyBool2(int8_t pwr) {return true;}
bool dummyBool3(uint8_t* data, uint8_t len){return true;}