//Sketch adapted for the Adafruit 10DoF IMU board
//By Maj Bryan Sparkman
//Sensor Package: LSM303DLHC, L3GD20, BMP180, ADS1115, ADXL377
//-----------Change Log------------
//26 Nov 17: Version 1 created

//***************************************************************************
//Common I2C Communication Functions
//***************************************************************************
  
void write8(byte address, byte reg, byte value){
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();}

static void read16(uint8_t address, byte reg, uint16_t *value){
  Wire.beginTransmission((uint8_t)address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address, (byte)2);
  *value = (Wire.read() << 8) | Wire.read();
  Wire.endTransmission();}

static void readS16(byte address, byte reg, int16_t *value){
  uint16_t i;
  read16(address, reg, &i);
  *value = (int16_t)i;}
  
void readSensor(byte address, byte reg, byte sensor){
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)6);
  while(Wire.available() < 6){};

  uint8_t xLow  = Wire.read();
  uint8_t xHigh = Wire.read();
  uint8_t yLow  = Wire.read();
  uint8_t yHigh = Wire.read();
  uint8_t zLow  = Wire.read();
  uint8_t zHigh = Wire.read();

  switch(sensor){

    case 1://accelerometer
      accelX = (int16_t)(xLow | (xHigh << 8)) >> 4;
      accelY = (int16_t)(yLow | (yHigh << 8)) >> 4;
      accelZ = (int16_t)(zLow | (zHigh << 8)) >> 4;
      break;

    case 2://magnetometer (high & low reversed)
      magX = (int16_t)(xHigh | ((int16_t)xLow << 8));
      magY = (int16_t)(yHigh | ((int16_t)yLow << 8));
      magZ = (int16_t)(zHigh | ((int16_t)zLow << 8));
      break;

    case 3://gyro
      gyroX = (int16_t)(xLow | (xHigh << 8));
      gyroY = (int16_t)(yLow | (yHigh << 8));
      gyroZ = (int16_t)(zLow | (zHigh << 8));
      break;
  }}//end void
  
//***************************************************************************
//LSM303 Accelerometer
//***************************************************************************

void beginAccel(){

  #define LSM303_ADDRESS_ACCEL               (0x32 >> 1)
  #define LSM303_REGISTER_ACCEL_CTRL_REG4_A  (0x23)
  #define LSM303_REGISTER_ACCEL_CTRL_REG1_A  (0x20)
  
  //set gain to 24G
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x38);  

  //set data rate: 400Hz for Metro, 1600Hz for Teensy
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x97);
  #else
    write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x77);
  #endif
  }

void getAccel(){
  #define LSM303_REGISTER_ACCEL_OUT_X_L_A  (0x28)
  readSensor(LSM303_ADDRESS_ACCEL, (LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80), (byte)1);}

//***************************************************************************
//LSM303 Magnetometer
//***************************************************************************
void beginMag(){

  #define LSM303_ADDRESS_MAG            (0x3C >> 1)
  #define LSM303_REGISTER_MAG_MR_REG_M  (0x02)
  #define LSM303_REGISTER_MAG_CRB_REG_M (0x01)
  #define LSM303_REGISTER_MAG_CRA_REG_M (0x00)
  #define LSM303_MAGGAIN_1_3            (0x20)
  #define LSM303_MAGRATE_15             (0x10)
  
  //enable magnetometer
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

  //set gain
  //_lsm303Mag_Gauss_LSB_XY = 1100;
  //_lsm303Mag_Gauss_LSB_Z  = 980;
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, LSM303_MAGGAIN_1_3);

  //set data rate
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, LSM303_MAGRATE_15);
  
  //enable the temperature sensor and set data rate
  /*#define LSM303_REGISTER_CRA_REG_M (0x00)
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_CRA_REG_M, 0b10010000);*/
  }//end beginMag

void getMag(){
  #define LSM303_REGISTER_MAG_OUT_X_H_M  (0x03)
  readSensor(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M, (byte)2);}

/*void getBoardTemp(){
  #define LSM303_REGISTER_MAG_TEMP_OUT_H_M (0x31)
  Wire.beginTransmission(LSM303_ADDRESS_MAG);
  Wire.write(LSM303_REGISTER_MAG_TEMP_OUT_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM303_ADDRESS_MAG, (byte)2);
  while(Wire.available() < 2){};

  byte boardTempHi = Wire.read();
  byte boardTempLo = Wire.read();
  int16_t sensorTemp;
  float boardTemp;
  sensorTemp = (int16_t)(boardTempLo | (boardTempHi << 8)) >> 4;}
  boardTemp = sensorTemp * 0.13762 + 19.54762;*/
  
//***************************************************************************
//L3GD20 Gyroscope
//***************************************************************************
void beginGyro(){

  #define L3GD20_ADDRESS           0x6B
  #define GYRO_REGISTER_CTRL_REG1  0x20
  #define GYRO_REGISTER_CTRL_REG4  0x23
  
  //reset then enable
  write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x00);
  write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0x0F);
  
  //set gain
  write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0x20);

  //set data rate: metro 400Hz, Teensy 800Hz
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0xdf);
  #else
    write8(L3GD20_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0xaf);
  #endif
  }

void getGyro(){
  #define GYRO_REGISTER_OUT_X_L (0x28)
  readSensor(L3GD20_ADDRESS, (GYRO_REGISTER_OUT_X_L | 0x80), (byte)3);}

//***************************************************************************
//BMP180 Pressure Sensor
//***************************************************************************
/***************************************************************************
  This is a library for the BMP085 pressure sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
***************************************************************************/
#define BMP085_ADDRESS  (0x77)
#define _bmp085Mode     (3)

struct bmp085_calib_data
{
      int16_t  ac1;
      int16_t  ac2;
      int16_t  ac3;
      uint16_t ac4;
      uint16_t ac5;
      uint16_t ac6;
      int16_t  b1;
      int16_t  b2;
      int32_t  b5;
      int16_t  mb;
      int16_t  mc;
      int16_t  md;
    } ;
static bmp085_calib_data _bmp085_coeffs;

static void read8(byte reg, uint8_t *value){
  Wire.beginTransmission((uint8_t)BMP085_ADDRESS);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP085_ADDRESS, (byte)1);
  *value = Wire.read();
  Wire.endTransmission();}

static void beginPressure(void){
  
  #define BMP085_REGISTER_CAL_AC1 0xAA
  #define BMP085_REGISTER_CAL_AC2 0xAC
  #define BMP085_REGISTER_CAL_AC3 0xAE
  #define BMP085_REGISTER_CAL_AC4 0xB0
  #define BMP085_REGISTER_CAL_AC5 0xB2
  #define BMP085_REGISTER_CAL_AC6 0xB4
  #define BMP085_REGISTER_CAL_B1  0xB6
  #define BMP085_REGISTER_CAL_B2  0xB8
  #define BMP085_REGISTER_CAL_MB  0xBA
  #define BMP085_REGISTER_CAL_MC  0xBC
  #define BMP085_REGISTER_CAL_MD  0xBE

  //Read calibration coefficients
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_AC1, &_bmp085_coeffs.ac1);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_AC2, &_bmp085_coeffs.ac2);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_AC3, &_bmp085_coeffs.ac3);
  read16(BMP085_ADDRESS, BMP085_REGISTER_CAL_AC4, &_bmp085_coeffs.ac4);
  read16(BMP085_ADDRESS, BMP085_REGISTER_CAL_AC5, &_bmp085_coeffs.ac5);
  read16(BMP085_ADDRESS, BMP085_REGISTER_CAL_AC6, &_bmp085_coeffs.ac6);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_B1, &_bmp085_coeffs.b1);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_B2, &_bmp085_coeffs.b2);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_MB, &_bmp085_coeffs.mb);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_MC, &_bmp085_coeffs.mc);
  readS16(BMP085_ADDRESS, BMP085_REGISTER_CAL_MD, &_bmp085_coeffs.md);}

void initiateTemp(){
  #define BMP085_REGISTER_CONTROL         0xF4
  #define BMP085_REGISTER_READTEMPCMD     0x2E
  #define BMP085_REGISTER_READPRESSURECMD 0x34
  
  write8(BMP085_ADDRESS, BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);}

void initiatePressure(float *temp){
  int32_t  ut = 0;
  int32_t UT, X1, X2, B5;     // following ds convention
  float t;

  //Read ucompensated temperature
  #define BMP085_REGISTER_TEMPDATA 0xF6
  uint16_t rt;
  read16(BMP085_ADDRESS, BMP085_REGISTER_TEMPDATA, &rt);
  ut = rt;

  //Calculate true temperature
  X1 = (ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5) >> 15;
  X2 = ((int32_t)_bmp085_coeffs.mc << 11) / (X1+(int32_t)_bmp085_coeffs.md);
  _bmp085_coeffs.b5 = X1 + X2;
  t = (_bmp085_coeffs.b5+8) >> 4;
  t /= 10;
  *temp = t;
  
  //Initiate Pressure
  write8(BMP085_ADDRESS, BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6));}

void getPressure(float *pressure){

  uint8_t  p8;
  uint16_t p16;
  int32_t  up = 0, compp = 0;
  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;

  //Read uncompensated pressure
  #define BMP085_REGISTER_PRESSUREDATA 0xF6
  read16(BMP085_ADDRESS, BMP085_REGISTER_PRESSUREDATA, &p16);
  up = (uint32_t)p16 << 8;
  read8(BMP085_REGISTER_PRESSUREDATA+2, &p8);
  up += p8;
  up >>= (8 - _bmp085Mode);

  //Calculate true pressure
  b6 = _bmp085_coeffs.b5 - 4000;
  x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) _bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
  x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
  x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (_bmp085_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> _bmp085Mode));

  if (b7 < 0x80000000){p = (b7 << 1) / b4;}
  else {p = (b7 / b4) << 1;}

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  compp = p + ((x1 + x2 + 3791) >> 4);

  /* Assign compensated pressure value */
  *pressure = compp/100.0F;}

float pressureToAltitude(float seaLevel, float atmospheric){return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));}

//***************************************************************************
//ADS1115 ADC interface to ADXL377
//***************************************************************************
/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the ADS1015/ADS1115 ADC

    This is a library for the Adafruit MPL115A2 breakout
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#define ADS1115_ADDRESS                 (0x48)

static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();}

void beginADC(byte dataRate){

  #define ADS1115_REG_CONFIG_DR_32SPS    (0x0040)
  #define ADS1115_REG_CONFIG_DR_800SPS   (0x00E0)
  #define ADS1115_REG_CONFIG_DR_400SPS   (0x00C0)  
  uint16_t rateSPS;

  switch(dataRate){
    case 1: rateSPS = ADS1115_REG_CONFIG_DR_800SPS;
    break;

    case 2: rateSPS = ADS1115_REG_CONFIG_DR_400SPS;
    break;

    case 3: rateSPS = ADS1115_REG_CONFIG_DR_32SPS;
    break;}
  
  uint16_t configADS = 
   
  #define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  
  ADS1115_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
  #define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)
  ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
  #define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000) 
  ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
  #define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  
  ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
  //set the sample rate
  rateSPS   | 
  #define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  
  ADS1115_REG_CONFIG_MODE_CONTIN;   // Continuous mode

  //Set gain
  #define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200) 
  configADS |= ADS1115_REG_CONFIG_PGA_4_096V;

  //Set channel
  #define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  
  configADS |= ADS1115_REG_CONFIG_MUX_SINGLE_0;

  // Set 'start single-conversion' bit
  #define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  
  configADS |= ADS1115_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  #define ADS1115_REG_POINTER_CONFIG      (0x01)
  writeRegister(ADS1115_ADDRESS, ADS1115_REG_POINTER_CONFIG, configADS);}

 void getADC0(){
  #define ADS1115_REG_POINTER_CONVERT     (0x00)
  readS16(ADS1115_ADDRESS, ADS1115_REG_POINTER_CONVERT, &analogAccelX);}  


