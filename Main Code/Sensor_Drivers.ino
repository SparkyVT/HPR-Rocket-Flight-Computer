//Sketch adapted for the Adafruit 10DoF IMU board
//By Bryan Sparkman, TRA #12111, NAR #85720, L3
//Sensor Package 1: LSM303DLHC, L3GD20, BMP180, ADS1115, ADXL377
//Sensor Package 2: LSM9DS1, BMP280, H3LIS331DL
//Sensor Package 3: LSM9DS1, MPL3115A2, H3LIS331DL
//-----------Change Log------------
//26 Nov 17: Version 1 created
//10 Nov 18: Version 2 created to support new sensor package
//30 Apr 19: Version 3 created to support MPL3115A2 after EMI problems with BMP280 & BMP388

//***************************************************************************
//LSM9DS1 Accelerometer, Gyroscope, & Magnetometer
//***************************************************************************

bool beginLSM9DS1(){

  //Addresses for the registers
  #define LSM9DS1_ADDRESS_ACCELGYRO          (0x6B)
  #define LSM9DS1_ADDRESS_MAG                (0x1E)
  #define LSM9DS1_XG_ID                      (0b01101000)
  #define LSM9DS1_MAG_ID                     (0b00111101)
  #define LSM9DS1_REGISTER_CTRL_REG1_G         (0x10)
  #define LSM9DS1_REGISTER_CTRL_REG5_XL        (0x1F)
  #define LSM9DS1_REGISTER_CTRL_REG6_XL        (0x20)
  #define LSM9DS1_REGISTER_CTRL_REG1_M         (0x20)
  #define LSM9DS1_REGISTER_CTRL_REG3_M         (0x22)
  #define LSM9DS1_REGISTER_CTRL_REG4_M         (0x23)
  #define LSM9DS1_REGISTER_CTRL_REG3_G         (0x12)

  //----------------------
  //GET WHO AM I REGISTERS
  //----------------------
  byte id = read8(0x0F,LSM9DS1_ADDRESS_ACCELGYRO);
  if (id != 0b01101000){return false;}
  
  //----------------------
  //CONFIGURE ACCELEROMETER
  //----------------------
  //Set 16G Range, 952 Hz ODR,
  write8(LSM9DS1_REGISTER_CTRL_REG6_XL, LSM9DS1_ADDRESS_ACCELGYRO,  0b11001000);
  
  //----------------------
  //CONFIGURE GYROSCOPE
  //----------------------
  //Set 2000dps Range, 952 Hz ODR
  write8(LSM9DS1_REGISTER_CTRL_REG1_G, LSM9DS1_ADDRESS_ACCELGYRO, 0b11011000);

  //enable high-pass filtern at 30Hz cutoff frequency
  write8(LSM9DS1_REGISTER_CTRL_REG3_G, LSM9DS1_ADDRESS_ACCELGYRO, 0b01000001);

  //----------------------
  //CONFIGURE MAGNETOMETER
  //----------------------
  //Mag Temp Compensation, UltraHigh Perf, 10Hz
  write8(LSM9DS1_REGISTER_CTRL_REG1_M, LSM9DS1_ADDRESS_MAG, 0b11110000);
  //Mag Continuous Conversion
  write8(LSM9DS1_REGISTER_CTRL_REG3_M, LSM9DS1_ADDRESS_MAG, 0b000000000);
  //Mag Reverse Magnetometer MSB / LSB Order, Z-Axis high-perf mode
  write8(LSM9DS1_REGISTER_CTRL_REG4_M, LSM9DS1_ADDRESS_MAG, 0b000011100);

  return true;
  }//end begin

void getAccelGyro(){

  //this uses the LSM9DS1 burst read to rapidly sample the sensors
  #define LSM9DS1_REGISTER_OUT_X_L_XL (0x28)
  #define LSM9DS1_REGISTER_OUT_X_L_G  (0x18)
  //readSensor(LSM9DS1_ADDRESS_ACCELGYRO, 0x80 | LSM9DS1_REGISTER_OUT_X_L_XL, (byte)1);
  
  Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
  //Wire.write(0x80 | LSM9DS1_REGISTER_OUT_X_L_G);
  Wire.write(LSM9DS1_REGISTER_OUT_X_L_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS1_ADDRESS_ACCELGYRO, (byte)12);

  //wait for the gyro data to arrive
  while(Wire.available() < 6){};
  
  //Capture current timestamp
  timeGyroClockPrev = timeGyroClock;
  timeGyroClock = micros();

  //wait for the accelerometer data to arrive
  while(Wire.available() < 12){};

  //Capture current timestamp
  timeClockPrev = timeClock;
  timeClock = micros();

  //read the data
  uint8_t gxLow  = Wire.read();
  uint8_t gxHigh = Wire.read();
  uint8_t gyLow  = Wire.read();
  uint8_t gyHigh = Wire.read();
  uint8_t gzLow  = Wire.read();
  uint8_t gzHigh = Wire.read();
  uint8_t axLow  = Wire.read();
  uint8_t axHigh = Wire.read();
  uint8_t ayLow  = Wire.read();
  uint8_t ayHigh = Wire.read();
  uint8_t azLow  = Wire.read();
  uint8_t azHigh = Wire.read();
  
  accelX = (int16_t)(axLow | (axHigh << 8));
  accelY = (int16_t)(ayLow | (ayHigh << 8));
  accelZ = (int16_t)(azLow | (azHigh << 8));
  gyroX  = (int16_t)(gxLow | (gxHigh << 8));
  gyroY  = (int16_t)(gyLow | (gyHigh << 8));
  gyroZ  = (int16_t)(gzLow | (gzHigh << 8));
  }

void getAccel(){
  #define LSM9DS1_REGISTER_OUT_X_L_XL (0x28)
  readSensor(LSM9DS1_ADDRESS_ACCELGYRO, 0x80 | LSM9DS1_REGISTER_OUT_X_L_XL, (byte)1);}

void getGyro(){
  #define LSM9DS1_REGISTER_OUT_X_L_G  (0x18)
  readSensor(LSM9DS1_ADDRESS_ACCELGYRO, 0x80 | LSM9DS1_REGISTER_OUT_X_L_G, (byte)3);}

void getMag(){
  #define LSM9DS1_REGISTER_OUT_X_L_M  (0x28)
  readSensor(LSM9DS1_ADDRESS_MAG, 0x80 | LSM9DS1_REGISTER_OUT_X_L_M, (byte)2);}

void readSensor(byte address, byte reg, byte sensor){
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(I2C_NOSTOP);
  Wire.requestFrom(address, (byte)6);
  while(Wire.available() < 6){};

  uint8_t xLow  = Wire.read();
  uint8_t xHigh = Wire.read();
  uint8_t yLow  = Wire.read();
  uint8_t yHigh = Wire.read();
  uint8_t zLow  = Wire.read();
  uint8_t zHigh = Wire.read();

  switch(sensor){

    case 1://LSM9DS1 accelerometer
      accelX = (int16_t)(xLow | (xHigh << 8));
      accelY = (int16_t)(yLow | (yHigh << 8));
      accelZ = (int16_t)(zLow | (zHigh << 8));
      break;

    case 2://LSM9DS1 magnetometer (high & low reversed)
      magX = (int16_t)(xHigh | ((int16_t)xLow << 8));
      magY = (int16_t)(yHigh | ((int16_t)yLow << 8));
      magZ = (int16_t)(zHigh | ((int16_t)zLow << 8));
      break;

    case 3://LSM9DS1 gyro
      gyroX = (int16_t)(xLow | (xHigh << 8));
      gyroY = (int16_t)(yLow | (yHigh << 8));
      gyroZ = (int16_t)(zLow | (zHigh << 8));
      break;

    case 4://H3LIS331 high-G accelerometer (potential future use)
      highGx = -1*(int16_t)(xLow | (xHigh << 8))>> 4;
      highGy = (int16_t)(yLow | (yHigh << 8))>> 4;
      highGz = (int16_t)(zLow | (zHigh << 8))>> 4;
      break; 
  }}//end void
  
//***************************************************************************
//H3LIS331DL High-G Accelerometer
//***************************************************************************

bool beginH3LIS331DL(byte DR){
  
  #define H3LIS331_ADDRESS            (0x19)
  #define H3LIS331_REGISTER_CTRL_REG1 (0x20)
  #define H3LIS331_REGISTER_CTRL_REG2 (0x21)
  #define H3LIS331_REGISTER_CTRL_REG4 (0x23)

  byte id = read8(0x0F, H3LIS331_ADDRESS);
  if(id != 0b00110010){return false;}
  
  if(DR == 1){
    //Normal Mode (001), 1000 Hz data rate (11), Enable axes (111)
    write8(H3LIS331_REGISTER_CTRL_REG1, H3LIS331_ADDRESS, 0b00111111);}

  if(DR == 2){
    //Normal Mode (001), 100 Hz data rate (01), Enable axes (111)
    write8(H3LIS331_REGISTER_CTRL_REG1, H3LIS331_ADDRESS, 0b00101111);

    //Normal Mode (0), High-Pass filter mode (01), Filter Data Select (1), 
    //HP Interrupt2 (0), HP Interrupt1 (0), HPF Coeff(00)
    //write8(H3LIS331_REGISTER_CTRL_REG2, H3LIS331_ADDRESS, 0b00110000);
  
    //Set 100G scale
    write8(H3LIS331_REGISTER_CTRL_REG4, H3LIS331_ADDRESS, 0b00000000);}

  if(DR == 3){
    //Normal Mode (001), 400 Hz data rate (10), Enable axes (111)
    write8(H3LIS331_REGISTER_CTRL_REG1, H3LIS331_ADDRESS, 0b00110111);}

  return true;
  }

void getHighG(){

    #define H3LIS331_REGISTER_OUT_X_L   0b10101000 //(0x28)
    //readSensor(H3LIS331_ADDRESS, H3LIS331_REGISTER_OUT_X_L, (byte)4);

    Wire.beginTransmission(H3LIS331_ADDRESS);
    Wire.write(H3LIS331_REGISTER_OUT_X_L);
    Wire.endTransmission(I2C_NOSTOP);
    Wire.requestFrom(H3LIS331_ADDRESS, (byte)6);
    while(Wire.available() < 6){};

    uint8_t xLow  = Wire.read();
    uint8_t xHigh = Wire.read();
    uint8_t yLow  = Wire.read();
    uint8_t yHigh = Wire.read();
    uint8_t zLow  = Wire.read();
    uint8_t zHigh = Wire.read();

    highGx = (int16_t)(xLow | (xHigh << 8))>> 4;
    highGy = (int16_t)(yLow | (yHigh << 8))>> 4;
    highGz = (int16_t)(zLow | (zHigh << 8))>> 4;
}

//***************************************************************************
//MPL3115A2 Barometric Pressure Sensor
//***************************************************************************

boolean beginMPL3115A2(){
  #define MPL3115A2_ADDRESS  0x60
  #define MPL3115A2_I2C_READ 0xC1
  #define MPL3115A2_I2C_WRITE 0xC0
  #define MPL3115A2_WHOAMI   0x0C

  boolean bmpOK = true;
  //establish contact
  uint8_t reply = read8(MPL3115A2_WHOAMI, MPL3115A2_ADDRESS);
  if(reply != 0xC4){bmpOK = false;}
  //Serial.println(reply, HEX);
  
  //SETUP Control Register
  //barometer mode: 0
  //RAW mode: 0
  //Oversampling: 34ms - 011
  //Reset: 0
  //One Shot Mode (OST): 1
  //Standby: 0
  write8(0x26, MPL3115A2_ADDRESS, 0b00011010);

  return bmpOK; 
}

void readMPLbaro(){

  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(MPL3115A2_ADDRESS, (byte)5);
  while(Wire.available() < 4){};

  uint32_t adc_P;
  adc_P = Wire.read();
  adc_P <<= 8;
  adc_P |= Wire.read();
  adc_P <<= 8;
  adc_P |= Wire.read();
  adc_P >>= 4;
  
  int16_t adc_T;
  adc_T = Wire.read();
  adc_T <<= 8;
  adc_T |= Wire.read();
  adc_T >>= 4;
  
  pressure = (float)(adc_P) * 0.0025;
  temperature = adc_T;
  if (adc_T & 0x800){adc_T |= 0xF000;}
  temperature = (float)(adc_T) * 0.0625;
  Alt = 44330.77 * (1.0 - pow(pressure / seaLevelPressure, 0.1902632));

  //initiate next reading
  write8(0x26, MPL3115A2_ADDRESS, 0b00011010);
}

float pressureToAltitude(float seaLevel, float atmospheric){return 44330.77 * (1.0 - pow(atmospheric / seaLevel, 0.1902632));}

//***************************************************************************
//BMP388 Barometric Pressure Sensor
//***************************************************************************
#define BMP388_ADDRESS                (0x77)

enum{
  REGISTER_NVM_PAR_P11 = 0x45,
  REGISTER_NVM_PAR_P10 = 0x44,
  REGISTER_NVM_PAR_P09U = 0x43,
  REGISTER_NVM_PAR_P09L = 0x42,
  REGISTER_NVM_PAR_P08 = 0x41,
  REGISTER_NVM_PAR_P07 = 0x40,
  REGISTER_NVM_PAR_P06U = 0x3F,
  REGISTER_NVM_PAR_P06L = 0x3E,
  REGISTER_NVM_PAR_P05U = 0x3D,
  REGISTER_NVM_PAR_P05L = 0x3C,
  REGISTER_NVM_PAR_P04 = 0x3B,
  REGISTER_NVM_PAR_P03 = 0x3A,
  REGISTER_NVM_PAR_P02U = 0x39,
  REGISTER_NVM_PAR_P02L = 0x38,
  REGISTER_NVM_PAR_P01U = 0x37,
  REGISTER_NVM_PAR_P01L = 0x36,

  REGISTER_NVM_PAR_T3 = 0x35,
  REGISTER_NVM_PAR_T2U = 0x34,
  REGISTER_NVM_PAR_T2L = 0x33,
  REGISTER_NVM_PAR_T1U = 0x32,
  REGISTER_NVM_PAR_T1L = 0x31,
  
};

typedef struct
    {
      int8_t    NVM_PAR_P11;
      int8_t    NVM_PAR_P10;
      int16_t   NVM_PAR_P09;
      int8_t    NVM_PAR_P08;
      int8_t    NVM_PAR_P07;
      uint16_t  NVM_PAR_P06;
      uint16_t  NVM_PAR_P05;
      int8_t    NVM_PAR_P04;
      int8_t    NVM_PAR_P03;
      int16_t   NVM_PAR_P02;
      int16_t   NVM_PAR_P01;

      int8_t    NVM_PAR_T3;
      uint16_t  NVM_PAR_T2;
      uint16_t  NVM_PAR_T1;

      double    PAR_P11 ;
      double    PAR_P10 ;
      double    PAR_P09 ;
      double    PAR_P08 ;
      double    PAR_P07 ;
      double    PAR_P06 ;
      double    PAR_P05 ;
      double    PAR_P04 ;
      double    PAR_P03 ;
      double    PAR_P02 ;
      double    PAR_P01 ;

      double    PAR_T3 ;
      double    PAR_T2 ;
      double    PAR_T1 ;

      float     t_lin ;
      
    } bmp388_calib_data;

bmp388_calib_data bmp388cal;

boolean beginBMP388(){

  #define BMP388_REGISTER_CHIPID 0x00
  
  if (read8(BMP388_REGISTER_CHIPID, BMP388_ADDRESS) != 0x50)
    return false;

  #define BMP388_REGISTER_PWR_CTRL  0x1B
  #define BMP388_REGISTER_CONFIG    0x1F
  #define BMP388_REGISTER_OSR       0x1C
  
  delay(100);
  
  bmp388readCoefficients();
  
  //put the BMP388 into sleep mode
  write8(BMP388_REGISTER_PWR_CTRL, BMP388_ADDRESS, 0x00);
  
  //Set the config register 4 IIR coeff: 010 = 4
  write8(BMP388_REGISTER_CONFIG,  BMP388_ADDRESS, 0b00000100);
  
  //Set the control register to Ultra high resolution
  //osr_p: 100 = 16x (2:0)
  //osr_t: 001 = 2x  (5:3)
  write8(BMP388_REGISTER_OSR, BMP388_ADDRESS, 0b00001100);
  
  return true;
}

void bmp388readCoefficients(){

    double denominator;
  
    bmp388cal.NVM_PAR_P11 = readS8(REGISTER_NVM_PAR_P11, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P10 = readS8(REGISTER_NVM_PAR_P10, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P09 = readS16_LE(REGISTER_NVM_PAR_P09L, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P08 = readS8(REGISTER_NVM_PAR_P08, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P07 = readS8(REGISTER_NVM_PAR_P07, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P06 = read16_LE(REGISTER_NVM_PAR_P06L, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P05 = read16_LE(REGISTER_NVM_PAR_P05L, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P04 = readS8(REGISTER_NVM_PAR_P04, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P03 = readS8(REGISTER_NVM_PAR_P03, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P02 = readS16_LE(REGISTER_NVM_PAR_P02L, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_P01 = readS16_LE(REGISTER_NVM_PAR_P01L, BMP388_ADDRESS);

    bmp388cal.NVM_PAR_T3 = readS8(REGISTER_NVM_PAR_T3, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_T2 = read16_LE(REGISTER_NVM_PAR_T2L, BMP388_ADDRESS);
    bmp388cal.NVM_PAR_T1 = read16_LE(REGISTER_NVM_PAR_T1L, BMP388_ADDRESS);

    denominator = 0.00390625f;
    bmp388cal.PAR_T1 = (double)bmp388cal.NVM_PAR_T1/denominator;

    denominator = 1073741824.0f;
    bmp388cal.PAR_T2 = (double)bmp388cal.NVM_PAR_T2/denominator;

    denominator = 281474976710656.0f;
    bmp388cal.PAR_T3 = (double)bmp388cal.NVM_PAR_T3/denominator;

    denominator = 36893488147419103232.0f;
    bmp388cal.PAR_P11 = (double)bmp388cal.NVM_PAR_P11/denominator;

    denominator = 281474976710656.0f;
    bmp388cal.PAR_P10 = (double)bmp388cal.NVM_PAR_P10/denominator;

    bmp388cal.PAR_P09 = (double)bmp388cal.NVM_PAR_P09/denominator;

    denominator = 32768.0f;
    bmp388cal.PAR_P08 = (double)bmp388cal.NVM_PAR_P08/denominator;
    
    denominator = 256.0f;
    bmp388cal.PAR_P07 = (double)bmp388cal.NVM_PAR_P07/denominator;

    denominator = 64.0f;
    bmp388cal.PAR_P06 = (double)bmp388cal.NVM_PAR_P06/denominator;

    denominator = 0.125f;
    bmp388cal.PAR_P05 = (double)bmp388cal.NVM_PAR_P05/denominator;

    denominator = 137438953472.0f;
    bmp388cal.PAR_P04 = (double)bmp388cal.NVM_PAR_P04/denominator;

    denominator = 4294967296.0f;
    bmp388cal.PAR_P03 = (double)bmp388cal.NVM_PAR_P03/denominator;

    denominator = 536870912.0f;
    bmp388cal.PAR_P02 = ((double)bmp388cal.NVM_PAR_P02 - 16384.0f)/denominator;

    denominator = 1048576.0f;
    bmp388cal.PAR_P01 = ((double)bmp388cal.NVM_PAR_P01 - 16384.0f)/denominator;
}

void bmp388GetReading(){

  //---------------------------------------------------------------
  //burst read from the registers
  //---------------------------------------------------------------

  #define BMP388_REGISTER_PRESSUREDATA 0x04
  
  Wire.beginTransmission(BMP388_ADDRESS);
  Wire.write(BMP388_REGISTER_PRESSUREDATA);
  Wire.endTransmission(I2C_NOSTOP);
  Wire.requestFrom(BMP388_ADDRESS, (byte)6);
  while(Wire.available() < 6){};

  uint32_t uncomp_press;
  /*uncomp_press = Wire.read();
  uncomp_press <<= 8;
  uncomp_press |= Wire.read();
  uncomp_press <<= 8;
  uncomp_press |= Wire.read();
  uncomp_press >>= 4;*/
  uncomp_press = Wire.read();
  uncomp_press += (Wire.read() << 8);
  uncomp_press += (Wire.read() << 16);
  
  uint32_t uncomp_temp;
  /*uncomp_temp = Wire.read();
  uncomp_temp <<= 8;
  uncomp_temp |= Wire.read();
  uncomp_temp <<= 8;
  uncomp_temp |= Wire.read();
  uncomp_temp >>= 4;*/
  uncomp_temp = Wire.read();
  uncomp_temp += (Wire.read() << 8);
  uncomp_temp += (Wire.read() << 16);

  //put the BMP388 into forced mode
  //press_en: 1 (0)
  //temp_en:  1 (1)
  //mode:     01 (5:4)
  write8(BMP388_REGISTER_PWR_CTRL, BMP388_ADDRESS, 0b00010011);
  
  //---------------------------------------------------------------
  //compensate and compute temperature
  //---------------------------------------------------------------
  
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;

  partial_data1 = (float)(uncomp_temp - bmp388cal.PAR_T1);

  partial_data2 = (float)(partial_data1 * bmp388cal.PAR_T2);

  bmp388cal.t_lin = partial_data2 + (partial_data1 * partial_data1) * bmp388cal.PAR_T3;

  temperature = bmp388cal.t_lin;
  //temperature *= .01;
  
  //---------------------------------------------------------------
  //compensate and compute pressure
  //---------------------------------------------------------------

  partial_data1 = bmp388cal.PAR_P06 * bmp388cal.t_lin;
  partial_data2 = bmp388cal.PAR_P07 * (bmp388cal.t_lin * bmp388cal.t_lin);
  partial_data3 = bmp388cal.PAR_P08 * (bmp388cal.t_lin * bmp388cal.t_lin * bmp388cal.t_lin);
  partial_out1 = bmp388cal.PAR_P05 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = bmp388cal.PAR_P02 * bmp388cal.t_lin;
  partial_data2 = bmp388cal.PAR_P03 * (bmp388cal.t_lin * bmp388cal.t_lin);
  partial_data3 = bmp388cal.PAR_P04 * (bmp388cal.t_lin * bmp388cal.t_lin * bmp388cal.t_lin);
  partial_out2 = (float)uncomp_press * (bmp388cal.PAR_P01 + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = (float)uncomp_press * (float)uncomp_press;
  partial_data2 = bmp388cal.PAR_P09 + bmp388cal.PAR_P10 * bmp388cal.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * bmp388cal.PAR_P11;
  pressure  = partial_out1 + partial_out2 + partial_data4;

  //--------------------------------------------
  //Calculate Altitude
  //--------------------------------------------
  
  pressure *= 0.01;

  Alt = 44330 * (1.0 - pow(pressure / (seaLevelPressure), 0.1903));
}

//***************************************************************************
//BMP280 Barometric Pressure Sensor
//***************************************************************************
/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#define BMP280_ADDRESS                (0x77)

 enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;

bmp280_calib_data _bmp280_calib;

boolean beginBMP280(){
  
  if (read8(BMP280_REGISTER_CHIPID, BMP280_ADDRESS) != 0x58)
    return false;

  delay(100);
  readCoefficients();
  //put the BMP280 into sleep mode
  write8(BMP280_REGISTER_CONTROL, BMP280_ADDRESS, 0x00);
  //Set the config register to 0.5ms standby, 16 IIR coeff, 
  write8(BMP280_REGISTER_CONFIG,  BMP280_ADDRESS, 0b00010100);//16 = 0b00011100 = 0x1C, 8 = 0b00001100 = 0x0C
  //Set the control register to Ultra high resolution and sleep mode: osrr_t,osrs_p,mode
  write8(BMP280_REGISTER_CONTROL, BMP280_ADDRESS, 0b01010111);//0b00111111 = 0x3F, 0b01010100 = 0x54, 0b01011111 = 0x5F
  
  return true;
}

void readCoefficients(){
  
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1, BMP280_ADDRESS);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2, BMP280_ADDRESS);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3, BMP280_ADDRESS);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1, BMP280_ADDRESS);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2, BMP280_ADDRESS);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3, BMP280_ADDRESS);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4, BMP280_ADDRESS);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5, BMP280_ADDRESS);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6, BMP280_ADDRESS);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7, BMP280_ADDRESS);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8, BMP280_ADDRESS);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9, BMP280_ADDRESS);
}

void bmpGetReading(){

  //---------------------------------------------------------------
  //burst read from the registers
  //---------------------------------------------------------------

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(BMP280_REGISTER_PRESSUREDATA);
  Wire.endTransmission(I2C_NOSTOP);
  Wire.requestFrom(BMP280_ADDRESS, (byte)6);
  while(Wire.available() < 6){};

  int32_t adc_P;
  adc_P = Wire.read();
  adc_P <<= 8;
  adc_P |= Wire.read();
  adc_P <<= 8;
  adc_P |= Wire.read();
  adc_P >>= 4;
  
  int32_t adc_T;
  adc_T = Wire.read();
  adc_T <<= 8;
  adc_T |= Wire.read();
  adc_T <<= 8;
  adc_T |= Wire.read();
  adc_T >>= 4;

  //put the BMP280 into forced mode
  write8(BMP280_REGISTER_CONTROL, BMP280_ADDRESS, 0b10101010); //0b01010110 = 0x56
  
  //---------------------------------------------------------------
  //compensate and compute temperature
  //---------------------------------------------------------------
  
  int32_t var1, var2, t_fine;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
     ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
       ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
     ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  temperature  = (t_fine * 5 + 128) >> 8;
  temperature *= .01;
  //---------------------------------------------------------------
  //Read pressure and convert
  //---------------------------------------------------------------

  int64_t var3, var4, p;

  var3 = ((int64_t)t_fine) - 128000;
  var4 = var3 * var3 * (int64_t)_bmp280_calib.dig_P6;
  var4 = var4 + ((var3*(int64_t)_bmp280_calib.dig_P5)<<17);
  var4 = var4 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var3 = ((var3 * var3 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var3 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var3 = (((((int64_t)1)<<47)+var3))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var3 == 0) {
    pressure = 0;  // avoid exception caused by division by zero
  }
  else{
    p = 1048576 - adc_P;
    p = (((p<<31) - var4)*3125) / var3;
    var3 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var4 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var3 + var4) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
    pressure = (float)p/256;}

  //--------------------------------------------
  //Calculate Altitude
  //--------------------------------------------
  
  pressure *= 0.01;

  Alt = 44330 * (1.0 - pow(pressure / (seaLevelPressure), 0.1903));
}

uint16_t read16(byte reg, byte _i2caddr){
  
  uint16_t value;

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(I2C_NOSTOP);
  Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
  value = (Wire.read() << 8) | Wire.read();

  return value;}

uint16_t read16_LE(byte reg, byte _i2caddr) {
  uint16_t temp = read16(reg, _i2caddr);
  return (temp >> 8) | (temp << 8);}

int16_t readS16_LE(byte reg, byte _i2caddr){
  return (int16_t)read16_LE(reg, _i2caddr);}

int16_t readS16(byte reg, byte _i2caddr){
  return (int16_t)read16(reg, _i2caddr);}

int8_t readS8(byte reg, byte _i2caddr){

  int8_t value;

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(I2C_NOSTOP);
  Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
  value = Wire.read();

  return value;
}

uint8_t read8(byte reg, byte _i2caddr){
  
  uint8_t value;

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(I2C_NOSTOP);
  Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
  value = Wire.read();

  return value;}

void write8(byte reg, byte _i2caddr, byte value){

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission(I2C_STOP);}
    
