//-----------------------------------------------
//Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
//22 OCT 23: Initial Build
//19 NOV 23: Added more devices
//----------------------------------
//This file is for the use of external libraries to drive devices & sensors not supported in Device_Drivers.ino, SX127X_Driver.ino, or UBLOX_GNSS_Config.ino
//This system can support exeternal libraries for 5 types of devices: 
//accelerometer, gyroscope, high-G accelerometer, barometric pressure/temperature, GNSS reciever, and packet radio
//To use external libraries follow the process below:
/*Step 1: Use the EEPROMsettings.txt file to identify which sensor(s) use an external library
Step 2: Find the function below for the sensor (i.e. accelerometer, gyroscope, magnetometer, etc)
Step 3: Follow the commented process and write the commands from the external library
Step 4: Place the EEPROMsettings.txt file in the root folder with the Settings.txt file (it will be deleted after use)*/

//Include statements
//#include <mySensor.h>
#include <Adafruit_GPS.h>

//make necessary declarations
//Adafruit GPS setup
Adafruit_GPS ultGPS(HWSERIAL);

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//Accelerometer
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

bool beginExternalAccel(){

  //write the begin statement to start the sensor
  bool accelResponse = false;
  //accelResponse = mySensorBegin();
  if(accelResponse){Serial.println("Acclerometer OK!");}
  else{Serial.println("Acclerometer not found!");}

  //set the range of the accelerometer to the maximum setting, usually 16Gs
  //setMySensorRange(16G);

  //tell the system the lsb per G from the data sheet
  float Sensor_G_per_LSB;
  //Sensor_G_per_LSB = 0.000732;
  accel.gainX = accel.gainY = accel.gainZ = Sensor_G_per_LSB;

  //set the 1G reference value
  g = (int16_t)(1/accel.gainX);

  //set the output data rate to at least 1000Hz
  //setMySensorODR();

  //tell the system how frequently to sample the sensor (this should be the ODR)
  uint32_t ODR = 1000;
  accel.timeBtwnSamp = (uint32_t)(1000000/ODR);

  //set the value where the system changes over to the high-G accelerometer
  //accel.ADCmax = percent threshold * maximum ADC output
  //percent threshould should be no less than 90%
  //maximum ADC output should be the maximum value the ADC can output, ie 12-bit ADC = 2048, 16-bit ADC = 32768
  accel.ADCmax = (int)(0.98 * 32768);

  return accelResponse;}

void getExternalAccel(){

  int16_t accelX, accelY, accelZ;

  //read the data, this must be the raw data for the system to work properly
  //getAccelData();

  //set the sensor raw data equal to the raw sensor output
  accel.rawX = accelX;
  accel.rawY = accelY;
  accel.rawZ = accelZ;}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//Gyroscope
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

bool beginExternalGyro(){

  //write the begin statement to start the sensor
  bool gyroResponse = false;
  //gyroResponse = mySensorBegin();

  //set the range of the gyro to maximum setting, usually 2000dps
  //setMySensorGain(2000DPS);

  //tell the system what the gain value is from the data sheet
  float degrees_per_LSB;
  //degrees_per_LSB = 0.07;
  gyro.gainX = gyro.gainY = gyro.gainZ = degrees_per_LSB;

  //set the output data rate to at least 1000Hz
  //setMySensorODR();

  //tell the system how frequently to sample the sensor (this should be the ODR)
  uint32_t ODR = 1000;
  gyro.timeBtwnSamp = (uint32_t)(1000000/ODR);

  return gyroResponse;}

void getExternalGyro(){

  int16_t gyroX, gyroY, gyroZ;

  //read the data, this must be the raw data for the system to work properly
  //getGyroData();
  
  //set the raw output values
  gyro.rawX = gyroX;
  gyro.rawY = gyroY;
  gyro.rawZ = gyroZ;}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//Magetometer
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

bool beginExternalMag(){

  //write the begin statement to start the sensor
  bool magResponse = false;
  //magResponse = mySensorBegin();
  if(magResponse){Serial.println("Magnetometer OK!");}
  else{Serial.println("Magnetometer not found!");}

  //set the range of the magnetometer to 4Gauss or 8 Gauss if the magnetic switch is enabled
  float gauss_per_LSB;
  if(settings.magSwitchEnable){
    //setGain(8GAUSS);
    //gauss_per_LSB = 0.008;
  }
  else{
    //setGain(4GAUSS);
    //gauss_per_LSB = 0.004;
  }

  //tell the system what the gain value is from the data sheet
  mag.gainX = mag.gainY = mag.gainZ = gauss_per_LSB;

  //set the output data rate to at least 30Hz
  //setMySensorODR(30Hz);

  //tell the system how frequently to sample the sensor (this should be the ODR)
  uint32_t ODR = 30;
  mag.timeBtwnSamp = (uint32_t)(1000000/ODR);

  return magResponse;}

 void getExternalMag(){

  int16_t magX, magY, magZ;

  //read the data, this must be the raw data for the system to work properly
  //getAccelData();
  
  //set the raw output values
  mag.rawX = magX;
  mag.rawY = magY;
  mag.rawZ = magZ;}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//High-G Accelerometer
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
bool beginExternalHighG(){

  //write the begin statement to start the sensor
  bool highG_Response = false;
  //highG_Response = mySensorBegin();
  if(highG_Response){Serial.println("High-G Acclerometer OK!");}
  else{Serial.println("High-G Acclerometer not found!");}

  //set the range of the accelerometer, usually to around 100Gs
  //setRange(100G);

  //tell the system the G's per LSB from the data sheet
  float Sensor_G_per_LSB;
  //Sensor_G_per_LSB = 0.049;
  highG.gainX = highG.gainY = highG.gainZ = Sensor_G_per_LSB;

  //determine the 1G reference value
  high1G = (int16_t)(1/highG.gainX);

  //set the size of the moving average filter
  sizeHighGfilter = 15;

  //set the output data rate to at least 1000Hz
  //setODR(1000HZ);

  //tell the system how frequently to sample the sensor (this should be the ODR)
   uint32_t ODR = 30;
  highG.timeBtwnSamp = (uint32_t)(1000000/ODR);

  return highG_Response;}

void getExternalHighG(){

  int16_t highG_X, highG_Y, highG_Z;

  //read the data, this must be the raw data for the system to work properly
  //getAccelData();
  
  //set the raw output values
  highG.rawX = highG_X;
  highG.rawY = highG_Y;
  highG.rawZ = highG_Z;}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//Barometric Pressure Sensor
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

bool beginExternalBaro(){

  //write the begin statement to start the sensor
  bool baroResponse = false;
  //baroResponse = mySensorBegin();
  if(baroResponse){Serial.println("Barometer OK!");}
  else{Serial.println("Barometer not found!");}

  //set the barometer precision to the maximum precision capable of at least 30Hz ODR
  //setRange(UltraHighPrecision);

  //set the output data rate to at least 30Hz
  //setODR(30HZ);

  //tell the system how frequently to sample the sensor (this should be the ODR)
  uint32_t ODR = 30;
  baro.timeBtwnSamp = (uint32_t)(1000000/ODR);

  return baroResponse;
}

void getExternalBaro(){

  //get a pressure reading, output must be in hPa
  //baro.pressure = getBaroData();

  //convert pressure to altitude
  //baro.rawAlt = convert(baro.pressure);

  //get a temperature reading, output must be in Celcius with 0.1C precision
  //baro.temperature = getTempData();

}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//GNSS Device
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
void externalGNSS_PSM(){

  //Set lower update rate for Adafruit Ultimate GPS
  ultGPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);}

void ExternalGNSSconfig(){

    //setup for the Adafruit Ultimate GPS

    //restrict the output sentences
    ultGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    //increase the update rate
    ultGPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

    //increase the baud rate
    ultGPS.sendCommand(PMTK_SET_BAUD_57600);
      
    //restart Serial output at a higher rate
    Serial.begin(57600);}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//Packet radio
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
bool beginExternalRadio(){

  bool response = false;
  //this is where you write the commands from the separate library to start the radio

  //reset the system if needed

  //set the important registers for the wave form

  //set the important registers for bandwidth

  //set the registers for baud rate

  //set the registers for interrupts

  //set the packet timing, default for inflight data is 5 packets per second, 4 samples per packet
  //be very careful with these settings. If the packets are very large and the radio cannot transmit the data
  //before the next packet finishes, then the telemetry may not finish or the system may crash
  pktInterval.preLiftoff = 1000000UL;//default of 1 packet per second before liftoff
  pktInterval.inflight = 200000UL;//default of 5 packets per second while in flight
  pktInterval.postFlight = 10000000UL;//default of 10 seconds between packets post-flight
  pktInterval.samplesPerPkt = 4;//default of 4 samples per inflight packet

  return response;}

void setExternalRadioFreq(float freq){

  //external library commands to set the frequency (MHz)

}

void setExternalRadioPwr(uint8_t pwr){

  //external library commands to set the power level (dBm)

}

bool sendExternalRadioDataPkt(uint8_t* data, uint8_t len){

  bool response = false;

  //external library commands to send the packet
  //len is how many bytes to send
  //*data is a pointer to the array that holds the data
  //if you use the provided routines, the data is in dataPacket[77]

  return response;}

void clearFlagsExternalRadio(){

  //if there are interrupts or flags that need to be cleared, put the code in this section
  //otherwise just leave it blank
}

void externalRadioSleep(){
  //write the commands to send the radio to sleep mode

}