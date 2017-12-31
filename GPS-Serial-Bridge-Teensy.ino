#include <TinyGPS++.h>

boolean gpsStatus[] = {false, false, false, false, false, false, false};
unsigned long start;

HardwareSerial gpsSerial(Serial1);
TinyGPSPlus GPS;

void setup()
{
  gpsSerial.begin(115200); 
  // START OUR SERIAL DEBUG PORT
  Serial.begin(115200);
  //
  //Settings Array contains the following settings: [0]NavMode, [1]DataRate1, [2]DataRate2, [3]PortRateByte1, [4]PortRateByte2, [5]PortRateByte3, 
  //[6]NMEA GLL Sentence, [7]NMEA GSA Sentence, [8]NMEA GSV Sentence, [9]NMEA RMC Sentence, [10]NMEA VTG Sentence
  //NavMode: 
  //Pedestrian Mode    = 0x03
  //Automotive Mode    = 0x04
  //Sea Mode           = 0x05
  //Airborne < 1G Mode = 0x06
  //Airborne < 2G Mode = 0x07
  //Airborne < 4G Mod3 = 0x08
  //
  //DataRate:
  //1Hz     = 0xE8 0x03
  //2Hz     = 0xF4 0x01
  //3.33Hz  = 0x2C 0x01
  //4Hz     = 0xFA 0x00
  //5Hz     = 0xC8 0x00
  //10Hz    = 0x64 0x00
  //
  //PortRate:
  //4800   = C0 12 00
  //9600   = 80 25 00
  //19200  = 00 4B 00  **SOFTWARESERIAL LIMIT FOR ARDUINO UNO R3!**
  //38400  = 00 96 00  **SOFTWARESERIAL LIMIT FOR ARDUINO MEGA 2560!**
  //57600  = 00 E1 00
  //115200 = 00 C2 01
  //230400 = 00 84 03
  //
  //NMEA Messages: 
  //OFF = 0x00
  //ON  = 0x01
  //                       NavMode   DataRate1  DataRate2  PortRate1  PortRate2    PortRate3    GLL   GSA   GSV   RMC   VTG
  byte settingsArray[] = {  0x08,      0xFA,      0x00,      0x00,     0xC2,        0x01,       0x01, 0x01, 0x01, 0x01, 0x01}; //
  //configureUblox(settingsArray); 
}

void loop()
{
  while(1) {
    if(gpsSerial.available())
    {
    // THIS IS THE MAIN LOOP JUST READS IN FROM THE GPS SERIAL AND ECHOS OUT TO THE ARDUINO SERIAL.
    Serial.write(gpsSerial.read()); 

    
    }
    
        } 
}   

