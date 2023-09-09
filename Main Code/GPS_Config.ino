//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//restoreGPSdefaults(): overrides command settings and restores factory settings (not currently used)
//configGPS(): sends UBX commands to update navigation, data rate, GNSS constellations, reduce NMEA sentences, interference thresholds, and baud rate
//GPSpowerSaveMode(): after touchdown reduce the power consumption and update rate
//sendUBX(): helper function to send serial data
//getUBX_ACK(): function to receive the UBX acknowledgement for each command
//printHex(): helper function to turn binary to hex
//----------------------------
//UBLOX configuration modified from https://playground.arduino.cc/UBlox/GPS

#include <Adafruit_GPS.h>

//Adafruit GPS setup
Adafruit_GPS ultGPS(HWSERIAL);

void restoreGPSdefaults(bool serialOutput){

  byte gpsSetSuccess = 0;
  if(serialOutput){Serial.println("Configuring u-Blox GPS initial state...");}
  
   //Generate the configuration string for Factory Default Settings
  byte setDefaults[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE};
  
   //Restore Factory Defaults
  while(gpsSetSuccess < 3) {
    if(serialOutput){Serial.print("Restoring Factory Defaults... ");}
    sendUBX(&setDefaults[0], sizeof(setDefaults));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setDefaults[2]);}
  if (gpsSetSuccess == 3 && serialOutput){Serial.println("Restore factory defaults failed!");}}

void configGPS(bool serialOutput, byte gpsVersion, bool VTGoption) {
  
  byte gpsSetSuccess = 0;

  if(serialOutput){Serial.println("Configuring u-Blox GPS ...");}

  //Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F};

  //Generate the configuration string for Data Rate
  //4Hz nominal rate
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};
  
  //5Hz Max data rate for NEO-M8N
  if(gpsVersion == 1){setDataRate[6] = 0xC8; setDataRate[12] = 0xDE; setDataRate[13] = 0x6A;}
  
  //10Hz Max data rate for NEO-M8Q, MAX-M8Q/W, SAM-M8Q
  if(gpsVersion == 2){setDataRate[6] = 0x64; setDataRate[12] = 0x7A; setDataRate[13] = 0x12;}
  
  //25Hz Max data rate for NEO-M9N
  if(gpsVersion == 3){setDataRate[6] = 0x28; setDataRate[12] = 0x3E; setDataRate[13] = 0xAA;}

  //Faster Baud Rate for the higher update rates
  byte setBaudRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70};
  
  //Generate the configuration string for NMEA messages
  byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
  byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
  byte set4_1[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57};

  //Generate the configuration string for interference resistance settings
  byte setJam[] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45};

  //set the constellations used
  //For M8, just add Galileo, from https://portal.u-blox.com/s/question/0D52p00008HKEEYCA5/ublox-gps-galileo-enabling-for-ubx-m8
  byte setSat[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56};

  //UBLOX Devices are numbered 1 through 3, Adafruit is 4
  if(gpsVersion < 4){
    
    //Turn Off NMEA GSA Messages
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Deactivating NMEA GSA Messages... ");}
      sendUBX(setGSA, sizeof(setGSA));
      gpsSetSuccess += getUBX_ACK(&setGSA[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("NMEA GSA Message Deactivation Failed!");}
    gpsSetSuccess = 0;
    
    //Set Navigation Mode
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Setting Navigation Mode... ");}
      sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
      gpsSetSuccess += getUBX_ACK(&setNav[2]);}
    if (gpsSetSuccess == 3 && serialOutput) {Serial.println("Navigation mode configuration failed!");}
    gpsSetSuccess = 0;
  
    //Turn Off NMEA GSV Messages
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Deactivating NMEA GSV Messages... ");}
      sendUBX(setGSV, sizeof(setGSV));
      gpsSetSuccess += getUBX_ACK(&setGSV[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("NMEA GSV Message Deactivation Failed!");}
    gpsSetSuccess = 0;
    
    //Set Data Update Rate
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Setting Data Update Rate... ");}
      sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
      gpsSetSuccess += getUBX_ACK(&setDataRate[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Data update mode configuration failed!");}
    gpsSetSuccess = 0;
  
    //Turn on NMEA4.1 Messages for less than M9 versions
    while(gpsVersion < 3 && gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Turning on NMEA 4.1 Messages... ");}
      sendUBX(set4_1, sizeof(set4_1));
      gpsSetSuccess += getUBX_ACK(&set4_1[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("NMEA 4.1 Message Activation Failed!");}
    gpsSetSuccess = 0;
  
    //Turn on Galileo for M8 versions since it is not enabled by default
    while(gpsVersion < 3 && gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Setting Satellite Constellations... ");}
      sendUBX(setSat, sizeof(setSat));
      gpsSetSuccess += getUBX_ACK(&setSat[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Satellite Settings Failed!");}
    gpsSetSuccess = 0;
  
    //Turn Off NMEA VTG Messages
    while(!VTGoption && gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Deactivating NMEA VTG Messages... ");}
      sendUBX(setVTG, sizeof(setVTG));
      gpsSetSuccess += getUBX_ACK(&setVTG[2]);}
    if (!VTGoption && gpsSetSuccess == 3 && serialOutput){Serial.println("NMEA VTG Message Deactivation Failed!");}
    gpsSetSuccess = 0;
    
    //Set Interference Thresholds
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Setting Interferece Thresholds... ");}
      sendUBX(setJam, sizeof(setJam));
      gpsSetSuccess += getUBX_ACK(&setJam[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Interference Settings Failed!");}
    gpsSetSuccess = 0;
    
    //Turn Off NMEA GLL Messages
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Deactivating NMEA GLL Messages... ");}
      sendUBX(setGLL, sizeof(setGLL));
      gpsSetSuccess += getUBX_ACK(&setGLL[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("NMEA GLL Message Deactivation Failed!");}
    gpsSetSuccess = 0;
    
    //Increase Baud-Rate on M8Q for faster GPS updates
    while(gpsVersion == 2 && gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Setting Ublox Baud Rate 38400... ");}
      sendUBX(setBaudRate, sizeof(setBaudRate));
      gpsSetSuccess += getUBX_ACK(&setBaudRate[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Ublox Baud Rate 38400 Failed!");}
    else if(gpsVersion == 2){
      HWSERIAL->end();
      HWSERIAL->clear();
      HWSERIAL->begin(38400);}}

    if(gpsVersion == 4){
      ultGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      ultGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      ultGPS.sendCommand(PMTK_SET_BAUD_57600);}
  
  }//end configGPS

void GPSpowerSaveMode(bool serialOutput, byte gpsVersion){

  //configure UBLOX chips
  if(gpsVersion < 4){
    
    byte gpsSetSuccess = 0;
    if(serialOutput){Serial.println("Configuring u-Blox GPS Power Save mode... ");}
  
    //Generate the configuration string to enable power save mode (RXM)
    byte enablePwrSv[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
    
    //Generate configuration string to put reciever into power save mode (PM2)
    byte setPwr2[] = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x0E, 0x90, 0x40, 0x01, 0x20, 0x4E, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x67, 0x83}; 
  
    //Generate configuration string to put receiver into 1Hz update mode
    //byte setDataRate1Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};//once per second
    byte setDataRate01Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00, 0x4D, 0xDD};//once per 10 seconds
  
    //Enable Power Save Mode
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Enabling Power Save Mode... ");}
      sendUBX(enablePwrSv, sizeof(enablePwrSv));
      gpsSetSuccess += getUBX_ACK(&enablePwrSv[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Enable Power Save Failed!");}
    gpsSetSuccess = 0;
  
    //Config Power Save Mode
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Config Power Save Mode... ");}
      sendUBX(setPwr2, sizeof(setPwr2));
      gpsSetSuccess += getUBX_ACK(&setPwr2[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Commit PSM Message Failed!");}
    gpsSetSuccess = 0;
  
    //Set Data Update Rate
    while(gpsSetSuccess < 3) {
      if(serialOutput){Serial.print("Setting Data Update Rate... ");}
      sendUBX(setDataRate01Hz, sizeof(setDataRate01Hz));  //Send UBX Packet
      gpsSetSuccess += getUBX_ACK(&setDataRate01Hz[2]);}
    if (gpsSetSuccess == 3 && serialOutput){Serial.println("Data update mode configuration failed!");}
    gpsSetSuccess = 0;}

  //Set lower update rate for Adafruit Ultimate GPS
  if(gpsVersion == 4){ultGPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);}
  
}//End PowerSave Mode

void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;}
    
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;}

void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    HWSERIAL->write(UBXmsg[i]);
    HWSERIAL->flush();}
    
  HWSERIAL->println();
  HWSERIAL->flush();}

byte getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (HWSERIAL->available()) {
      incoming_char = HWSERIAL->read();
      if (incoming_char == ackPacket[i]) {
        i++;}
      else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;}}
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      Serial.println("ACK Timeout");
      return 5;}
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;}}

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;}
  
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.print("ACK Received! ");
    printHex(ackPacket, sizeof(ackPacket));
    return 10;}
  else {
    Serial.print("ACK Checksum Failure: ");
    printHex(ackPacket, sizeof(ackPacket));
    delay(1000);
    return 1;}}

void printHex(uint8_t *data, uint8_t length){ // prints 8-bit data in hex

  char tmp[length*2+1];
  byte first ;
  int j=0;
  for (byte i = 0; i < length; i++){
    
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)7;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)7; 
    else tmp[j] = first;
    j++;}
    
  tmp[length*2] = 0;
  for (byte i = 0, j = 0; i < sizeof(tmp); i++) {
    Serial.print(tmp[i]);
    if (j == 1) {
      Serial.print(" "); 
      j = 0;}
    else j++;}
  Serial.println();}