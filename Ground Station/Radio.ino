/*Functions:
 * bool radioBegin(): starts code and checks radio functionality
 * bool setRadioPWR(uint8_t pwr): sets output power (0 - 20 dbm)
 * bool setRadioFreq(float freq): sets the radio frequency (expressed as MHz, ie 434.250
 * bool radioSendPkt(uint8_t* data, uint8_t len): send a radio packet from a byte array, with length len
 * bool radioRecvPkt(uint8_t* data): recieve a packet and place data from FIFO into array "data"
 * bool radioSetMode(uint8_t mode): sets the radio mode, mostly used for setting RX continuous mode in setup
 * 
 *SPI transfer functions:
 *uint8_t read8(byte reg)
 *bool write8(byte reg, byte data)
 *bool burstRead(byte reg, uint8_t *data, byte len)
 *bool burstWrite(byte reg, uint8_t *data, byte len)
 */
byte radioMode = 0;
bool TX = false;

bool radioBegin(uint8_t CS, uint8_t radioRST){

  radioCS = CS;

  uint8_t debugVal;
  boolean successFlag = true;
  
  pinMode(radioCS, OUTPUT);
  pinMode(radioRST, OUTPUT);
  
  //reset radio
  digitalWrite(radioCS, HIGH);
  digitalWrite(radioRST, HIGH);
  delay(10);
  digitalWrite(radioRST, LOW);
  delayMicroseconds(100);
  digitalWrite(radioRST, HIGH);
  delay(10);
  
  #define RegOpMode       0x01
  #define RegPreambleMsb  0x20
  #define RegPreambleLsb  0x21
  #define RegModemConfig1 0x1D
  #define RegModemConfig2 0x1E
  #define RegModemConfig3 0x26
  uint8_t radioSetMode;
  
  //set radio to sleep mode
  write8(RegOpMode, SleepMode);
  radioMode = 0;
  delay(10);
  debugVal = read8(RegOpMode);
  if(debugVal != SleepMode){
    Serial.print("Set Sleep Mode Failed: "); 
    Serial.println(debugVal, BIN);
    successFlag = false;}
  
  //set radio to Long Range Mode
  radioSetMode = LoRaMode;
  write8(RegOpMode, radioSetMode);
  radioMode = 1;
  delay(10);
  debugVal = read8(RegOpMode);
  if (debugVal != radioSetMode){
    Serial.print("Set LoRa Mode Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //configure the modem to 125kHz bw, 4/5 cr, explicit header
  uint8_t modemConfig = 0b01110010;
  write8(RegModemConfig1, modemConfig);
  delay(10);
  debugVal = read8(RegModemConfig1);
  if(debugVal != modemConfig){
    Serial.print("Config1 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //configure the modem to sf7, normal TX mode, CRC on, RX MSB 0
  modemConfig = 0b01110100;
  write8(RegModemConfig2, modemConfig);
  delay(10);
  debugVal = read8(RegModemConfig2);
  if(debugVal != modemConfig){
    Serial.print("Config2 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //Set the Automatic Gain Control (AGC)
  modemConfig = 0x04;
  write8(RegModemConfig3, modemConfig);
  delay(10);
  debugVal = read8(RegModemConfig3);
  if(debugVal != modemConfig){
    Serial.print("Config3 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}
    
  //set the preamble length to 8
  uint16_t preambleLength = 8;
  union{
    uint16_t val;
    byte Byte[2];
  } intUnion;
  intUnion.val = preambleLength;
  write8(RegPreambleLsb, intUnion.Byte[0]);
  write8(RegPreambleMsb, intUnion.Byte[1]);
  delay(10);
  debugVal = read8(RegPreambleLsb);
  intUnion.Byte[0] = debugVal;
  debugVal = read8(RegPreambleMsb);
  intUnion.Byte[1] = debugVal;
  if(intUnion.val != preambleLength){
    Serial.print("Set Preamble Len Failed: ");
    Serial.println(intUnion.val, DEC);
    successFlag = false;}

  //Set the FIFO buffer to the start
  #define RegFifoTxBase 0x0E
  write8(RegFifoTxBase, 0x00);
  delay(10);
  debugVal = read8(RegFifoTxBase);
  if(debugVal != 0x00){
    Serial.print("Set FIFO TX Base Failed: ");
    Serial.println(debugVal, BIN);}

  //Set the FIFO Rx Buffer base address to the start
  #define RegFifoRxBase 0x0F
  write8(RegFifoRxBase, 0x00);
  delay(10);
  debugVal = read8(RegFifoRxBase);
  if(debugVal != 0x00){
    Serial.print("Set FIFO RX Base Failed: ");
    Serial.println(debugVal, BIN);}
    
  //Set radio to Standby Mode
  radioSetMode = StandbyMode;
  write8(RegOpMode, radioSetMode);
  delay(10);
  debugVal = read8(RegOpMode);
  if(debugVal != (LoRaMode | radioSetMode)){
    Serial.print("Set Standby Mode Failed: "); 
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //clear interrupts
  #define RegIrqFlags     0x12
  #define RegIrqFlagsMask 0x11
  write8(RegIrqFlagsMask, 0x00);
  write8(RegIrqFlags, 0xFF);  

  return successFlag;}
 
bool setRadioPWR(uint8_t pwr){
  
  #define regPaDac    0x4D
  #define regPaConfig 0x09
  
  boolean successFlag = true;
  uint8_t debugVal;
  radioCS = radio1CS;
  
  //do this like RadioHead RFM95 does it
  //range is 2 to 20 dbm
  if(pwr > 20){pwr = 20;}
  if(pwr < 2){pwr = 2;}
  
  //enable PA_DAC if power is above 17
  if(pwr > 17){
    write8(regPaDac, 0x17);
    pwr -= 3;
    delay(10);
    debugVal = read8(regPaDac);
    if(debugVal != 0x17){
      successFlag = false;
      Serial.print("Set RegPaDac Failed: ");Serial.println(debugVal, HEX);}}
  else{
    //disable DAC
    write8(regPaDac, 0x14);
    delay(10);
    debugVal = read8(regPaDac);
    if(debugVal != 0x14){
      successFlag = false;
      Serial.print("Set RegPaDac Failed: ");Serial.println(debugVal, HEX);}}

  //write to the power config register
  write8(regPaConfig, (0x80 | (pwr-2)));
  delay(10);
  debugVal = read8(regPaConfig);
  if(debugVal != (0x80 | (pwr-2))){
    successFlag = false;
    Serial.print("Set RegPaConfig Failed: ");Serial.println(debugVal, HEX);}
  
  return successFlag;}

bool radioRecvPkt(uint8_t* data){

  //Check Error Flags
  #define RegIrqFlags          0x12
  uint8_t flags = read8(RegIrqFlags);
  bool flagError = false;

  //good packet recieved
  if(flags == 0x40 || flags == 0x50){Serial.print("Good Packet Recieved: ");Serial.println(flags, HEX);}

  //bad packet recieved
  else if(flags != 0x00){
    Serial.print("IRQ Flag error: ");Serial.println(flags, BIN);
    flagError = true;}
    
  //clear the flags
  write8(RegIrqFlags, 0xFF);

  //if there was an error, then do not process the packet
  if(flagError){return false;}

  //set the RegFifoAddrPtr to RegFifoRxCurrentAddr
  #define RegFifoAddrPtr       0x0D
  #define RegFifoRxCurrentAddr 0x10
  uint8_t val = read8(RegFifoRxCurrentAddr);
  write8(RegFifoAddrPtr, val);

  //read the number of bytes recieved
  #define RegRxNbBytes 0x13
  uint8_t pktLen = read8(RegRxNbBytes);
  
  //read the packet
  burstRead(0x00, data, pktLen);

  //get SNR
  #define RegPktSNR 0x19
  int8_t pktSNR = (int8_t)read8(RegPktSNR);
  
  //get Rssi
  #define RegPktRssiValue 0x1A
  pktRssi = read8(RegPktRssiValue);

  //Calculate Rssi from datasheet
  if(pktSNR < 0){pktRssi += pktSNR/4;}
  else{pktRssi = (int16_t)(pktRssi * 16 / 15);}

  if(radio1Freq > 719.000){pktRssi -= 157;}
  else{pktRssi -= 164;}

  return true;}

bool setRadioFreq(float freq){

  boolean successFlag = true;
  
  //set mode to standby
  write8(RegOpMode, StandbyMode);

  uint32_t frf = ((freq * 1000000.0) / 32000000)*524288;
  union{
    uint32_t val;
    uint8_t Byte[4];
  } ulongUnion;

  ulongUnion.val = frf;
  
  #define RegFrfMsb 0x06
  #define RegFrfMid 0x07
  #define RegFrfLsb 0x08
  
  write8(RegFrfMsb, ulongUnion.Byte[2]);
  write8(RegFrfMid, ulongUnion.Byte[1]);
  write8(RegFrfLsb, ulongUnion.Byte[0]);

  delay(10);

  //read back the frequency
  ulongUnion.Byte[2] = read8(RegFrfMsb);
  ulongUnion.Byte[1] = read8(RegFrfMid);
  ulongUnion.Byte[0] = read8(RegFrfLsb);

  if(ulongUnion.val != frf){
    successFlag = false;
    Serial.print("Set Freq Fail: ");
    Serial.println(ulongUnion.val);}
  else{
    Serial.print("Set Freq Success: ");
    Serial.println(ulongUnion.val);}

  return successFlag;}

bool radioSetMode(uint8_t mode){

  #define RegOpMode      0x01
  #define RegDioMapping1  0x40
  
  //set radio mode
  write8(RegOpMode, mode);
  radioMode = mode;

  delay(1);
  
  //check mode
  uint8_t val = read8(RegOpMode);
  if(val != (LoRaMode | mode)){Serial.print("Set Mode Failed: ");Serial.println(val, HEX);return false;}

  //set the IRQ to fire on TX sent
  if(mode == TXmode){
    write8(RegDioMapping1, 0x40);
    radioFnctn = TXmode;}

  //set the IRQ to fire on RX complete
  if(mode == RXmode){
    write8(RegDioMapping1, 0x00);
    radioFnctn = RXmode;}

  return true;}

void hopFreq(){

  byte hopChnl;

  chnlUsed++;
  if(chnlUsed == 1){hopChnl = nextChnl;if(debugSerial){Serial.println(hopChnl);}}
  else if(chnlUsed == 2){hopChnl = nextChnl2;if(debugSerial){Serial.println(hopChnl);}}
  else if(chnlUsed >= 3){hopChnl = hailChnl_1;if(debugSerial){Serial.print(F("Moving to sync chnl: ")); Serial.println(hopChnl); syncFreq = true;}}
  chnl1 = hopChnl;
  setRadioFreq(getFreq(chnl1));
  lastHopTime = micros();}

void syncPkt(){
  
  byte currentChnl = chnl1;
  syncFreq = false;
  //parse packet
  chnl1 = (byte)dataPacket1[1];
  nextChnl = (byte)dataPacket1[2];
  nextChnl2= (byte)dataPacket1[3];
  chnlUsed = 0;
  hopFreq();
  if(debugSerial){
    Serial.print("------ Sync Packet Recieved ------");Serial.println(currentChnl);Serial.print("------ ");
    dispPktInfo();}}

void dispPktInfo(){
  freq1 = getFreq(chnl1);
  if(debugSerial){
    Serial.print(F("Current Chnl: "));Serial.print(chnl1);
    Serial.print(F(", "));Serial.print(freq1);Serial.println(F("MHz"));
    Serial.print(F("Next Chnl: "));Serial.print(nextChnl);Serial.print(F(", Next Freq: "));}
  freq1 = getFreq(nextChnl);
  if(debugSerial){Serial.print(freq1, 3); Serial.println("MHz");}
}

float getFreq(byte chnl){
  
  float frq;
  
  if(band433){frq = 433.250 + chnl * 0.125;}
  
  else{frq = 902.300F + 0.200F * chnl;}
  
  return frq;}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//SPI transfer functions
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t read8(byte reg){

  byte val;
  
  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(radioCS, LOW);

  //Read data
  SPI.transfer(reg);
  val = SPI.transfer(0);
  
  //end SPI transaction
  digitalWrite(radioCS, HIGH);
  SPI.endTransaction();

  return val;}

bool write8(byte reg, byte data){

  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(radioCS, LOW);
  
  //Send data
  SPI.transfer(writeMask | reg);
  SPI.transfer(data);

  //end SPI transaction
  digitalWrite(radioCS, HIGH);
  SPI.endTransaction();

  return true;}
  
bool burstWrite(byte reg, uint8_t *data, byte len){
  
  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(radioCS, LOW);
  
  //Send data
  SPI.transfer(writeMask | reg);
  for(byte i = 0; i < len; i++){SPI.transfer(*(data+i));}
  Serial.println(" ");

  //end SPI transaction
  digitalWrite(radioCS, HIGH);
  SPI.endTransaction();

  return true;}

bool burstRead(byte reg, uint8_t *data, byte len){

  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(radioCS, LOW);

  //Read data
  SPI.transfer(reg);
  for(byte i = 0; i < len; i++){*(data+i) = SPI.transfer(0);}
  
  //end SPI transaction
  digitalWrite(radioCS, HIGH);
  SPI.endTransaction();

  return true;}
