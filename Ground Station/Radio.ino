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

//---------------------------
//433 MHz Frequency List
//---------------------------
float radioFreq[9] = {433.250, 433.500, 433.625, 433.750, 433.875, 434.000, 434.125, 434.250, 434.400};

//---------------------------
//915 MHz LoRa channel list on ISM band
//---------------------------
const float FHSSchnl[64] = {
  902.3,  902.5,  902.7,  902.9,  903.1,  903.3,  903.5,  903.7,  903.9,  904.1,
  904.3,  904.5,  904.7,  904.9,  905.1,  905.3,  905.5,  905.7,  905.9,  906.1,  
  906.3,  906.5,  906.7,  906.9,  907.1,  907.3,  907.5,  907.7,  907.9,  908.1,  
  908.3,  908.5,  908.7,  908.9,  909.1,  909.3,  909.5,  909.7,  909.9,  910.1,  
  910.3,  910.5,  910.7,  910.9,  911.1,  911.3,  911.5,  911.7,  911.9,  912.1,  
  912.3,  912.5,  912.7,  912.9,  913.1,  913.3,  913.5,  913.7,  913.9,  914.1,
  914.3,  914.5,  914.7,  914.9};
  
byte radioMode = 0;
bool TX = false;

bool radioBegin(){

  uint8_t debugVal;
  boolean successFlag = true;
  
  delay(10);
  digitalWrite(activeRadio->rst, LOW);
  delayMicroseconds(100);
  digitalWrite(activeRadio->rst, HIGH);
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
  if(!activeRadio->enable){return false;}
  delay(10);
  debugVal = read8(RegOpMode);
  if(debugVal != SleepMode){
    Serial.print("Set Sleep Mode Failed: "); 
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //set radio power
  setRadioPWR(activeRadio->TXpwr);
  
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

  //if FHSS, set the user frequency to the closest LoRa channel as the hail frequency
  if(activeRadio->FHSS){
    activeRadio->hailChnl = (uint8_t)(5*(activeRadio->frq - 902.300F));
    activeRadio->frq = getFreq(activeRadio->hailChnl);
    Serial.print("Radio ");Serial.print(activeRadio->num);Serial.print(" hail channel ");
    Serial.print(activeRadio->hailChnl);Serial.print(", freq: ");Serial.println(activeRadio->frq, 3);}

  return successFlag;}
 
bool setRadioPWR(uint8_t pwr){
  
  #define regPaDac    0x4D
  #define regPaConfig 0x09
  
  boolean successFlag = true;
  uint8_t debugVal;
  
  //do this like RadioHead RFM95 does it
  //range is 2 to 20 dbm
  if(pwr > 20){pwr = 20;}
  if(pwr < 2){pwr = 2;}
  
  //enable PA_DAC if power is above 17
  if(pwr > 17){
    write8(regPaDac, 0b01010111);
    pwr -= 3;
    delay(10);
    debugVal = read8(regPaDac);
    if(debugVal != 0b01010111){
      successFlag = false;
      Serial.print("Set RegPaDac Failed: ");Serial.println(debugVal, HEX);}}

  //write to the power config register
  uint8_t radioPwr = (0x80 | (pwr-2));
  Serial.print("Power Set: ");Serial.println(radioPwr, HEX);
  write8(regPaConfig, radioPwr);
  delay(10);
  debugVal = read8(regPaConfig);
  if(debugVal != radioPwr){
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
  else{
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
  len = pktLen;
  
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

float readRadioFreq(){

  //uint32_t frf = ((freq * 1000000.0) / 32000000)*524288;
  uint32_t frf;
  union{
    uint32_t val;
    uint8_t Byte[4];
  } ulongUnion;

  ulongUnion.val = frf;
  
  #define RegFrfMsb 0x06
  #define RegFrfMid 0x07
  #define RegFrfLsb 0x08

  //read back the frequency
  ulongUnion.Byte[2] = read8(RegFrfMsb);
  ulongUnion.Byte[1] = read8(RegFrfMid);
  ulongUnion.Byte[0] = read8(RegFrfLsb);
  frf = ulongUnion.val;

  float frq = 32*(float)frf/524288.0F;

  return frq;}

bool setRadioFreq(float freq){

  boolean successFlag = true;

  //read the current mode
  uint8_t currentMode = read8(RegOpMode);
  
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

  //delay(10);

  //read back the frequency
  ulongUnion.Byte[2] = read8(RegFrfMsb);
  ulongUnion.Byte[1] = read8(RegFrfMid);
  ulongUnion.Byte[0] = read8(RegFrfLsb);

  if(ulongUnion.val != frf){
    successFlag = false;
    Serial.print("Set Freq Fail: ");
    Serial.println(ulongUnion.val);}

  //Set back to the previous mode
  radioSetMode(RXmode);
  
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

  uint8_t hopChnl;
  float freq;
  
  activeRadio->chnlUsed++;
  Serial.print("chnlUsed = ");Serial.println(activeRadio->chnlUsed);
  if(activeRadio->chnlUsed == 1){hopChnl = activeRadio->nextChnl;if(debugSerial){Serial.print("Hopping to Channel: ");Serial.println(hopChnl);}}
  else if(activeRadio->chnlUsed == 2){hopChnl = activeRadio->nextChnl2;if(debugSerial){Serial.print("Hopping to Channel: ");Serial.println(hopChnl);}}
  else if(activeRadio->chnlUsed >= 3){hopChnl = activeRadio->hailChnl;if(debugSerial){Serial.print(F("Moving to sync chnl: ")); Serial.println(hopChnl); activeRadio->syncFreq = true;}}
  activeRadio->chnl1 = hopChnl;
  freq = getFreq(activeRadio->chnl1);
  activeRadio->frq = freq;
  setRadioFreq(freq);
  activeRadio->lastHopTime = micros();
  Serial.print("Timestamp: ");Serial.println(activeRadio->lastHopTime);}

void syncPkt(byte rxPacket[]){
  
  activeRadio->signalEst = true;
  activeRadio->syncFreq = false;
  lostSignalTime = 1400000UL;
  //parse packet
  activeRadio->chnl1 = (byte)rxPacket[1];
  activeRadio->nextChnl = (byte)rxPacket[2];
  activeRadio->nextChnl2= (byte)rxPacket[3];
  activeRadio->chnlUsed = 0;
  hopFreq();
  if(debugSerial){
    Serial.print("Moving to Channel ");Serial.println(activeRadio->nextChnl);
    dispPktInfo();}}

void dispPktInfo(){
  Serial.print(F("Current Chnl: "));Serial.print(activeRadio->chnl1);Serial.print(F(", "));Serial.print(getFreq(activeRadio->chnl1));Serial.println(F("MHz"));
  Serial.print(F("Next Chnl: "));Serial.print(activeRadio->nextChnl);Serial.print(F(", Next Freq: "));Serial.print(getFreq(activeRadio->nextChnl), 3); Serial.println("MHz");
  Serial.print("3rd Chnl: ");Serial.print(activeRadio->nextChnl2);Serial.print(", ");Serial.print(getFreq(activeRadio->nextChnl2),3);Serial.println("MHz");}

float getFreq(byte chnl){
  
  float frq;
  
  if(activeRadio->frq < 800.000F){
    if(chnl>=sizeof(radioFreq)){chnl = sizeof(radioFreq)-1;}
    frq = radioFreq[chnl];}
  
  else{
    if(chnl>=sizeof(FHSSchnl)){chnl = sizeof(FHSSchnl)-1;}
    frq = FHSSchnl[chnl];}
  
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
  digitalWrite(activeRadio->cs, LOW);

  //Read data
  SPI.transfer(reg);
  val = SPI.transfer(0);
  
  //end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return val;}

bool write8(byte reg, byte data){

  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);
  
  //Send data
  SPI.transfer(writeMask | reg);
  SPI.transfer(data);

  //end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return true;}
  
bool burstWrite(byte reg, uint8_t *data, byte len){
  
  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);
  
  //Send data
  SPI.transfer(writeMask | reg);
  for(byte i = 0; i < len; i++){SPI.transfer(*(data+i));}
  Serial.println(" ");

  //end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return true;}

bool burstRead(byte reg, uint8_t *data, byte len){

  //begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);

  //Read data
  SPI.transfer(reg);
  for(byte i = 0; i < len; i++){*(data+i) = SPI.transfer(0);}
  
  //end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return true;}
