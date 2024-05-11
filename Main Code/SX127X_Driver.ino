//These are optimized drivers for the Semtech SX1276, SX1277, SX1278, and SX1279 packet radios
//-----------------------------------------------
//Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
//19 Nov 23: Version 1 broken out from the previous Telemetry.ino file to enable other radios with external libraries
//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//beginSX127X(): starts the radio
//setPwrSX127X(): sets the power in dDm
//sendPktSX127X(): sends the packet
//setFreqSX127X(): sets the radio frequency
//setModeSX127X(): sets the radio mode
//clearFlagsSX127X(): clears the interrupt flags

bool beginSX127X(uint8_t radioRST){

  radioBus.spiSet = SPISettings(10000000, MSBFIRST, SPI_MODE0);
  radioBus.cs = pins.radioCS;
  radioBus.writeMask = 0x80;
  radioBus.readMask = 0x00;
  radioBus.incMask = 0x00;
  startSPI(&radioBus, sensors.radioBusNum);

  uint8_t debugVal;
  boolean successFlag = true;

  //Set interrupts
  pinMode(pins.radioIRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(pins.radioIRQ), clearIRQ, RISING);
  
  //reset radio
  digitalWrite(pins.radioCS, HIGH);
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

  //Set DIO0 to fire when transmit is complete
  #define RegDioMapping1 0x40
  write8(RegDioMapping1, 0x40);
  delay(10);
  debugVal = read8(RegDioMapping1);
  if(debugVal != 0x40){
    Serial.print("Set DIO0 Failed: ");
    Serial.println(debugVal, HEX);}
    
  //clear interrupts
  #define RegIrqFlags     0x12
  #define RegIrqFlagsMask 0x11
  write8(RegIrqFlagsMask, 0x00);
  write8(RegIrqFlags, 0xFF);  

  if(successFlag){Serial.println("SX127X Radio OK!");}

  return successFlag;}
 
bool setPwrSX127X(int8_t pwr){
  
  #define regPaDac    0x4D
  #define regPaConfig 0x09
  
  boolean successFlag = true;
  int8_t debugVal;
  
  //set bus
  activeBus = &radioBus;
  
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
    int8_t radioPwr = (0x80 | (pwr-2));
    if(settings.testMode){Serial.print("Power Set: ");Serial.println(radioPwr, HEX);}
    write8(regPaConfig, radioPwr);
    delay(10);
    debugVal = read8(regPaConfig);
    if(debugVal != radioPwr){
      successFlag = false;
      Serial.print("Set RegPaConfig Failed: ");Serial.println(debugVal, HEX);}
  
  return successFlag;}

bool sendPktSX127X(uint8_t* data, uint8_t len){
  
  #define RegFIFO          0x00
  #define RegOpMode        0x01
  #define RegFifoAddrPtr   0x0D
  #define RegPayloadLength 0x22

  //set bus
  activeBus = &radioBus;
  
  //put radio in standby
  write8(RegOpMode, StandbyMode);
  radioMode = 1;

  //clear flags
  uint8_t myFlags = read8(RegIrqFlags);
  if(myFlags != 0x00){Serial.print("Prior TX Error: ");Serial.println(myFlags);}
  write8(RegIrqFlags, 0x00);
  
  //set the FIFO pointer to start of the buffer
  write8(RegFifoAddrPtr, 0x00);

  //write to FIFO
  burstWrite(RegFIFO, data, len);

  //write the payload length
  write8(RegPayloadLength, len);

  //send the packet
  write8(RegOpMode, TXmode);

  //set flag
  radioFnctn = TXmode;

  //indicate the radio is transmitting
  TX = true;
  
  return true;}

bool config70cmSX127X(){

  //NOTE: the maximum allowable bandwidth on 70cm Ham band is 100kHz.  To keep the same data rate
  //we need to halve the bandwidth but increase the bitrate.  SF6 is the only way to do this, 
  //but it requires a special configuration

  #define RegDectectOptimize    0x31
  #define RegDetectionThreshold 0x37

  //set bus
  activeBus = &radioBus;
  bool successFlag = true;

  //configure the modem to 62.5kHz bw, 4/5 cr, implicit header
  uint8_t modemConfig = 0b01100011;
  write8(RegModemConfig1, modemConfig);
  delay(10);
  uint8_t debugVal = read8(RegModemConfig1);
  if(debugVal != modemConfig){
    Serial.print("Config1 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //configure the modem to sf6, normal TX mode, CRC on, RX MSB 0
  modemConfig = 0b01100100;
  write8(RegModemConfig2, modemConfig);
  delay(10);
  debugVal = read8(RegModemConfig2);
  if(debugVal != modemConfig){
    Serial.print("Config2 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //Set the bit field DetectionOptimize of register RegLoRaDetectOptimize to value "0b101"
  modemConfig = 0b11100101;
  write8(RegDectectOptimize, modemConfig);
  delay(10);
  debugVal = read8(RegDectectOptimize);
  if(debugVal != modemConfig){
    Serial.print("RegDectectOptimize Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //Write 0x0C in the register RegDetectionThreshold
  modemConfig = 0x0C;
  write8(RegDetectionThreshold, modemConfig);
  delay(10);
  debugVal = read8(RegDetectionThreshold);
  if(debugVal != modemConfig){
    Serial.print("RegDetectionThreshold Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;}

  //set flag for implicit header
  implicitHdr = true;

  return successFlag;}

bool setFreqSX127X(float freq){

  boolean successFlag = true;

  //set bus
  activeBus = &radioBus;
  
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

  return successFlag;}

bool setModeSX127X(uint8_t mode){

  #define RegOpMode      0x01
  #define RegDioMapping1  0x40

  //set bus
  activeBus = &radioBus;
  
  //set radio mode
  write8(RegOpMode, mode);
  radioMode = mode;

  delay(1);
  
  //check mode
  uint8_t val = read8(RegOpMode);
  if(val != (LoRaMode | mode)){
    Serial.print("Set Mode Failed: ");
    Serial.println(val, HEX);
    return false;}

  //set the IRQ to fire on TX sent
  if(mode == TXmode){
    write8(RegDioMapping1, 0x40);
    radioFnctn = TXmode;}

  //set the IRQ to fire on RX complete
  if(mode == RXmode){
    write8(RegDioMapping1, 0x00);
    radioFnctn = RXmode;}

  return true;}
  
void clearFlagsSX127X(){

  uint32_t TXtime;
  TXtime = micros() - TXstartTime;
  TX = false;

  //set bus
  activeBus = &radioBus;

  //read the IRQ flags
  uint8_t debugVal = read8(RegIrqFlags);

  if(radioDebug && settings.testMode){
    if(debugVal == 0x08){
      Serial.print(", TX Done: ");Serial.print(debugVal, HEX);
      Serial.print(", TX Time: ");Serial.println(TXtime);}
    else{Serial.print(", TX error: ");Serial.println(debugVal, BIN);}}
  
  //clear flag
  write8(RegIrqFlags, 0xFF);

  //set the radio code
  radioMode = 1;

  noInterrupts();
  clearTX = false;
  interrupts();}