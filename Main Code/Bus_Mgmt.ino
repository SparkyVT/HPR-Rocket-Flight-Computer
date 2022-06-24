//--------Supported Buses---------
//Teensy 3.2  3.5  3.6  4.0  4.1
//I2C0    Y    Y    Y    Y    Y
//I2C1    Y    Y    Y    Y    Y
//I2C2    N    Y    Y    Y    Y
//I2C3    N    N    Y    N    N    
//SPI0    Y    Y    Y    Y    Y
//SPI1    N    Y    Y    Y    Y
//SPI2    N    Y    Y    Y    N
//SDIO    N    Y    Y    Y    Y
//SER1    Y    Y    Y    Y    Y
//SER2    Y    Y    Y    Y    Y
//SER3    Y    Y    Y    Y    Y
//SER4    N    Y    Y    Y    Y
//SER5    N    Y    Y    Y    Y
//SER6    N    Y    Y    Y    Y
//SER7    N    N    N    Y    Y
//SER8    N    N    N    N    Y
//-----------Change Log------------
//03 Jan 22: Created to support Teensy 3.2, 4.0, and 4.1
//18 APR 22: Updated to support any device on any bus
//16 JUN 22: Streamlined bus usage with more efficient use of pointers
//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//startI2C(): starts the I2C bus
//testSensor(): tests to see if there is a sensor at the given I2C address
//startSPI(): starts the SPI bus
//write8(): writes 8 bits to the register 
//write16(): writes 16 bits to the register
//read8(): reads 8 bits from the register
//burstRead(): reads a given number of bytes from the starting register
//setHWSERIAL(): sets the active UART bus for the GPS to the user specified bus
//------Code Start---------

//***************************************************************************
//Generic UART Hardware Serial Functions
//***************************************************************************
void setHWSERIAL(){

  Serial.print(F("Setting HW Serial"));Serial.print(sensors.gpsBusNum);Serial.print(F("..."));

  switch(sensors.gpsBusNum){

    case 1: 
      HWSERIAL = &Serial1;
      break;
      
    case 2:
      HWSERIAL = &Serial2;
      break;
      
    case 3:
      HWSERIAL = &Serial3;
      break;
      
    //Teensy 3.5, 3.6, 4.0, 4.1
    #if defined (__MK64FX512__) || defined (__MK66FX1M0__) || defined (__IMXRT1062__)
      case 4:
        HWSERIAL = &Serial4;
        break;

      case 5:
        HWSERIAL = &Serial5;
        break;
        
      case 6:
        HWSERIAL = &Serial6;
        break;
      
    #endif

    //Teensy 4.0 and Teensy 4.1
    #if defined (__IMXRT1062__)

      case 7:
        HWSERIAL = &Serial7;
        break;

    #endif

    //Teensy 4.1
    #if defined (ARDUINO_TEENSY41)

      case 8:
        HWSERIAL = &Serial8;
        break;
    #endif
  }
  Serial.println(F("Done!"));}
//***************************************************************************
//Generic I2C Functions
//***************************************************************************
void startI2C(_bus *myBus, uint8_t busNum){

  //start the bus
  if(settings.testMode){Serial.print("Starting i2c bus ");}
  
  myBus->type = 'I';
  
  switch (busNum){

    //All Teensy 3.X versions
    #if defined (__MK64FX512__) || defined (__MK66FX1M0__) || defined (__MK20DX256__)
      case 0: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire;
              break;
      case 4: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire;
              busNum = 0;
              break;
    #endif

    //Teensy 3.5 and Teensy 3.6
    #if defined (__MK64FX512__) || defined (__MK66FX1M0__)
      case 1: Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire1;
              break;
      case 2: Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire2;
              break;
      case 5: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_7_8, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire;
              busNum = 0;
              break;
      case 6: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_33_34, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire;
              busNum = 0;
              break;
      case 7: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_47_48, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire;
              busNum = 0;
              break;
    #endif

    //Teensy 3.2
    #if defined (__MK20DX256__)
      case 1: Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, myBus->i2cRate);
              myBus->wire = &Wire1;
              busNum = 1;
              break;
    #endif

    //Teensy 3.6 extra bus
    #if defined (__MK66FX1M0__)
      case 3: Wire3.begin(I2C_MASTER, 0x00, I2C_PINS_56_57, I2C_PULLUP_EXT, myBus->i2cRate);
               myBus->wire = &Wire3;
               break;
    #endif

    //Teensy 4.0 and Teensy 4.1
    #if defined (__IMXRT1062__)
       case 0: Wire.begin();
               Wire.setClock(myBus->i2cRate);
               myBus->wire = &Wire;
               break;
      case 1:  Wire1.begin();
               Wire1.setClock(myBus->i2cRate);
               myBus->wire = &Wire1;
               break;
      case 2:  Wire2.begin();
               Wire2.setClock(myBus->i2cRate);
               myBus->wire = &Wire2;
               break;
      default: Wire.begin();
               Wire.setClock(400000);
               myBus->wire = &Wire;
               break;
    #endif

    //All Teensy 3.X versions
    #if defined (__MK64FX512__) || defined (__MK66FX1M0__) || defined (__MK20DX256__)
      default: Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
              myBus->wire = &Wire;
              break;
    #endif
  }

  //set the active bus to the current bus
  activeBus = myBus;
  
  if(settings.testMode){Serial.print(busNum);Serial.print(F("..."));}}

bool testSensor(byte address) {
  activeBus->wire->beginTransmission(address);
  if(activeBus->wire->endTransmission() == 0){return true;}
  else{return false;}}

//***************************************************************************
//Generic SPI Functions
//***************************************************************************
void startSPI(_bus *myBus, uint8_t busNum){
    
  if(settings.testMode){Serial.print("Starting SPI bus "); Serial.print(busNum);Serial.print(F("..."));}

  pinMode(myBus->cs, OUTPUT);
  digitalWrite(myBus->cs, HIGH);
            
  myBus->type = 'S';
  
  switch (busNum){

    //All boards
    case 0: SPI.begin();
            myBus->spi = &SPI;
            break;

    //Only for Teensy 3.5, 3.6, 4.0, and 4.1
    #if defined (__MK64FX512__) || defined (__MK66FX1M0__) || defined (__IMXRT1062__)
    
    case 1: SPI1.begin();
            myBus->spi = &SPI1;
            break;
    case 2: SPI2.begin();
            myBus->spi = &SPI2;
            break;
    #endif

    default: SPI.begin();
             myBus->spi = &SPI;
             break;}
  //set the bus to the current bus
  activeBus = myBus;}//end startBus
                
//***************************************************************************
//Common Read / Write Functions
//***************************************************************************
uint8_t read8(byte reg) {

  uint8_t val;

  //I2C Read
  if(activeBus->type == 'I'){
    activeBus->wire->setClock(activeBus->i2cRate);
    activeBus->wire->beginTransmission(activeBus->i2cAddress);
    activeBus->wire->write(reg);
    activeBus->wire->endTransmission(false);
    activeBus->wire->requestFrom(activeBus->i2cAddress, (byte)1);
    while (activeBus->wire->available() < 1) {};
    val = activeBus->wire->read();}
    
  //SPI Read
  else{
    //begin SPI transaction
    activeBus->spi->beginTransaction(activeBus->spiSet);
    digitalWrite(activeBus->cs, LOW);
    //send register
    activeBus->spi->transfer(reg);
    //read data
    val = activeBus->spi->transfer(0);
    //end SPI transaction
    digitalWrite(activeBus->cs, HIGH);
    activeBus->spi->endTransaction();}
    
    return val;}
    
void burstRead(uint8_t reg, byte bytes) {

  //I2C Burst Read
  if(activeBus->type == 'I'){
    activeBus->wire->setClock(activeBus->i2cRate);
    activeBus->wire->beginTransmission(activeBus->i2cAddress);
    activeBus->wire->write(reg);
    activeBus->wire->endTransmission(false);
    activeBus->wire->requestFrom(activeBus->i2cAddress, bytes);
    while (activeBus->wire->available() < bytes) {};
    for (byte i = 0; i < bytes; i++) {rawData[i] = activeBus->wire->read();}}

  //SPI Burst Read
  else{
    
    //begin SPI transaction
    activeBus->spi->beginTransaction(activeBus->spiSet);
    digitalWriteFast(activeBus->cs, LOW);
    
    //send register
    activeBus->spi->transfer(reg);

    //read data
    for(byte i = 0; i < bytes; i++){rawData[i] = activeBus->spi->transfer(0);}
    
    //end SPI transaction
    digitalWriteFast(activeBus->cs, HIGH);
    activeBus->spi->endTransaction();}}

void write8(byte reg, uint8_t val) {

  uint8_t readVal;
  
  //I2C Write
  if(activeBus->type == 'I'){
    activeBus->wire->setClock(activeBus->i2cRate);
    activeBus->wire->beginTransmission(activeBus->i2cAddress);
    activeBus->wire->write(reg);
    activeBus->wire->write(val);
    activeBus->wire->endTransmission(false);}

  //SPI Write
  else{

    //begin SPI transaction
    activeBus->spi->beginTransaction(activeBus->spiSet);
    digitalWriteFast(activeBus->cs, LOW);
    
    //Send data
    activeBus->spi->transfer(reg);
    activeBus->spi->transfer(val);

    //End transaction
    digitalWriteFast(activeBus->cs, HIGH);
    activeBus->spi->endTransaction();
    
    //read data back
    activeBus->spi->beginTransaction(activeBus->spiSet);
    digitalWrite(activeBus->cs, LOW);
    activeBus->spi->transfer(reg | 0x80);
    readVal = activeBus->spi->transfer(0);
  
    //end SPI transaction
    digitalWrite(activeBus->cs, HIGH);
    activeBus->spi->endTransaction();

    if(readVal != val){Serial.println("Register Write Failed!");}}
    
    }

void write16(byte reg, uint16_t val) {

  //I2C write
  if(activeBus->type == 'I'){
    activeBus->wire->setClock(activeBus->i2cRate);
    activeBus->wire->beginTransmission(activeBus->i2cAddress);
    activeBus->wire->write((uint8_t)reg);
    activeBus->wire->write((uint8_t)(val >> 8));
    activeBus->wire->write((uint8_t)(val & 0xFF));
    activeBus->wire->endTransmission(false);}

  //SPI write
  else{
    
    //begin SPI transaction
    activeBus->spi->beginTransaction(activeBus->spiSet);
    digitalWrite(activeBus->cs, LOW);
    
    //Send data
    activeBus->spi->transfer(reg);
    activeBus->spi->transfer(val >> 8);
    activeBus->spi->transfer(val & 0xFF);
  
    //end SPI transaction
    digitalWrite(activeBus->cs, HIGH);
    activeBus->spi->endTransaction();}}
