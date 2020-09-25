

void preflightPacket(){
  
/*---------------------------------------------------
              PREFLIGHT PACKET
 -----------------------------------------------------*/
    
        //Read data from pre-flight packet
        pktPosn=1;
        GPSlock = (byte)dataPacket[pktPosn];pktPosn++;
        contCode = (byte)dataPacket[pktPosn];pktPosn++;
        for(byte i=0;i<20;i++){rocketName[i] = (byte)dataPacket[pktPosn];pktPosn++;}
        baseAlt = (byte)dataPacket[pktPosn];pktPosn++;
        baseAlt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        baseGPSalt = (byte)dataPacket[pktPosn];pktPosn++;
        baseGPSalt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        charGPSlat = (byte)dataPacket[pktPosn];pktPosn++;
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)dataPacket[pktPosn];pktPosn++;}
        GPSlatitude=radioUnion.GPScoord;
        charGPSlon = (byte)dataPacket[pktPosn];pktPosn++;
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)dataPacket[pktPosn];pktPosn++;}
        GPSlongitude=radioUnion.GPScoord;
        satNum = (byte)dataPacket[pktPosn];pktPosn++;
        satNum += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        if(debugSerial){Serial.println(F("Ground Packet Received"));}
        if(FHSS){
          nextChnl = dataPacket[pktPosn];pktPosn++;
          nextChnl2 = dataPacket[pktPosn];
          //set the next channel
          syncFreq = false;
          if(debugSerial){
            dispPktInfo();
            Serial.print(F("Hopping Freq: "));}
          chnlUsed = 0;
          hopFreq();}

        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock){captureGPS();}

        //display to LCD
        if(lcdON){preflightLCD();}
        }

void inflightPacket(){
        
/*---------------------------------------------------
              INFLIGHT PACKET
 -----------------------------------------------------*/     

        //write the last pre-flight packet to the SD card
        if (preFlightWrite && SDon){writePreflightData();}

        //parse the GPS data and packet number first, then the samples
        pktPosn = samples * 13;
        packetnum = (byte)dataPacket[pktPosn];pktPosn++;//53
        packetnum += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//54
        if(FHSS){
          nextChnl = (byte)dataPacket[pktPosn];pktPosn++;
          nextChnl2 = (byte)dataPacket[pktPosn];pktPosn++;}
        GPSalt = (byte)dataPacket[pktPosn];pktPosn++;//55
        GPSalt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//56
        for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)dataPacket[pktPosn];pktPosn++;}//60
        GPSlatitude=radioUnion.GPScoord;
        for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)dataPacket[pktPosn];pktPosn++;}//64
        GPSlongitude=radioUnion.GPScoord;
        
        //determine GPS lock from the packet length
        if(len < samples * 13 + 12){GPSlock = 0;}
        else{GPSlock = 1; lastGPSfix = micros();}
        
        //parse inflight packet of 4 samples
        pktPosn = 0;
        for(byte i=0;i<samples;i++){
                      
          //parse the samples
          event = (byte)dataPacket[pktPosn];pktPosn++;//1
          if(!apogee && event >=4 && event <=6){apogee = true;}
          sampleTime = (byte)dataPacket[pktPosn];pktPosn++;//2
          sampleTime += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//3
          velocity = (byte)dataPacket[pktPosn];pktPosn++;//4
          velocity += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//5
          if(!apogee && velocity > maxVelocity){maxVelocity = velocity;}
          Alt = (byte)dataPacket[pktPosn];pktPosn++;//6
          Alt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//7
          if(!apogee && Alt > maxAltitude){maxAltitude = Alt;}
          spin = (byte)dataPacket[pktPosn];pktPosn++;//8
          spin += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//9
          offVert = (byte)dataPacket[pktPosn];pktPosn++;//10
          offVert += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//11
          accel = (byte)dataPacket[pktPosn];pktPosn++;//12
          accel += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//13
          if(!apogee && accel > maxG){maxG = accel;}
          //write to the SD card
          if(SDon){writeInflightData();}
          }

        //sync the SD card
        if(SDon){myFile.sync();}

        if(debugSerial){Serial.print(F("Inflight Packet Received, PktNum: "));Serial.println(packetnum);}
        //set the next channel
        if(FHSS){
          syncFreq = false;
          if(debugSerial){dispPktInfo();}
          lastHopTime = lastRX - (packetnum%3 *200000UL);
          chnlUsed = 0;
          if(packetnum%3 == 0){
            if(debugSerial){Serial.print(F("Hopping Freq: "));}
            hopFreq();}}

        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock == 1){captureGPS();}
          
        //display to LCD
        if(lcdON){inflightLCD();}
        }

void postflightPacket(){
          
/*---------------------------------------------------
              POSTFLIGHT PACKET
 -----------------------------------------------------*/
        pktPosn = 1;
        maxAltitude = (byte)dataPacket[pktPosn];pktPosn++;
        maxAltitude += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        maxVelocity = (byte)dataPacket[pktPosn];pktPosn++;
        maxVelocity += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        maxG = (byte)dataPacket[pktPosn];pktPosn++;
        maxG += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        maxGPSalt = (byte)dataPacket[pktPosn];pktPosn++;
        maxGPSalt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        GPSlock = (byte)dataPacket[pktPosn];pktPosn++;
        GPSalt = (byte)dataPacket[pktPosn];pktPosn++;
        GPSalt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;
        charGPSlat = (byte)dataPacket[pktPosn];pktPosn++;
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)dataPacket[pktPosn];pktPosn++;}
        GPSlatitude=radioUnion.GPScoord;
        charGPSlon = (byte)dataPacket[pktPosn];pktPosn++;
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)dataPacket[pktPosn];pktPosn++;}
        GPSlongitude=radioUnion.GPScoord;

        //capture the last good GPS coordinates to potentially store later in EEPROM
        captureGPS();

        if(FHSS){
          syncFreq = false;
          if(debugSerial){
            Serial.print(F("Post-flight Packet Received, Chnl: "));Serial.println(chnl);
            dispPktInfo();}}
        
        //write to the SD card
        if(postFlightWrite && SDon){writePostflightData();}

        //display to LCD
        if(lcdON){postflightLCD();}
        }

void hopFreq(){

  byte hopChnl;

  chnlUsed++;
  if(chnlUsed == 1){hopChnl = nextChnl;if(debugSerial){Serial.println(hopChnl);}}
  else if(chnlUsed == 2){hopChnl = nextChnl2;if(debugSerial){Serial.println(hopChnl);}}
  else if(chnlUsed >= 3){hopChnl = hailChnl;if(debugSerial){Serial.print(F("Moving to sync chnl: ")); Serial.println(hopChnl); syncFreq = true;}}
  chnl = hopChnl;
  rf95.setModeIdle();
  rf95.setFrequency(getFreq(chnl));
  lastHopTime = micros();
}

void syncPkt(){

  byte currentChnl = chnl;
  syncFreq = false;
  //parse packet
  chnl = (byte)dataPacket[1];
  nextChnl = (byte)dataPacket[2];
  nextChnl2= (byte)dataPacket[3];
  chnlUsed = 0;
  hopFreq();
  if(debugSerial){
    Serial.print("------ Sync Packet Recieved ------");Serial.println(currentChnl);Serial.print("------ ");
    dispPktInfo();}}
      
void captureGPS(){
  //capture the GPS data as the last good position
  if(GPSlock == 1){
    lastGPSlat = GPSlatitude;
    lastGPSlon = GPSlongitude;}}

void dispPktInfo(){
  freq = getFreq(chnl);
  if(debugSerial){
    Serial.print(F("Current Chnl: "));Serial.print(chnl);
    Serial.print(F(", "));Serial.print(freq);Serial.println(F("MHz"));
    Serial.print(F("Next Chnl: "));Serial.print(nextChnl);Serial.print(F(", Next Freq: "));}
  freq = getFreq(nextChnl);
  if(debugSerial){Serial.print(freq, 3); Serial.println("MHz");}
}

float getFreq(byte chnl){
  float frq = 902.300F + 0.200F * chnl;
  return frq;}
