void preflightPacket(byte rxPacket[]){
  
/*---------------------------------------------------
              PREFLIGHT PACKET
 -----------------------------------------------------*/
    
        //Read data from pre-flight packet
        pktPosn=eventPosn;
        event = (byte)rxPacket[pktPosn];pktPosn++;//1
        GPSlock = (byte)rxPacket[pktPosn];pktPosn++;//2
        contCode = (byte)rxPacket[pktPosn];pktPosn++;//3
        for(byte i=0;i<20;i++){rocketName[i] = (byte)rxPacket[pktPosn];pktPosn++;}//23
        radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//24
        radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//25
        baseAlt = radioInt.unionInt;
        radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//26
        radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//27
        baseGPSalt = radioInt.unionInt;
        charGPSlat = (int8_t)rxPacket[pktPosn];pktPosn++;//28
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)rxPacket[pktPosn];pktPosn++;}//32
        GPSlatitude=radioUnion.GPScoord;
        charGPSlon = (byte)rxPacket[pktPosn];pktPosn++;//33
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)rxPacket[pktPosn];pktPosn++;}//37
        GPSlongitude=radioUnion.GPScoord;
        radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//38
        radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//39
        satNum = radioInt.unionInt;
        if(activeRadio->FHSS){
          activeRadio->nextChnl = (byte)rxPacket[pktPosn];pktPosn++;
          activeRadio->nextChnl2 = (byte)rxPacket[pktPosn];
          //set the next channel
          activeRadio->syncFreq = false;
          if(settings.debugSerial){
            dispPktInfo();
            Serial.print(F("Hopping Freq: "));}
          activeRadio->chnlUsed = 0;
          hopFreq();}
        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock == 1){
          lastGPSlat = GPSlatitude;
          lastGPSlon = GPSlongitude;}}

void inflightPacket(byte rxPacket[]){
        
/*---------------------------------------------------
              INFLIGHT PACKET
 -----------------------------------------------------*/     
        
        //parse the GPS data and packet number first, then the samples
        pktPosn = 52 + eventPosn;
        radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//53
        radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//54
        packetnum = radioInt.unionInt;

        if(debugSerial){Serial.print(F("Inflight Packet Received, PktNum: "));Serial.println(packetnum);}
        
        if(activeRadio->FHSS){
          activeRadio->nextChnl = (byte)rxPacket[pktPosn];pktPosn++;
          activeRadio->nextChnl2 = (byte)rxPacket[pktPosn];pktPosn++;}
        radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//55
        radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//56
        GPSalt = radioInt.unionInt;
        for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)rxPacket[pktPosn];pktPosn++;}//60
        GPSlatitude=radioUnion.GPScoord;
        for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)rxPacket[pktPosn];pktPosn++;}//64
        GPSlongitude=radioUnion.GPScoord;
      
        //determine GPS lock from the packet length
        if(len < 60+eventPosn){GPSlock = 0;}
        else{GPSlock = 1; lastGPSfix = micros();}
        
        //parse inflight packet of 4 samples
        pktPosn = eventPosn;
        for(byte i=0;i<4;i++){
                      
          //parse the samples
          event = (byte)rxPacket[pktPosn];pktPosn++;//1
          if(!apogee && event >=4 && event <=6){apogee = true;}
          sampleTime = (byte)rxPacket[pktPosn];pktPosn++;//2
          sampleTime += ((byte)rxPacket[pktPosn] << 8);pktPosn++;//3
          radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//4
          radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//5
          velocity = radioInt.unionInt;
          if(!apogee && velocity > maxVelocity){maxVelocity = velocity;}
          radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//6
          radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//7
          Alt = radioInt.unionInt;
          if(!apogee && Alt > maxAltitude){maxAltitude = Alt;}
          radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//8
          radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//9
          spin = radioInt.unionInt;
          radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//10
          radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//11
          offVert = radioInt.unionInt;
          radioInt.unionByte[0] = (byte)rxPacket[pktPosn];pktPosn++;//12
          radioInt.unionByte[1] = (byte)rxPacket[pktPosn];pktPosn++;//13
          accel = radioInt.unionInt;
          if(!apogee && accel > maxG){maxG = accel;}
          
          //write to the SD card
          if(SDinit){writeInflightData();}}

        if(settings.debugSerial){Serial.println("Inflight Data Written");}
        //set the next channel
        if(activeRadio->FHSS){
          activeRadio->syncFreq = false;
          if(debugSerial){dispPktInfo();}
          activeRadio->lastHopTime = activeRadio->lastRX - (packetnum%3 *200000UL);
          activeRadio->chnlUsed = 0;
          if(packetnum%3 == 0){
            if(settings.debugSerial){Serial.print(F("Hopping Freq: "));}
            hopFreq();}}

        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock == 1){
          lastGPSlat = GPSlatitude;
          lastGPSlon = GPSlongitude;}}

void postflightPacket(byte rxPacket[]){
          
/*---------------------------------------------------
              POSTFLIGHT PACKET
 -----------------------------------------------------*/
        pktPosn = eventPosn+1;
        maxAltitude = (byte)rxPacket[pktPosn];pktPosn++;
        maxAltitude += ((byte)rxPacket[pktPosn] << 8);pktPosn++;
        maxVelocity = (byte)rxPacket[pktPosn];pktPosn++;
        maxVelocity += ((byte)rxPacket[pktPosn] << 8);pktPosn++;
        maxG = (byte)rxPacket[pktPosn];pktPosn++;
        maxG += ((byte)rxPacket[pktPosn] << 8);pktPosn++;
        maxGPSalt = (byte)rxPacket[pktPosn];pktPosn++;
        maxGPSalt += ((byte)rxPacket[pktPosn] << 8);pktPosn++;
        GPSlock = (byte)rxPacket[pktPosn];pktPosn++;
        GPSalt = (byte)rxPacket[pktPosn];pktPosn++;
        GPSalt += ((byte)rxPacket[pktPosn] << 8);pktPosn++;
        charGPSlat = (byte)rxPacket[pktPosn];pktPosn++;
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)rxPacket[pktPosn];pktPosn++;}
        GPSlatitude=radioUnion.GPScoord;
        charGPSlon = (byte)rxPacket[pktPosn];pktPosn++;
        for(byte i=0;i<4;i++){radioUnion.unionByte[i]=(byte)rxPacket[pktPosn];pktPosn++;}
        GPSlongitude=radioUnion.GPScoord;

        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock == 1){
          lastGPSlat = GPSlatitude;
          lastGPSlon = GPSlongitude;}
          
        //write to the SD card
        if(SDinit && parseSustainer && sustainerPostFlightWrite){writePostflightData();}
        if(SDinit && parseBooster && boosterPostFlightWrite){writePostflightData();}

        //sync the SD card
        if(SDinit && parseSustainer){sustainerFile.flush();}
        if(SDinit && parseBooster){boosterFile.flush();}
}
