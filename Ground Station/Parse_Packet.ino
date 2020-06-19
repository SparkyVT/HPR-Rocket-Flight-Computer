
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

        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock){captureGPS();}

        //display to LCD
        preflightLCD();}

void inflightPacket(){
        
/*---------------------------------------------------
              INFLIGHT PACKET
 -----------------------------------------------------*/     

        //write the last pre-flight packet to the SD card
        if (preFlightWrite){writePreflightData();}

        //parse the GPS data and packet number first, then the samples
        pktPosn = samples * 13;
        packetnum = (byte)dataPacket[pktPosn];pktPosn++;//53
        packetnum += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//54
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
          accelX = (byte)dataPacket[pktPosn];pktPosn++;//12
          accelX += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//13
          if(!apogee && accelX > maxG){maxG = accelX;}
          //write to the SD card
          writeInflightData();}

        //sync the SD card
        myFile.sync();

        //capture the last good GPS coordinates to potentially store later in EEPROM
        if(GPSlock == 1){captureGPS();}
          
        //display to LCD
        inflightLCD();}

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
          
        //write to the SD card
        if(postFlightWrite){writePostflightData();}

        //display to LCD
        postflightLCD();}

void captureGPS(){
  //capture the GPS data as the last good position
  if(GPSlock == 1){
    lastGPSlat = GPSlatitude;
    lastGPSlon = GPSlongitude;}}
