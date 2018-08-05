
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
        if (preFlightWrite && GPSlock == 1){writePreflightData();}

        //display to LCD
        preflightLCD();}

void inflightPacket(){
        
/*---------------------------------------------------
              INFLIGHT PACKET
 -----------------------------------------------------*/     
        prevAlt = baroAlt;
        prevTime=currentTime;
      
        //parse inflight packet of 3 samples
        pktPosn = 0;
        for(byte i=0;i<4;i++){
          event = (byte)dataPacket[pktPosn];pktPosn++;//1
          for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)dataPacket[pktPosn];pktPosn++;}//5
          currentTime = radioUnion.radioTime;
          accelVel = (byte)dataPacket[pktPosn];pktPosn++;//6
          accelVel += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//7
          accelAlt = (byte)dataPacket[pktPosn];pktPosn++;//8
          accelAlt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//9
          baroAlt = (byte)dataPacket[pktPosn];pktPosn++;//10
          baroAlt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//11
          angX = (byte)dataPacket[pktPosn];pktPosn++;//12
          angX += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//13
          angY = (byte)dataPacket[pktPosn];pktPosn++;//14
          angY += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//15
          angZ = (byte)dataPacket[pktPosn];pktPosn++;//16
          angZ += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//17
          accelX = (byte)dataPacket[pktPosn];pktPosn++;//18
          accelX += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//19
          if(i == 0){
            GPSalt = (byte)dataPacket[pktPosn];pktPosn++;//20
            GPSalt += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//21
            charGPSlat = (byte)dataPacket[pktPosn];pktPosn++;//22
            for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)dataPacket[pktPosn];pktPosn++;}//26
            GPSlatitude=radioUnion.GPScoord;
            charGPSlon = (byte)dataPacket[pktPosn];pktPosn++;//27
            for(byte j=0;j<4;j++){radioUnion.unionByte[j]=(byte)dataPacket[pktPosn];pktPosn++;}//31
            GPSlongitude=radioUnion.GPScoord;
            packetnum = (byte)dataPacket[pktPosn];pktPosn++;//32
            packetnum += ((byte)dataPacket[pktPosn] << 8);pktPosn++;//33

            //capture the last good GPS coordinates to potentially store later in EEPROM
            captureGPS();}

          //write to the SD card
          writeInflightData();}

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
        if(postFlightWrite && GPSlock == 1){writePostflightData();}

        //display to LCD
        postflightLCD();}

void captureGPS(){
  //capture the GPS data as the last good position
  if(GPSlock == 1 && charGPSlat != '\0'){
    lastCharGPSlat = charGPSlat;
    lastGPSlat = GPSlatitude;
    lastCharGPSlon = charGPSlon;
    lastGPSlon = GPSlongitude;}}
