
void radioSendPacket(){
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
  static byte gndPkt = 0;
  unsigned long TXlength;
  unsigned long preflightLength = 77060UL;
  bool TX;
  
//------------------------------------------------------------------
//                  PRE-FLIGHT PACKET
//------------------------------------------------------------------
  //send the preflight packet, 39 bytes, or 41 bytes if 915MHz FHSS
  if(events.preLiftoff){
    
    pktPosn=0;
    dataPacket[pktPosn]=radio.event; pktPosn++;
    dataPacket[pktPosn]=gpsFix; pktPosn++;
    dataPacket[pktPosn]=cont.reportCode; pktPosn++;
    for (byte j = 0; j < sizeof(settings.rocketName); j++){
      dataPacket[pktPosn] = settings.rocketName[j];
      pktPosn++;}
    dataPacket[pktPosn]=lowByte(radio.baseAlt); pktPosn++;
    dataPacket[pktPosn]=highByte(radio.baseAlt); pktPosn++;
    dataPacket[pktPosn]=lowByte(radio.GPSalt); pktPosn++;
    dataPacket[pktPosn]=highByte(radio.GPSalt); pktPosn++;
    dataPacket[pktPosn]=gpsLat; pktPosn++;
    for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlatitude.GPSbyte[i]; pktPosn++;}
    dataPacket[pktPosn]=gpsLon; pktPosn++;
    for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlongitude.GPSbyte[i]; pktPosn++;}
    dataPacket[pktPosn]=lowByte(radio.satNum); pktPosn++;
    dataPacket[pktPosn]=highByte(radio.satNum); pktPosn++;
    rf95.send((uint8_t *)dataPacket, pktPosn);
    pktPosn = 0;}
    
//------------------------------------------------------------------
//                  IN-FLIGHT PACKET
//------------------------------------------------------------------
  //send inflight packet, 64 bytes
  //build the packet of 4 samples: 13 bytes per sample, 12 bytes GPS & pktnum, 13 x 4 + 12 = 64 bytes flight data
  else if(events.inFlight){  
    
    //update sample number
    sampNum++;
    //event
    dataPacket[pktPosn] = radio.event; pktPosn++;//1
    //time
    radio.fltTime = (uint16_t)(fltTime.timeCurrent/10000);
    dataPacket[pktPosn] = lowByte(radio.fltTime);pktPosn++;//2
    dataPacket[pktPosn] = highByte(radio.fltTime);pktPosn++;//3
    //velocity
    dataPacket[pktPosn] = lowByte(radio.vel);pktPosn++;//4
    dataPacket[pktPosn] = highByte(radio.vel);pktPosn++;//5
    //altitude
    radio.alt = (int16_t)Alt;
    dataPacket[pktPosn] = lowByte(radio.alt);pktPosn++;//6
    dataPacket[pktPosn] = highByte(radio.alt);pktPosn++;//7
    //Roll data
    radio.roll = rollZ;
    dataPacket[pktPosn] = lowByte(radio.roll);pktPosn++;//8
    dataPacket[pktPosn] = highByte(radio.roll);pktPosn++;//9
    //Off Vertical data
    radio.offVert = offVert;
    dataPacket[pktPosn] = lowByte(radio.offVert);pktPosn++;//10
    dataPacket[pktPosn] = highByte(radio.offVert);pktPosn++;//11
    //Acceleration
    radio.accel = (int16_t)(accelNow * 33.41406087); //33.41406087 = 32768 / 9.80665 / 100
    dataPacket[pktPosn] = lowByte(radio.accel);pktPosn++;//12
    dataPacket[pktPosn] = highByte(radio.accel);pktPosn++;//13
      
    //GPS & packet data collected once per packet
    if(sampNum >= packetSamples){

      //update packet number
      radio.packetnum++;
      dataPacket[pktPosn] = lowByte(radio.packetnum); pktPosn++;//53
      dataPacket[pktPosn] = highByte(radio.packetnum); pktPosn++;//54
      
      //GPS Data
      if(gpsTransmit){
        gpsTransmit=false;
        dataPacket[pktPosn] = lowByte(radio.GPSalt);pktPosn++;//55
        dataPacket[pktPosn] = highByte(radio.GPSalt);pktPosn++;//56
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlatitude.GPSbyte[i];pktPosn++;}//60
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlongitude.GPSbyte[i];pktPosn++;}}//64
        
    //send packet
    rf95.send((uint8_t *)dataPacket, pktPosn);
    radioTX = true;
    //reset counting variables
    sampNum = 0;
    pktPosn = 0;}}
    
//------------------------------------------------------------------
//                  POST-FLIGHT PACKET
//------------------------------------------------------------------
  //send post flight packet, 22 bytes
  else if(events.postFlight){

      pktPosn=0;
      dataPacket[pktPosn]=radio.event; pktPosn++;//1 byte
      dataPacket[pktPosn]=lowByte(radio.maxAlt); pktPosn++;//2 bytes
      dataPacket[pktPosn]=highByte(radio.maxAlt); pktPosn++;//3 bytes
      dataPacket[pktPosn]=lowByte(radio.maxVel); pktPosn++;//4 bytes
      dataPacket[pktPosn]=highByte(radio.maxVel); pktPosn++;//5 bytes
      dataPacket[pktPosn]=lowByte(radio.maxG); pktPosn++;//6 bytes
      dataPacket[pktPosn]=highByte(radio.maxG); pktPosn++;//7 bytes
      dataPacket[pktPosn]=lowByte(radio.maxGPSalt); pktPosn++;//8 bytes
      dataPacket[pktPosn]=highByte(radio.maxGPSalt); pktPosn++;//9 bytes
      dataPacket[pktPosn]=gpsFix; pktPosn++;//10 bytes
      dataPacket[pktPosn]=lowByte(radio.GPSalt); pktPosn++;//11 bytes
      dataPacket[pktPosn]=highByte(radio.GPSalt); pktPosn++;//12 bytes
      dataPacket[pktPosn]=gpsLat; pktPosn++;//13 bytes
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlatitude.GPSbyte[i]; pktPosn++;}//17 bytes
      dataPacket[pktPosn]=gpsLon;pktPosn++;//18 bytes
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlongitude.GPSbyte[i];pktPosn++;}//22 bytes
      TX = rf95.send((uint8_t *)dataPacket, pktPosn);
      }//end postFlight code

}//end radioSendPacket
