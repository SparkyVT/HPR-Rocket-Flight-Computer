/*900MHz Strategy

preflight: 
- send packet every 600ms
- sync packet sent on common hailing freq once per 1200ms
-- use one common channel, send sync packet after every other data packet
-- data in sync packet transmits the channel for the data packet
- data packet sent on the frequency from the hailing packet

inflight:
- shift frequencies every 600ms
- 3 packets sent then shift
- use MCU timer functions to send packets
-- FC uses timer functions to send packets and change channels if necessary
-- GS uses timer functions to change channels if not done automatically
-- if the packet is going to be transmitted during a change, then the FC will change frequencies after the packet is sent
-- the GS will change frequencies based on a time after the last packet is received.  An interval timer may be used.

postflight:
- sync packet sent on common hailing freq once every 10 seconds
-- data in sync packet transmits the channel for the data packet
- data packet sent on the frequency from the hailing packet
- sync packet stays on until system is turned off*/

//SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

// A_new = A_old + V_old * dt * cos(OffVert) + .5 * A_old * dt^2

void radioSendPacket(){
RH_RF95 rf95(pins.radioCS, pins.radioIRQ);

//------------------------------------------------------------------
//                  PRE-FLIGHT PACKET
//------------------------------------------------------------------
  //send the preflight packet, 39 bytes
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
    //even thought the packet is shorter, sending 64 bytes is needed to activate the Ublox counter-interference software for in-flight data capture
    //rf95.send((uint8_t *)dataPacket, 64);
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
      
    //GPS Data collected once per packet then transmit
    if(sampNum >= packetSamples){
      radio.packetnum++;
  
      //GPS Altitude
      dataPacket[pktPosn] = lowByte(radio.packetnum); pktPosn++;//53
      dataPacket[pktPosn] = highByte(radio.packetnum); pktPosn++;//54
      if(gpsTransmit){
        gpsTransmit=false;
        dataPacket[pktPosn] = lowByte(radio.GPSalt);pktPosn++;//55
        dataPacket[pktPosn] = highByte(radio.GPSalt);pktPosn++;//56
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlatitude.GPSbyte[i];pktPosn++;}//60
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlongitude.GPSbyte[i];pktPosn++;}}//64
      //send packet
      rf95.send((uint8_t *)dataPacket, pktPosn);
      //reset counting variables
      if(sensors.status_ADXL377){TXnow = micros();}
      sampNum = 0;
      pktPosn = 0;
      radioTX = true;}}
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
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlongitude.GPSbyte[i]; pktPosn++;}//17 bytes
      dataPacket[pktPosn]=gpsLon;pktPosn++;//18 bytes
      for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlatitude.GPSbyte[i];pktPosn++;}//22 bytes
      rf95.send((uint8_t *)dataPacket, pktPosn);
    }//end telemetry ground code
  
}//end radioSendPacket

void setChannel(byte channel){
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
//update counters for 915MHz radios
      if(sensors.radio == 2){
        TXnum++;
        if(TXnum == 3){
          freqNum = (freqNum >= 63) ? 0 : freqNum++;
          float freq = 902.300 + 0.2 * chnl[freqNum];
          rf95.setFrequency(freq);}
          TXnum = 0;}
  
}
