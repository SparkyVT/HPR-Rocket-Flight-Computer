void hopFreq(){

  byte hopChnl;

  chnlUsed++;
  if(chnlUsed == 1){hopChnl = nextChnl;if(debugSerial){Serial.println(hopChnl);}}
  else if(chnlUsed == 2){hopChnl = nextChnl2;if(debugSerial){Serial.println(hopChnl);}}
  else if(chnlUsed >= 3){hopChnl = hailChnl_1;if(debugSerial){Serial.print(F("Moving to sync chnl: ")); Serial.println(hopChnl); syncFreq = true;}}
  chnl1 = hopChnl;
  radio1.setModeIdle();
  radio1.setFrequency(getFreq(chnl1));
  lastHopTime = micros();
}

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
  
  else{float frq = 902.300F + 0.200F * chnl;}
  
  return frq;}
