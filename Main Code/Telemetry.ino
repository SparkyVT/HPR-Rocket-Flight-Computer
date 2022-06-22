//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//radioSendPacket(): main routine to build and send telemetry packets
//hopTXfreq(): hops frequency when FHSS is active
//syncPkt(): sends a sync packet on the FHSS hailing freq
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//---------------------------------

/*900MHz FHSS Strategy

preflight: 
- send packet every 600ms
- sync packet sent on common hailing freq once per 1800ms
-- use one common channel, send sync packet after every 3rd data packet
-- data in sync packet transmits the channel for the next data packet and the sequence number
- data packet sent on the frequency from the hailing packet

inflight:
- shift frequencies every 600ms
- Flight computer:
-- Force a sync packet on the current channel at liftoff
-- send 3 packets on one freq, sent then shift freq
-- every third shift (once per 1.8s) send sync packet on hailing frequency
- Ground Station
-- Shift frequency after 3rd consecutive packet
-- If 600ms passes and no packet is recieved, shift frequency anyway
-- If 1.2s passes and no packet is recieved, switch to hailing frequency

postflight:
- sync packet sent on common hailing freq once every 10 seconds
-- data in sync packet transmits the channel for the data packet
- data packet sent on the frequency from the hailing packet
- sync packet stays on until system is turned off*/

//This is a pseudo-random sequence of channels for the inflight packets that will stay within the FCC regulations 
const int8_t hopSequence[2000] = {
  49,   41,    4,  31,   56,   21,   16,   58,    7,   50,   39,    9,   24,   35,   54,   52,   36,   34,   11,   
  19,   23,   38,  27,    5,   28,    6,   40,   32,   17,   30,   20,   10,   43,   51,   12,   22,   63,   26,   
  57,   49,   29,  45,   61,   21,   42,    2,   47,   55,   56,   46,   48,    4,   18,   60,   11,   16,   50,   
   0,   24,   15,  38,   35,   27,   54,    1,   62,   44,   25,   31,   41,   12,   13,   37,   53,   20,    7,  
   33,   58,   51,   30,   8,  36,   6,  45,   49,   34,   29,   52,   26,   48,   18,   40,   55,   47,   60,   
   21,   9,  50,   17,   15,   23,   61,   44,   35,   22,   0,  12,   24,   4,  19,   25,   63,   2,  31,   38,   
   14,   56,   54,   37,   42,   58,   28,   7,  8,  46,   57,   20,   10,   40,   26,   52,   53,   51,   49,   
   55,   29,   59,   30,   34,   32,   17,   23,   11,   47,   33,   36,   4,  41,   61,   31,   18,   14,   15,   
   38,   25,   50,   9,  2,  16,   8,  37,   0,  60,   27,   19,   39,   58,   35,   1,  51,   10,   5,  45,   54,   
   57,   52,   22,   20,   48,   40,   62,   24,   13,   26,   53,   63,   29,   61,   49,   30,   7,  55,   33,   
   3,  4,  16,   32,   38,   15,   36,   41,   50,   56,   25,   27,   8,  42,   47,   6,  1,  12,   39,   23,   
   45,   21,   35,   43,   44,   52,   26,   28,   9,  20,   58,   22,   18,   57,   11,   46,   3,  53,   13,   
   7,  60,   34,   37,   38,   14,   17,   59,   10,   29,   5,  32,   54,   40,   6,  15,   16,   48,   42,   45,
   62,   31,   2,  21,   47,   0,  8,  25,   44,   19,   50,   30,   24,   20,   26,   35,   51,   61,   27,   49,
   53,   37,   41,   34,   43,   11,   60,   32,   13,   10,   39,   6,  52,   12,   38,   16,   55,   7,  29,   
   21,   15,   1,  4,  42,   22,   48,   2,  56,   31,   57,   44,   36,   26,   8,  45,   58,   5,  50,   20,   
   34,   61,   33,   24,   27,   46,   18,   59,   25,   32,   60,   0,  40,   19,   35,   30,   6,  62,   63,   
   55,   14,   9,  51,   49,   47,   53,   12,   7,  28,   57,   38,   36,   56,   11,   17,   52,   39,   61,   
   4,  42,   41,   31,   15,   44,   24,   20,   33,   23,   16,   46,   21,   29,   34,   8,  26,   37,   22,   
   45,   1,  18,   54,   62,   43,   49,   12,   59,   19,   28,   35,   53,   58,   11,   27,   2,  3,  7,  13,   
   47,   55,   50,   31,   14,   60,   52,   42,   23,   41,   25,   30,   36,   51,   5,  39,   44,   33,   18,   
   8,  0,  38,   40,   32,   6,  16,   29,   49,   53,   34,   20,   56,   1,  63,   62,   35,   48,   26,   9,  
   10,   61,   22,   12,   47,   15,   23,   4,  11,   3,  28,   21,   45,   59,   44,   37,   25,   50,   41,   2,  
   46,   36,   52,   39,   51,   27,   58,   17,   60,   43,   55,   32,   14,   53,   54,   8,  48,   33,   7,  
   35,   56,   19,   20,   30,   23,   38,   57,   21,   63,   9,  15,   11,   28,   44,   0,  25,   62,   5,  22,   
   2,  45,   50,   18,   31,   42,   10,   52,   51,   32,   36,   54,   13,   47,   41,   53,   48,   24,   43,   
   39,   14,   7,  1,  57,   26,   58,   20,   35,   40,   60,   8,  4,  44,   15,   9,  3,  6,  37,   49,   30,   
   56,   18,   0,  12,   42,   16,   55,   27,   38,   62,   25,   41,   28,   2,  24,   48,   43,   14,   36,   45,   
   11,   23,   53,   46,   51,   5,  63,   58,   47,   34,   35,   32,   8,  19,   44,   54,   37,   15,   49,   17,   
   59,   29,   40,   13,   21,   22,   52,   18,   10,   7,  61,   55,   31,   3,  39,   12,   16,   25,   14,   62,   
   41,   48,   50,   28,   4,  11,   38,   43,   0,  9,  30,   33,   19,   27,   56,   20,   53,   54,   45,   57,   
   59,   37,   8,  36,   22,   42,   21,   55,   29,   2,  23,   47,   51,   26,   63,   24,   15,   49,   18,   3,  
   41,   58,   48,   25,   14,   60,   30,   40,   19,   6,  46,   10,   11,   9,  43,   54,   35,   28,   1,  44,   
   39,   27,   38,   20,   34,   12,   36,   50,   42,   2,  61,   53,   26,   23,   5,  13,   22,   57,   21,   45,   
   52,   0,  30,   31,   40,   47,   17,   41,   60,   9,  3,  14,   35,   18,   11,   59,   49,   32,   63,   8,  54,   
   24,   27,   33,   37,   56,   15,   53,   62,   28,   43,   12,   13,   51,   38,   44,   61,   58,   4,  31,   52,   
   42,   22,   29,   6,  50,   1,  23,   16,   46,   36,   25,   21,   48,   2,  32,   60,   10,   3,  55,   34,   57,   
   24,   26,   30,   35,   11,   41,   8,  53,   7,  20,   63,   5,  28,   61,   19,   15,   9,  13,   38,   56,   37,   
   6,  40,   17,   44,   29,   50,   62,   21,   36,   2,  10,   51,   1,  52,   23,   47,   27,   32,   24,   39,   
   49,   26,   41,   12,   20,   55,   4,  30,   60,   28,   22,   19,   45,   53,   9,  3,  8,  5,  15,   44,   17,   
   63,   46,   18,   37,   56,   0,  43,   21,   50,   48,   2,  40,   47,   13,   10,   27,   62,   51,   11,   39,  
   61,   6,  29,   16,   31,   49,   55,   45,   34,   32,   52,   12,   3,  14,   35,   20,   9,  1,  24,   23,   63,   
   58,   0,  42,   28,   38,   48,   53,   50,   33,   36,   7,  5,  62,   4,  54,   13,   41,   57,   44,   25,   6,  
   60,   10,   26,   45,   17,   18,   55,   46,   8,  2,  59,   30,   11,   47,   9,  1,  27,   56,   14,   40,   3,  
   29,   38,   20,   0,  61,   43,   52,   42,   50,   39,   23,   41,   54,   4,  5,  48,   63,   32,   16,   31,   
   13,   15,   12,   55,   34,   17,   35,   8,  49,   47,   9,  45,   56,   6,  26,   22,   62,   36,   10,   53,   
   33,   18,   30,   60,   20,   0,  59,   19,   29,   43,   25,   52,   54,   27,   37,   28,   2,  24,   39,   32,   
   55,   4,  3,  46,   50,   31,   38,   14,   45,   51,   57,   61,   47,   5,  49,   21,   33,   6,  16,   7,  12,   
   56,   23,   19,   18,   63,   40,   30,   59,   44,   9,  15,   20,   34,   48,   36,   17,   26,   27,   41,   46,   
   37,   0,  54,   52,   55,   32,   24,   2,  22,   43,   28,   47,   33,   58,   39,   6,  25,   23,   21,   19,   
   35,   38,   30,   53,   45,   57,   44,   50,   40,   1,  59,   14,   7,  5,  26,   16,   3,  20,   42,   36,   9,  
   12,   17,   37,   15,   61,   4,  54,   34,   22,   39,   11,   46,   52,   48,   31,   55,   43,   30,   29,   19,   
   23,   45,   47,   21,   49,   38,   57,   40,   10,   41,   1,  8,  51,   24,   33,   63,   7,  6,  2,  25,   32,   
   17,   26,   58,   14,   39,   20,   56,   42,   22,   54,   48,   5,  61,   43,   37,   16,   53,   18,   12,   35,   
   60,   57,   19,   40,   11,   34,   27,   52,   13,   1,  47,   30,   10,   23,   29,   38,   24,   8,  15,   32,   
   55,   26,   39,   17,   25,   44,   31,   49,   62,   0,  43,   20,   42,   54,   53,   6,  61,   14,   56,   3,  
   18,   22,   45,   46,   58,   21,   16,   60,   47,   30,   51,   4,  24,   12,   11,   7,  2,  9,  1,  19,   55,   
   37,   57,   28,   31,   41,   34,   5,  50,   26,   63,   52,   44,   17,   29,   49,   10,   39,   25,   36,   15,   
   13,   27,   43,   6,  35,   32,   48,   14,   42,   45,   21,   2,  18,   24,   3,  30,   62,   59,   1,  9,  22,   
   11,   19,   5,  34,   58,   28,   47,   17,   51,   38,   56,   16,   23,   37,   54,   33,   63,   31,   39,   25,   
   49,   7,  43,   44,   13,   29,   14,   53,   32,   2,  50,   26,   24,   15,   20,   59,   8,  19,   3,  9,  35,   
   55,   5,  40,   12,   21,   46,   27,   42,   41,   16,   52,   23,   31,   38,   22,   36,   54,   10,   60,   45,   
   43,   0,  39,   13,   14,   48,   6,  33,   63,   32,   24,   44,   49,   8,  3,  34,   25,   9,  26,   4,  5,  35,   
   61,   12,   1,  7,  51,   37,   53,   2,  20,   62,   15,   31,   22,   42,   52,   45,   58,   13,   21,   60,   17,   
   48,   29,   54,   59,   57,   39,   63,   10,   33,   23,   9,  55,   47,   3,  27,   61,   26,   34,   44,   25,   
   18,   37,   36,   11,   6,  32,   31,   46,   49,   42,   62,   30,   12,   50,   15,   45,   19,   1,  52,   41,   
   53,   7,  56,   14,   57,   39,   8,  2,  10,   5,  51,   47,   58,   9,  54,   16,   23,   44,   43,   48,   27,   
   40,   38,   0,  21,   17,   63,   62,   20,   35,   11,   34,   50,   61,   55,   33,   24,   12,   6,  19,   26,   
   45,   36,   30,   22,   31,   1,  46,   13,   51,   29,   41,   14,   9,  3,  48,   5,  10,   49,   39,   0,  32,   
   2,  37,   28,   43,   54,   34,   44,   59,   62,   55,   61,   18,   23,   58,   15,   53,   52,   8,  27,   26,   
   38,   7,  20,   11,   45,   1,  6,  31,   22,   19,   5,  47,   4,  3,  10,   14,   42,   2,  33,   24,   9,  39,   
   34,   51,   56,   35,   29,   59,   17,   40,   13,   44,   37,   12,   25,   62,   53,   15,   54,   38,   41,   48,   
   58,   52,   7,  46,   31,   47,   32,   43,   63,   3,  1,  60,   23,   27,   42,   21,   8,  4,  9,  18,   55,   50,   
   17,   11,   40,   20,   6,  26,   25,   36,   14,   29,   12,   24,   49,   16,   56,   34,   51,   22,   53,   41,   
   38,   44,   32,   15,   2,  61,   54,   35,   30,   27,   42,   19,   59,   52,   8,  62,   48,   1,  21,   10,   50,   
   45,   18,   31,   7,  55,   28,   46,   57,   16,   40,   4,  43,   0,  63,   53,   14,   33,   37,   17,   5,  25,   
   13,   44,   49,   29,   36,   11,   32,   12,   54,   47,   52,   23,   42,   48,   34,   20,   56,   51,   8,  10,   
   2,  21,   1,  9,  30,   61,   38,   58,   45,   43,   0,  50,   24,   60,   6,  14,   35,   53,   46,   19,   25,   41,   
   63,   7,  39,   17,   5,  3,  12,   4,  52,   36,   31,   59,   26,   27,   42,   57,   51,   1,  11,   8,  10,   20,   
   9,  54,   32,   28,   18,   56,   0,  15,   44,   35,   37,   62,   25,   58,   49,   40,   22,   55,   53,   48,   46,   
   34,   39,   17,   52,   12,   2,  33,   19,   23,   29,   31,   14,   59,   38,   7,  57,   26,   63,   1,  11,   18,   
   51,   43,   21,   16,   30,   44,   5,  58,   49,   10,   40,   6,  41,   55,   56,   9,  28,   48,   4,  36,   25,   
   47,   24,   46,   53,   32,   31,   54,   37,   61,   13,   17,   22,   1,  62,   12,   23,   63,   52,   45,   14,   
   39,   34,   27,   2,  18,   26,   33,   29,   40,   3,  8,  16,   42,   55,   44,   0,  20,   10,   43,   38,   58,   
   28,   6,  57,   11,   24,   7,  35,   53,   61,   54,   50,   48,   32,   62,   22,   5,  34,   25,   31,   56,   23,   
   41,   17,   30,   13,   26,   3,  47,   8,  12,   51,   4,  9,  10,   52,   44,   28,   63,   40,   6,  20,   27,   0,  
   59,   16,   21,   50,   33,   46,   1,  14,   5,  19,   38,   48,   7,  58,   57,   37,   56,   13,   55,   45,   39,   
   43,   26,   3,  22,   25,   36,   18,   10,   29,   54,   35,   44,   60,   34,   51,   6,  12,   28,   49,   42,   53,   
   31,   15,   24,   17,   8,  62,   50,   14,   58,   57,   9,  23,   16,   38,   19,   32,   33,   63,   56,   2,  41,   
   61,   27,   43,   39,   26,   4,  22,   25,   18,   3,  47,   0,  36,   42,   55,   40,   60,   20,   37,   53,   62,   
   24,   51,   13,   45,   54,   49,   21,   31,   50,   48,   9,  19,   16,   56,   59,   7,  57,   11,   46,   63,   29,   
   6,  5,  32,   28,   14,   58,   38,   34,   22,   17,   36,   0,  52,   35,   41,   23,   61,   25,   45,   24,   43,   
   26,   39,   55,   31,   33,   4,  21,   50,   30,   49,   13,   10,   40,   44,   42,   48,   8,  62,   9,  54,   46,   
   18,   5,  37,   32,   38,   27,   12,   47,   3,  36,   61,   63,   53,   45,   56,   41,   60,   35,   52,   19,   17,   
   25,   7,  21,   43,   1,  33,   40,   2,  50,   55,   51,   10,   8,  28,   30,   20,   16,   44,   11,   31,   6,  0,  
   9,  34,   54,   57,   39,   23,   15,   26,   18,   47,   63,   58,   46,   60,   29,   37,   5,  13,   1,  38,   4,  25,  
   53,   2,  7,  3,  12,   59,   49,   30,   41,   21,   22,   33,   8,  20,   51,   36,   42,   62,   9,  31,   11,   6,  57,  
   0,  14,   56,   35,   54,   17,   28,   40,   32,   13,   29,   10,   1,  38,   45,   48,   27,   15,   63,   2,  30,   25,   4,  19};

const float freqList915[64] = {
  902.3,  902.5,  902.7,  902.9,  903.1,  903.3,  903.5,  903.7,  903.9,  904.1,
  904.3,  904.5,  904.7,  904.9,  905.1,  905.3,  905.5,  905.7,  905.9,  906.1,  
  906.3,  906.5,  906.7,  906.9,  907.1,  907.3,  907.5,  907.7,  907.9,  908.1,  
  908.3,  908.5,  908.7,  908.9,  909.1,  909.3,  909.5,  909.7,  909.9,  910.1,  
  910.3,  910.5,  910.7,  910.9,  911.1,  911.3,  911.5,  911.7,  911.9,  912.1,  
  912.3,  912.5,  912.7,  912.9,  913.1,  913.3,  913.5,  913.7,  913.9,  914.1,
  914.3,  914.5,  914.7,  914.9};

boolean hopFreq = true;
//byte dataPacket[64];
byte currentChnl = 0;
int16_t hopNum = 0;
int16_t nextHop;
int16_t nextHop2;
byte nextChnl;
byte nextChnl2;
int16_t pktNum = 0;
int16_t gndPktNum = 0;
byte hailChnl = 0;
float freq;
boolean sendPkt = false;

//void startRadio(){
//  startSPI(sensors.radioBusNum, pins.radioCS);
//  bus.radioSPI = bus.activeSPI;
//  hardware_spi1 = bus.activeSPI;}

void radioSendPacket(){
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
  bool TX;
  
//------------------------------------------------------------------
//                  PRE-FLIGHT PACKET
//------------------------------------------------------------------
  //send the preflight packet, 39 bytes, or 41 bytes if 915MHz FHSS
  if(events.preLiftoff){
    
    //hop frequency
    if(settings.FHSS && sensors.radio == 2){hopTXfreq();}
    
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
    if(sensors.radio == 2 && settings.FHSS){
      dataPacket[pktPosn]=nextChnl; pktPosn++;
      dataPacket[pktPosn]=nextChnl2; pktPosn++;}
    TX = rf95.send((uint8_t *)dataPacket, pktPosn);
    TXdataStart = micros();
    int32_t pktSize = pktPosn;
    pktPosn = 0;
    if(radioDebug && settings.testMode){
      if(TX){Serial.println(F("PreFlight Packet Sent"));Serial.print("PktSize: ");Serial.println(pktSize);}
      else if(!TX){Serial.println(F("PreFlight Packet Failed!"));}}
    if(settings.FHSS && sensors.radio == 2){
      hopNum = nextHop;
      hopFreq = true;
      gndPktNum++;
      if(gndPktNum%3==0 && hopSequence[nextHop] != hailChnl){syncFreq = true;gndPktNum = 0;}}}
    
//------------------------------------------------------------------
//                  IN-FLIGHT PACKET
//------------------------------------------------------------------
  //send inflight packet, 64 bytes
  //build the packet of 4 samples: 13 bytes per sample, 12 bytes GPS & pktnum, 13 x 4 + 12 = 64 bytes flight data
  else if(events.inFlight){  

    //update sample number
    sampNum++;
    
    //hop frequency if needed
    if(sensors.radio == 2 && settings.FHSS && hopFreq && sampNum >= packetSamples){hopTXfreq();}

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
    radio.alt = (int16_t)(baro.Alt);
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
      if(sensors.radio == 2){
        dataPacket[pktPosn]=nextChnl; pktPosn++;
        dataPacket[pktPosn]=nextChnl2; pktPosn++;}
      
      //GPS Data
      if(gpsTransmit){
        gpsTransmit=false;
        dataPacket[pktPosn] = lowByte(radio.GPSalt);pktPosn++;//55
        dataPacket[pktPosn] = highByte(radio.GPSalt);pktPosn++;//56
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlatitude.GPSbyte[i];pktPosn++;}//60
        for(byte i = 0; i < 4; i++){dataPacket[pktPosn]=GPSlongitude.GPSbyte[i];pktPosn++;}}//64
        
    //send packet
    TX = rf95.send((uint8_t *)dataPacket, pktPosn);
    TXdataStart = micros();
    if(radioDebug && settings.testMode){
      if(TX){Serial.print(F("InFlight Packet Sent, "));Serial.println(TXdataStart);}
      else if(!TX){Serial.println(F("InFlight Packet Failed!"));}}
    //reset counting variables
    sampNum = 0;
    pktPosn = 0;
    if(settings.FHSS && sensors.radio == 2){
      hopNum = nextHop;
      if(radio.packetnum%3==0){hopFreq = true;}//true
      if(radio.packetnum%9==0){syncFreq = true;}}//true
    radioTX = true;}}
    
//------------------------------------------------------------------
//                  POST-FLIGHT PACKET
//------------------------------------------------------------------
  //send post flight packet, 22 bytes
  else if(events.postFlight){

      //hop frequency
      if(settings.FHSS && sensors.radio == 2){hopTXfreq();}
    
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
      if(sensors.radio == 2){
        dataPacket[pktPosn]=hailChnl; pktPosn++;
        dataPacket[pktPosn]=hailChnl; pktPosn++;}
      TX = rf95.send((uint8_t *)dataPacket, pktPosn);
      TXdataStart = micros();
      //Serial debug
      if(radioDebug && settings.testMode){
        if(TX){Serial.println(F("PostFlight Packet Sent"));}
        else if(!TX){Serial.println(F("PostFlight Packet Failed!"));}}
      if(settings.FHSS && sensors.radio == 2){
        syncFreq = true;
        hopNum = nextHop;}
      
    }//end postFlight code

}//end radioSendPacket

void hopTXfreq(){
  
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
  int nextHop2;

  //Serial debug
  if(radioDebug && settings.testMode){
    Serial.print("Hopping Freq: ");Serial.print(freqList915[nextChnl], 3);
    Serial.print(", HopNum: ");Serial.print(nextHop);Serial.print(", time; ");Serial.println(micros());}
    
  rf95.setFrequency(freqList915[nextChnl]);
  hopNum = nextHop;
  nextHop = hopNum + 1;
  if(nextHop >= 2000){nextHop = 0;}
  currentChnl = hopSequence[hopNum];
  nextChnl = hopSequence[nextHop];
  nextHop2 = nextHop + 1;
  if(nextHop2 >= 2000){nextHop2 = 0;}
  nextChnl2 = hopSequence[nextHop2];
  hopFreq = false;}

void syncPkt(){

  Serial.println("Sync Packet");
  
  RH_RF95 rf95(pins.radioCS, pins.radioIRQ);
  float freq;

  //hop to the hailing channel
  freq = freqList915[hailChnl];
  if(liftoffSync){freq = freqList915[currentChnl];}
  if(!liftoffSync){rf95.setFrequency(freqList915[hailChnl]);}
  liftoffSync = false;

  //Serial debug
  if(radioDebug && settings.testMode){
    Serial.print("---Sending Sync Packet: "); Serial.println(freq, 3);
    Serial.print("---Sync nextChnl: ");Serial.print(nextChnl);Serial.print(", Freq: ");Serial.print(freqList915[nextChnl]);
    Serial.print(", time; ");Serial.println(micros());}
    
  //define packet
  dataPacket[0] = 255;//1
  dataPacket[1] = currentChnl;//2
  dataPacket[2] = nextChnl;//3
  dataPacket[3] = nextChnl2;//4
  //send packet
  rf95.send((uint8_t *)dataPacket, 4);
  syncFreq = false;}
