//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//radioSendPacket(): main routine to build and send telemetry packets
//hopTXfreq(): hops frequency when FHSS is active
//syncPkt(): sends a sync packet on the FHSS hailing freq
//radioSendPkt(): function to transmit the built packet to the radio
/*Radio Drivers
 * bool radioBegin(): starts code and checks radio functionality
 * bool setRadioPWR(uint8_t pwr): sets output power (0 - 20 dbm)
 * bool setRadioFreq(float freq): sets the radio frequency (expressed as MHz, ie 434.250)
 * bool radioSendPkt(uint8_t* data, uint8_t len): send a radio packet from a byte array, with length len
 * bool radioRecvPkt(uint8_t* data): recieve a packet and place data from FIFO into array "data"
 * bool radioSetMode(uint8_t mode): sets the radio mode, mostly used for setting RX continuous mode in setup
 */
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//18 AUG 22: eliminated RadioHead library and wrote independent drivers
//20 NOV 22: added in the callsign to the ham radio packet
//14 FEB 26: changed the FHSS hopping strategy to pass the hopSequence location in the sync packet, added mandatory callsign verification
//---------------------------------

/*900MHz FHSS Strategy

all flight phases: reject any packet that doesn't match the system callsign

preflight: 
- send ground packet every 1000ms
- sync packet sent on common hailing freq once per 2400ms plus a random delay between 0ms and 800ms after the last ground packet finishes transmitting
-- use one common hailing frequency, send sync packet after every 4th data packet
-- data in sync packet transmits the hop-sequence location known to both the transmitter and receiver
- ground packet sent on the frequency at the hop-sequence location from the sync packet

inflight:
- shift frequencies every 600ms, requivalently every 3rd packet
- Flight computer:
-- send 3 packets on one freq, sent then shift freq
-- every 4th shift (once per 2.4s) send sync packet on hailing frequency
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
const uint8_t hopSequence[2000] = {
  76, 117, 14, 63, 68, 7, 22, 114, 82, 55, 10, 3, 59, 83, 47, 40, 105, 49, 103, 45, 126, 95, 41, 51, 67, 
  24, 85, 121, 75, 31, 124, 50, 92, 37, 104, 29, 58, 110, 102, 106, 113, 36, 19, 111, 80, 8, 69, 7, 54, 
  63, 123, 119, 98, 22, 105, 118, 34, 88, 108, 15, 14, 94, 77, 68, 76, 91, 128, 2, 44, 64, 0, 4, 95, 58, 
  42, 38, 50, 100, 37, 99, 106, 55, 8, 61, 102, 32, 112, 41, 110, 122, 87, 127, 62, 97, 16, 27, 39, 49, 
  93, 22, 24, 71, 3, 56, 85, 29, 33, 90, 78, 73, 103, 117, 116, 28, 69, 64, 21, 108, 99, 77, 101, 7, 80, 
  100, 10, 31, 128, 109, 115, 112, 79, 48, 95, 41, 74, 110, 20, 86, 67, 2, 45, 23, 47, 29, 118, 66, 65, 
  59, 4, 70, 57, 54, 125, 9, 46, 50, 121, 96, 51, 15, 37, 55, 61, 82, 94, 56, 80, 75, 126, 98, 31, 106, 
  107, 103, 78, 113, 14, 124, 62, 67, 26, 115, 66, 71, 24, 21, 88, 128, 36, 117, 40, 105, 44, 111, 99, 
  32, 38, 77, 83, 84, 17, 18, 114, 104, 30, 8, 72, 87, 0, 74, 93, 116, 103, 57, 85, 29, 49, 27, 42, 3, 
  39, 25, 69, 71, 115, 108, 28, 54, 34, 10, 50, 13, 21, 100, 4, 20, 52, 105, 89, 43, 79, 98, 102, 1, 94, 
  56, 110, 96, 48, 44, 125, 107, 32, 83, 12, 49, 93, 124, 88, 120, 22, 77, 6, 101, 57, 58, 66, 18, 80, 
  15, 3, 27, 63, 24, 128, 92, 20, 23, 75, 95, 119, 2, 127, 45, 65, 43, 70, 118, 126, 73, 121, 5, 116, 
  98, 108, 25, 10, 64, 100, 11, 26, 91, 38, 32, 110, 114, 46, 56, 87, 81, 115, 34, 24, 68, 96, 85, 82, 
  37, 51, 127, 31, 1, 12, 16, 77, 86, 7, 94, 122, 103, 33, 15, 58, 9, 66, 36, 39, 20, 69, 3, 100, 101, 
  110, 65, 116, 2, 124, 92, 45, 72, 71, 42, 91, 70, 6, 55, 8, 13, 76, 73, 108, 75, 119, 7, 90, 122, 123, 
  49, 99, 56, 63, 12, 21, 28, 44, 86, 4, 98, 87, 120, 129, 39, 54, 32, 40, 29, 113, 46, 78, 107, 102, 
  112, 124, 18, 59, 116, 121, 5, 66, 42, 75, 53, 22, 20, 89, 91, 110, 104, 35, 57, 67, 10, 11, 31, 0, 
  108, 128, 7, 71, 76, 118, 98, 26, 17, 41, 114, 127, 28, 74, 6, 122, 3, 13, 56, 90, 112, 1, 94, 80, 75, 
  88, 66, 58, 93, 103, 85, 27, 61, 119, 64, 34, 44, 63, 102, 124, 16, 107, 82, 95, 49, 116, 36, 39, 19, 
  111, 128, 37, 127, 20, 109, 62, 114, 112, 70, 108, 23, 1, 22, 94, 26, 52, 67, 9, 91, 98, 0, 96, 77, 
  12, 113, 106, 65, 18, 8, 57, 54, 69, 29, 3, 102, 123, 100, 128, 38, 129, 44, 120, 45, 73, 11, 14, 74, 
  39, 68, 83, 53, 117, 114, 109, 97, 112, 118, 90, 82, 4, 28, 24, 25, 98, 61, 36, 84, 27, 59, 42, 8, 57, 
  115, 65, 79, 23, 102, 124, 35, 72, 12, 81, 37, 94, 75, 116, 93, 21, 40, 69, 73, 55, 46, 31, 117, 22, 
  82, 34, 13, 52, 17, 71, 56, 112, 38, 36, 7, 105, 68, 16, 58, 60, 63, 48, 104, 33, 77, 107, 41, 5, 99, 
  90, 126, 15, 24, 124, 35, 118, 53, 127, 100, 92, 80, 55, 31, 84, 42, 64, 8, 62, 57, 71, 27, 102, 39, 
  36, 94, 76, 123, 95, 50, 18, 112, 125, 28, 120, 109, 83, 110, 68, 99, 121, 104, 24, 89, 16, 114, 87, 
  14, 32, 45, 9, 10, 7, 26, 81, 5, 77, 105, 103, 6, 27, 108, 111, 118, 123, 41, 78, 69, 53, 106, 22, 64, 
  15, 38, 55, 101, 95, 12, 31, 97, 109, 50, 40, 28, 126, 71, 42, 2, 52, 35, 85, 129, 14, 68, 5, 45, 84, 
  72, 61, 26, 67, 62, 18, 29, 8, 93, 59, 13, 54, 119, 22, 0, 56, 6, 113, 81, 65, 92, 12, 99, 90, 91, 
  110, 77, 101, 82, 74, 76, 107, 66, 111, 57, 102, 96, 5, 16, 39, 34, 117, 124, 100, 2, 51, 72, 9, 89, 
  87, 23, 121, 73, 30, 105, 129, 127, 125, 68, 62, 61, 65, 53, 126, 63, 43, 47, 120, 84, 85, 80, 40, 78, 
  99, 45, 5, 76, 10, 22, 64, 117, 122, 83, 27, 52, 33, 94, 48, 110, 67, 57, 92, 46, 25, 42, 56, 36, 55, 
  82, 28, 32, 0, 91, 35, 62, 120, 16, 101, 90, 60, 102, 11, 112, 119, 73, 116, 20, 106, 14, 2, 45, 118, 
  40, 69, 51, 23, 47, 111, 8, 104, 57, 41, 30, 9, 100, 65, 113, 52, 21, 4, 54, 29, 62, 27, 93, 82, 53, 
  42, 127, 86, 58, 10, 84, 77, 126, 17, 26, 128, 18, 122, 129, 48, 7, 73, 0, 34, 103, 14, 13, 39, 81, 5, 
  28, 85, 20, 59, 108, 43, 46, 90, 79, 76, 29, 54, 121, 123, 104, 66, 42, 75, 78, 3, 69, 65, 74, 95, 11, 
  24, 33, 114, 53, 64, 105, 103, 41, 82, 113, 86, 25, 71, 97, 2, 17, 72, 98, 106, 119, 94, 56, 93, 39, 
  70, 91, 20, 40, 6, 66, 55, 5, 37, 83, 4, 87, 65, 99, 46, 24, 18, 42, 122, 121, 0, 31, 68, 126, 82, 105, 
  61, 58, 63, 75, 90, 120, 72, 106, 27, 96, 95, 52, 119, 80, 110, 8, 88, 81, 112, 79, 5, 44, 125, 128, 53, 
  116, 89, 51, 43, 17, 16, 107, 47, 111, 15, 34, 69, 24, 122, 25, 100, 121, 68, 127, 7, 41, 38, 77, 118, 
  6, 76, 85, 96, 101, 26, 112, 37, 93, 102, 70, 120, 28, 19, 11, 46, 126, 78, 2, 59, 103, 18, 27, 13, 56, 
  60, 42, 21, 43, 64, 122, 88, 109, 0, 123, 20, 73, 65, 90, 10, 118, 101, 57, 94, 111, 49, 34, 71, 92, 66, 
  116, 61, 100, 77, 33, 38, 112, 12, 93, 125, 31, 8, 114, 5, 63, 127, 67, 86, 78, 124, 96, 44, 45, 18, 19, 
  56, 0, 52, 30, 62, 16, 40, 28, 13, 108, 76, 48, 83, 109, 64, 75, 1, 58, 39, 97, 21, 117, 110, 17, 26, 23, 
  90, 22, 91, 50, 123, 51, 125, 121, 45, 46, 41, 63, 54, 2, 105, 67, 81, 43, 4, 102, 38, 122, 88, 16, 72, 
  87, 34, 8, 36, 37, 115, 11, 33, 89, 84, 68, 1, 52, 5, 53, 85, 71, 123, 59, 64, 127, 118, 12, 35, 110, 
  119, 62, 3, 49, 21, 25, 124, 112, 86, 96, 26, 109, 77, 129, 125, 126, 22, 114, 97, 39, 4, 67, 31, 33, 70, 
  32, 74, 29, 82, 46, 2, 30, 94, 51, 100, 104, 128, 102, 35, 13, 121, 81, 3, 37, 18, 86, 12, 112, 20, 63, 
  66, 44, 43, 15, 129, 23, 105, 40, 84, 97, 78, 124, 71, 5, 118, 114, 45, 106, 89, 103, 31, 65, 69, 32, 24, 
  119, 109, 2, 96, 4, 99, 56, 92, 91, 86, 29, 50, 111, 121, 82, 52, 90, 110, 27, 87, 49, 94, 22, 107, 0, 
  75, 28, 59, 108, 128, 122, 9, 36, 76, 125, 127, 105, 46, 7, 103, 89, 55, 78, 4, 109, 101, 113, 83, 57, 
  25, 115, 67, 121, 100, 117, 27, 32, 6, 85, 18, 95, 99, 71, 62, 79, 111, 90, 31, 110, 128, 102, 52, 24, 13, 
  23, 127, 72, 54, 73, 88, 28, 112, 12, 3, 118, 129, 45, 35, 56, 68, 21, 105, 19, 50, 89, 120, 103, 114, 69, 
  51, 99, 64, 126, 93, 100, 1, 77, 8, 94, 86, 74, 106, 14, 41, 18, 117, 16, 0, 72, 47, 26, 42, 5, 20, 31, 
  48, 71, 66, 45, 95, 12, 56, 87, 98, 59, 90, 73, 21, 24, 40, 23, 32, 114, 91, 119, 81, 85, 123, 74, 54, 52, 
  37, 129, 110, 53, 60, 4, 9, 115, 69, 50, 124, 26, 2, 6, 66, 71, 12, 38, 121, 8, 77, 63, 92, 30, 117, 35, 
  86, 103, 67, 10, 72, 73, 45, 32, 39, 17, 24, 80, 87, 0, 82, 41, 120, 116, 31, 15, 37, 69, 21, 114, 22, 
  100, 55, 1, 78, 6, 85, 79, 66, 94, 5, 20, 30, 47, 107, 98, 50, 106, 63, 48, 23, 97, 4, 27, 53, 34, 122, 
  68, 119, 61, 112, 51, 64, 71, 96, 56, 21, 33, 104, 81, 36, 86, 45, 78, 32, 126, 105, 37, 69, 100, 73, 38, 
  57, 92, 123, 125, 16, 0, 24, 30, 59, 88, 116, 103, 108, 17, 10, 102, 117, 76, 65, 54, 60, 122, 118, 68, 
  85, 29, 72, 61, 109, 55, 18, 66, 119, 124, 11, 127, 15, 8, 44, 77, 91, 38, 56, 99, 30, 21, 27, 88, 48, 45, 
  79, 74, 47, 87, 26, 121, 57, 98, 49, 110, 73, 28, 53, 118, 78, 100, 120, 13, 35, 3, 46, 86, 80, 5, 54, 33, 
  104, 66, 89, 60, 84, 61, 67, 113, 96, 65, 112, 32, 30, 7, 37, 129, 87, 69, 57, 25, 101, 117, 102, 83, 14, 
  95, 2, 92, 79, 28, 127, 42, 77, 19, 31, 46, 3, 122, 33, 116, 8, 68, 20, 81, 109, 5, 48, 34, 11, 90, 24, 
  119, 76, 108, 52, 80, 61, 64, 73, 13, 111, 36, 17, 107, 32, 118, 121, 91, 6, 88, 128, 103, 14, 29, 31, 4, 
  127, 70, 38, 79, 87, 30, 85, 89, 123, 97, 92, 1, 16, 45, 56, 51, 27, 95, 43, 46, 65, 124, 7, 8, 39, 82, 
  104, 23, 61, 13, 37, 53, 72, 42, 91, 0, 2, 80, 73, 102, 24, 52, 77, 87, 59, 62, 60, 17, 38, 115, 105, 96, 
  57, 3, 16, 125, 63, 106, 43, 7, 95, 68, 36, 64, 6, 82, 18, 48, 66, 71, 129, 121, 122, 40, 74, 47, 30, 54, 
  80, 114, 5, 29, 46, 100, 59, 104, 42, 55, 128, 88, 116, 34, 27, 15, 57, 44, 58, 0, 45, 36, 81, 112, 52, 
  96, 117, 8, 97, 2, 113, 43, 91, 41, 33, 63, 39, 72, 7, 16, 38, 127, 114, 85, 107, 71, 77, 59, 49, 108, 
  111, 65, 46, 19, 83, 13, 61, 44, 32, 53, 110, 125, 86, 100, 4, 92, 104, 70, 101, 3, 84, 42, 68, 33, 117, 
  28, 7, 78, 99, 63, 27, 73, 21, 54, 80, 107, 89, 127, 15, 97, 20, 9, 46, 22, 47, 6, 87, 11, 43, 61, 19, 13, 
  30, 123, 56, 65, 88, 118, 76, 67, 3, 49, 24, 108, 115, 122, 71, 32, 64, 35, 86, 2, 80, 126, 28, 60, 51, 
  102, 117, 55, 62, 20, 8, 47, 116, 45, 95, 104, 27, 22, 113, 83, 72, 92, 53, 59, 17, 46, 127, 38, 58, 125, 
  18, 52, 110, 94, 124, 32, 21, 89, 63, 76, 44, 101, 81, 30, 29, 67, 99, 31, 51, 118, 105, 71, 96, 68, 104, 
  74, 93, 0, 73, 78, 117, 36, 24, 86, 91, 128, 12, 15, 10, 65, 113, 28, 33, 54, 21, 109, 83, 6, 116, 82, 79, 
  69, 5, 16, 122, 23, 62, 80, 76, 108, 29, 19, 74, 81, 89, 63, 88, 32, 93, 101, 43, 112, 87, 123, 18, 42, 
  115, 10, 125, 70, 11, 61, 100, 102, 68, 55, 48, 104, 49, 73, 21, 33, 59, 118, 98, 25, 79, 27, 22, 41, 47, 
  15, 67, 57, 71, 108, 113, 56, 72, 85, 122, 0, 111, 88, 14, 38, 6, 28, 87, 120, 78, 58, 1, 60, 101, 76, 96, 
  106, 128, 90, 100, 30, 52, 8, 51, 34, 59, 103, 22, 39, 35, 41, 108, 10, 31, 98, 46, 29, 105, 121, 43, 67, 
  50, 74, 17, 33, 32, 62, 63, 6};

const float freqList915[130] = {
  902.1,  902.3,  902.5,  902.7,  902.9,  903.1,  903.3,  903.5,  903.7,  903.9,  
  904.1,  904.3,  904.5,  904.7,  904.9,  905.1,  905.3,  905.5,  905.7,  905.9,
  906.1,  906.3,  906.5,  906.7,  906.9,  907.1,  907.3,  907.5,  907.7,  907.9,  
  908.1,  908.3,  908.5,  908.7,  908.9,  909.1,  909.3,  909.5,  909.7,  909.9,  
  910.1,  910.3,  910.5,  910.7,  910.9,  911.1,  911.3,  911.5,  911.7,  911.9,  
  912.1,  912.3,  912.5,  912.7,  912.9,  913.1,  913.3,  913.5,  913.7,  913.9,
  914.1,  914.3,  914.5,  914.7,  914.9,  915.1,  915.3,  915.5,  915.7,  915.9,
  916.1,  916.3,  916.5,  916.7,  916.9,  917.1,  917.3,  917.5,  917.7,  917.9,
  918.1,  918.3,  918.5,  918.7,  918.9,  919.1,  919.3,  919.5,  919.7,  919.9,
  920.1,  920.3,  920.5,  920.7,  920.9,  921.1,  921.3,  921.5,  921.7,  921.9,
  922.1,  922.3,  922.5,  922.7,  922.9,  923.1,  923.3,  923.5,  923.7,  923.9,
  924.1,  924.3,  924.5,  924.7,  924.9,  925.1,  925.3,  925.5,  925.7,  925.9,
  926.1,  926.3,  926.5,  926.7,  926.9,  927.1,  927.3,  927.5,  927.7,  927.9};

struct {
  bool hopNow = true;
  int16_t hopNum = 0;
  int16_t nextHop = 1;
  uint8_t currentChnl = 0;
  uint8_t nextChnl;
  int16_t pktNum = 0;
  int16_t gndPktNum = 0;
  uint8_t hailChnl = 0;
} FHSS;
float freq;
union{
    uint32_t val = 0;
    uint8_t Byte[4];
  } idUnion; 

void beginTelemetry(){

  //set the hailing frequency channel
  if(settings.FHSS){
    FHSS.hailChnl = (uint8_t)(5*(settings.TXfreq - 902.300F));
    //we need to reset the user defined frequency to be the closest LoRa channel
    settings.TXfreq = freqList915[FHSS.hailChnl];}

  //set the packet header to the callsign
  for(uint8_t i = 0; i<6; i++){dataPacket[i] = settings.callSign[i];}
}

void buildTelmetryPkt(){

//------------------------------------------------------------------
//                  PRE-FLIGHT PACKET
//------------------------------------------------------------------
  //send the preflight packet, 42 bytes
  if(events.preLiftoff){
    
    //hop frequency
    if(settings.FHSS){hopTXfreq();}
    
    //start data packet build after the static callsign header
    dataPacket[6]=radio.event;//7
    dataPacket[7]=gnss.fix;//8
    dataPacket[8]=cont.reportCode;//9
    for (uint8_t j = 0; j < sizeof(settings.rocketName); j++){dataPacket[9+j] = settings.rocketName[j];}//23
    dataPacket[29]=lowByte(radio.baseAlt);//24
    dataPacket[30]=highByte(radio.baseAlt);//25
    dataPacket[31]=lowByte(radio.GPSalt);//26
    dataPacket[32]=highByte(radio.GPSalt);//27
    floatUnion.val = GPS.location.lat();
    for(byte i = 0; i < 4; i++){dataPacket[33+i]=floatUnion.Byte[i];}//31
    floatUnion.val = GPS.location.lng();
    for(byte i = 0; i < 4; i++){dataPacket[37+i]=floatUnion.Byte[i];}//35
    dataPacket[41]=radio.satNum;//36

    //send the packet
    sendPkt = true;
    pktSize = 42;
    if(radioDebug && settings.testMode){if(settings.serialDebug==3){Serial.println("");} Serial.print(F("PreFlight Packet Sent"));}

    //set the FHSS flags
    if(settings.FHSS){
      FHSS.gndPktNum++;
      if(FHSS.gndPktNum%4==0){
        syncFreq = true;
        FHSS.gndPktNum = 0;}}}
    
//------------------------------------------------------------------
//                  IN-FLIGHT PACKET
//------------------------------------------------------------------
  //build and send inflight packet 
  //packet structure: 6 bytes callsign, 11 bytes per sample, 4 samples per packet, 12 bytes GPS & pktnum
  //packet requirements: 11 x 4 + 12 + 6 = 62 bytes per packet
  else if(events.inFlight){  

    //check to see if an SD card latency made us miss a sample
    uint32_t sampleTime = 0UL;
    sampleTime = micros();
    if(sampNum > 0 && sampleTime - radio.lastSampTime > 100000UL){Serial.println("Sample Missed");}

    radio.lastSampTime = sampleTime;

    //update sample number
    sampNum++;
    pktPosn = (sampNum -1) * 11 + 6;
    
    //hop frequency if needed
    if(settings.FHSS && FHSS.hopNow && sampNum >= packetSamples){hopTXfreq();}

    //event
    dataPacket[pktPosn] = radio.event;//1
    //velocity
    dataPacket[pktPosn + 1] = lowByte(radio.vel);//2
    dataPacket[pktPosn + 2] = highByte(radio.vel);//3
    //altitude
    dataPacket[pktPosn + 3] = lowByte(radio.alt);//4
    dataPacket[pktPosn + 4] = highByte(radio.alt);//5
    //Roll data
    radio.roll = rollZ;
    dataPacket[pktPosn + 5] = lowByte(radio.roll);//6
    dataPacket[pktPosn + 6] = highByte(radio.roll);//7
    //Off Vertical data
    radio.offVert = offVert;
    dataPacket[pktPosn + 7] = lowByte(radio.offVert);//8
    dataPacket[pktPosn + 8] = highByte(radio.offVert);//9
    //Acceleration
    radio.accel = (int16_t)(accelNow * 33.41406087); //33.41406087 = 32768 / 9.80665 / 100
    dataPacket[pktPosn + 9] = lowByte(radio.accel);//10
    dataPacket[pktPosn + 10] = highByte(radio.accel);//11
      
    //GPS & packet data collected once per packet at 12 bytes
    if(sampNum >= packetSamples){

      //reset the sample number
      sampNum = 0;

      //update packet number
      radio.packetnum++;
      dataPacket[50] = lowByte(radio.packetnum);//51
      dataPacket[51] = highByte(radio.packetnum);//52
      //GPS Data
      dataPacket[52] = lowByte(radio.GPSalt);//53
      dataPacket[53] = highByte(radio.GPSalt);//54
      floatUnion.val = GPS.location.lat();
      for(uint8_t i = 0; i < 4; i++){dataPacket[54 + i]=floatUnion.Byte[i];}//58
      floatUnion.val = GPS.location.lng();
      for(uint8_t i = 0; i < 4; i++){dataPacket[58 + i]=floatUnion.Byte[i];}//62

      //send packet
      sendPkt = true;
      pktSize = 62;
      //debug output
      if(radioDebug && settings.testMode){
        if(settings.serialDebug==3){Serial.println("");}
        Serial.print(F("InFlight Packet Sent "));Serial.print(radio.packetnum);}

      //set FHSS flags      
      if(settings.FHSS){
        if(radio.packetnum%3==0){FHSS.hopNow = true;}
        if(radio.packetnum%12==0){syncFreq = true;}}
    }}
    
//------------------------------------------------------------------
//                  POST-FLIGHT PACKET
//------------------------------------------------------------------
  //send post flight packet: 26 bytes per packet
  else if(events.postFlight){

      //hop frequency
      if(settings.FHSS){hopTXfreq();}
    
      dataPacket[6]=radio.event;//7 bytes
      dataPacket[7]=lowByte(radio.maxAlt);//8 bytes
      dataPacket[8]=highByte(radio.maxAlt);//9 bytes
      dataPacket[9]=lowByte(radio.maxVel);//10 bytes
      dataPacket[10]=highByte(radio.maxVel);//11 bytes
      dataPacket[11]=lowByte(radio.maxG);//12 bytes
      dataPacket[12]=highByte(radio.maxG);//13 bytes
      dataPacket[13]=lowByte(radio.maxGPSalt);//14 bytes
      dataPacket[14]=highByte(radio.maxGPSalt);//15 bytes
      dataPacket[15]=gnss.fix;//16 bytes
      dataPacket[16]=lowByte(radio.GPSalt);//17 bytes
      dataPacket[17]=highByte(radio.GPSalt);//18 bytes
      floatUnion.val = GPS.location.lat();
      for(uint8_t i = 0; i < 4; i++){dataPacket[18+i]=floatUnion.Byte[i];}//22 bytes
      floatUnion.val = GPS.location.lng();
      for(uint8_t i = 0; i < 4; i++){dataPacket[22+i]=floatUnion.Byte[i];}//26 bytes

      //send the packet
      sendPkt = true;
      pktSize = 26;
      if(settings.FHSS){syncFreq = true;}
       //debug output
      if(radioDebug && settings.testMode){if(settings.serialDebug==3){Serial.println("");}Serial.print(F("Post Flight Packet Sent"));}

    }//end postFlight packet

  //turn off the flag now that we've processed the packet command
  buildPkt = false;}//end radioSendPacket

void sendTelemetryPkt(){

  static uint32_t TXlastStart = 0UL;

  TX = radioSendPkt(dataPacket, pktSize);
  TXstartTime = micros();
  if(events.liftoff && TX){SDradioTX = true;}
  //Serial debug
  if(radioDebug && settings.testMode){
    if(TX){Serial.print(F("...Success! Packet size "));Serial.print(pktSize);}
    else if(!TX){Serial.print(F("..Failed! Packet size "));Serial.print(pktSize);}
    Serial.print(", time between packets ");Serial.print(TXstartTime - TXlastStart);}
  TXlastStart = TXstartTime;
  //reset the send flag
  sendPkt = false;}

void hopTXfreq(){

  //Serial debug
  if(radioDebug && settings.testMode){
    if(settings.serialDebug==3){Serial.println("");}
    Serial.print("Hopping Freq: ");Serial.print(freqList915[FHSS.nextChnl], 3);
    Serial.print(", HopNum: ");Serial.print(FHSS.nextHop);Serial.print(", timeStamp; ");Serial.print(micros());}

  //set the radio to the new frequency
  setRadioFreq(freqList915[FHSS.nextChnl]);
  FHSS.currentChnl = FHSS.nextChnl;

  //identify the next channel in the hop sequence
  FHSS.hopNum = FHSS.nextHop;
  FHSS.nextHop++;
  uint16_t hopLimit = sizeof(hopSequence)/sizeof(hopSequence[0]);
  if(FHSS.nextHop >= hopLimit){FHSS.nextHop = 0;}
  //move again if the next channel is the hail channel
  if(hopSequence[FHSS.nextHop] == FHSS.hailChnl){
    FHSS.nextHop++;
    if(FHSS.nextHop >= hopLimit){FHSS.nextHop = 0;}}

  //set the next channel nunber
  FHSS.nextChnl = hopSequence[FHSS.nextHop];

  //reset the flag hopFreq
  FHSS.hopNow = false;}

void syncPkt(){

  //get the hailing frequency
  float freq = freqList915[FHSS.hailChnl];

  //hop to the hailing channel
  setRadioFreq(freq);

  //Serial debug
  if(radioDebug && settings.testMode){
    Serial.print("---Sending Sync Packet: "); Serial.println(freq, 3);
    Serial.print("---Sync nextHop: ");Serial.print(FHSS.nextHop);Serial.print(", nextChnl: ");Serial.print(FHSS.nextChnl);
    Serial.print(", timeStamp; ");Serial.print(micros());}
    
  //define packet of 10 bytes
  dataPacket[6] = 255;//7
  dataPacket[7] = FHSS.currentChnl;//8
  dataPacket[8] = lowByte(FHSS.nextHop);//9
  dataPacket[9] = highByte(FHSS.nextHop);//10

  //send packet if the current or next channel are not the hail channel
  radioSendPkt(dataPacket, 10);
  syncFreq = false;
  noInterrupts();
  syncFlag = false;
  interrupts();}