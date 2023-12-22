// Driver for RF Design RFD900 Radioes
//-----------------------------------------------
// Written by Étienne Didion
//-----------Change Log--------------------------
// 04 Dec 23: Version 1 I copied from RFD900 Driver and changed for RFD900
//----------------------------
// LIST OF FUNCTIONS & ROUTINES
//----------------------------
// beginRFD900(): starts the radio
// sendPktRFD900(): sends the packet

// temporaire pour l'instant:
#define radioPort Serial2
#define RADIO_UART_SPEED 57600
#define RADIO_TX_BUFFER_BYTES 1024

bool beginRFD900()
{
  // Cette radio n'a pas de fonction de RESET, nous supposons qu'elle est configurée
  // et fonctionne à l'aide du logi RFD avant de la connecter au système
  //
  // radio has no RESET fonction pin like the SPI LORA radio.
  // assume it is set correctly using RFD .exe app before starting.
  //
  // futur: reset + config with codes AT modem

  bool successFlag = true;

  radioPort.begin(RADIO_UART_SPEED);

  uint8_t txBuffer[RADIO_TX_BUFFER_BYTES];

  if (successFlag){Serial.println("RFD900 Radio OK!");}

  return successFlag;}

bool sendPktRFD900(uint8_t *data, uint8_t len){
  for (int i = 0; i < len; i++){
    radioPort.write(data[i]);
    radioPort.flush();}

  radioPort.println();
  radioPort.flush();
  // set flag
  radioFnctn = TXmode;
  TX = true;
  return true;}
