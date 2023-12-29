
#define radioPort Serial2
#define RADIO_UART_SPEED 57600


bool beginRFD900()
{


  boolean successFlag = true;

  radioPort.begin(RADIO_UART_SPEED);


  if (successFlag)
  {
    Serial.println("RFD900 Radio OK!");
  }

  return successFlag;
}

bool sendPktRFD900(uint8_t *data, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    radioPort.write(data[i]);
    radioPort.flush();
  }
  radioPort.println();
  radioPort.flush();
  // set flag
  radioFnctn = TXmode;
  TX = true;
  return true;
}
