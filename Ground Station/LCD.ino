
void startupLCD(){

  //-----------------------------------------------------
  //Startup Display
  //-----------------------------------------------------
  //Start I2C
  Wire.begin();
  
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  lcd.begin(Wire);
  lcd.setBacklight(0, 0, 0); //Set backlight to bright med at (125, 125, 125)
  lcd.setContrast(50); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear();

  //Display the battery voltage
  //lcd.setCursor(0,0);
  lcd.print(F("Battery: "));
  float dispVolt = battVolt * 3.3 /(0.3125 * 1023.0);
  lcd.print(dispVolt);
  lcd.print('V');
  lcd.print('\r');
  //lcd.setCursor(0,1);
  if(SDinit){lcd.print(F("SD Card OK!"));}
  else{lcd.print(F("SD Card Failed!"));}
  //lcd.setCursor(0,2);
  lcd.print('\r');
  byte radioCode = 0;
  if(radio1status && radio2status){radioCode = 1;}
  else if(!radio1status && radio2status){radioCode = 2;}
  else if(radio1status && !radio2status){radioCode = 3;}
  else if(!radio1status && !radio2status){radioCode = 4;}
  switch (radioCode){
    case 1:
      lcd.print(F("Both Radios OK!"));
      break;

    case 2:
      lcd.print(F("Radio1 Failed!"));
      break;

    case 3:
      lcd.print(F("Radio2 Failed!"));
      break;

    case 4:
      lcd.print(F("Both Radios Failed!"));
      break;

    default:
      lcd.print(F("LCD Code Error"));
      break;}
  //lcd.setCursor(0,3);
  lcd.print('\r');
  lcd.print(radio1Freq, 3);
  lcd.print(F("      "));
  lcd.print(radio2Freq, 3);
  delay(5000);
  
  //Print to the LCD the last good coordinates from the previous flight
  lcd.clear();
  //----------------------------------------
  //Message - Line 1    
  //----------------------------------------
  lcd.print(F("Last Packet From:"));
  //----------------------------------------
  //Rocket Name - Line 2
  //----------------------------------------
  lcd.print('\r');
  lcd.print(rocketName);
  //----------------------------------------
  //GPS Latitude - Line 3
  //----------------------------------------
  lcd.print('\r');
  parseCoord(lastGPSlat);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlat);
  //----------------------------------------
  //GPS Latitude - Line 4
  //----------------------------------------
  lcd.print('\r');
  //lcd.print(lastGPSlon,4);
  parseCoord(lastGPSlon);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlon);}

void preflightLCD(){
  
  //-----------------------------------------------------
  //PreFlight Display
  //-----------------------------------------------------
  if(flightPhase != 1){lcd.clear(); flightPhase = 1;}
  //Setup Display Text
  //-------------------------------------------------
  //line 0: Display rocket name
  String myString = rocketName;
  myString.trim();
  for(byte i = myString.length()+1; i < 21; i++){myString += " ";}
  //line 1
  if(contCode == 5){myString += "All 3 Pyros Detected";}
  else if (contCode == 6){myString += "All 4 Pyros Detected";}
  else if (contCode == 7){myString += "Pyro Apogee Only";}
  else if (contCode == 8){myString += "Pyro Mains Only";}
  else if (contCode == 9){myString += "Pyro Mains & Apogee";}
  else if (contCode == 0){myString += "No Pyros Detected!";}
  else{myString += "No Cont Pyro ";myString += String(contCode);}
  for(byte i = myString.length()+1; i < 41; i++){myString += " ";}
  //line 2
  if(GPSlock == 1){
    myString += "GPS FIX OK! ";
    myString += String(satNum);myString += " SVs";}
  else{
    myString += "NO GPS FIX: ";
    myString += String(satNum);myString += " SVs";}
  for(byte i = myString.length()+1; i < 61; i++){myString += " ";}
  //line 3
  myString += "Base Alt: ";
  myString += String((long)(baseAlt*3.2808));
  myString += " ft";
  for(byte i = myString.length()+1; i < 81; i++){myString += " ";}
  lcd.print(myString);
  if(debugSerial){;Serial.println(myString);}

  }//end Preflight Code

void inflightLCD(){
  //-----------------------------------------------------------
  //Inflight Code
  //-----------------------------------------------------------
  if(flightPhase != 2){lcd.clear(); flightPhase = 2;}
  //-----------------------------------
  //event - line 1
  //-----------------------------------
  for(byte i = 0; i < 21; i++){dataString[i]= '\0';}
  strcpy_P(dataString, (char *)pgm_read_word(&(eventTable[event])));
  String myString = dataString;
  myString.trim();
  for(byte i = myString.length()+1; i < 21; i++){myString += " ";}
//  //light up the LCD if it has this capability
//  if(!ledLight){for(byte i = 0; i < sizeof(greenEvents) && !ledLight; i++){
//    if(event == greenEvents[i]){
//      lcd.setBacklight(0, greenIntensity, 0); //Set backlight to green
//      colorStart = micros();
//      ledLight = true;
//      i = sizeof(greenEvents);}}}
//  if(!ledLight){for(byte i = 0; i < sizeof(redEvents) && !ledLight; i++){
//    if(event == redEvents[i]){
//      lcd.setBacklight(redIntensity, 0, 0); //Set backlight to red
//      colorStart = micros();
//      ledLight = true;
//      i = sizeof(redEvents);}}}
    //----------------------------------------
  //display altitude - line 2
  //----------------------------------------
  myString += "Altitude: ";
  myString += String((long)(Alt*3.2808));
  myString += " ft";
  for(byte i = myString.length()+1; i < 41; i++){myString += " ";}
  //----------------------------------------
  //display velocity - line 3
  //----------------------------------------
  myString += "Speed: ";
  myString += String((long)(velocity*3.2808));
  myString += " fps";
  for(byte i = myString.length()+1; i < 61; i++){myString += " ";}
  //----------------------------------------
  //signal strength - line 4
  //----------------------------------------
  myString += "Signal: ";
  myString += String(signalStrength);
  myString += " dBm";
  for(byte i = myString.length()+1; i < 78; i++){myString += " ";}
  if(micros() - lastGPSfix < 2000000UL){myString += "GPS";}
  else{myString += "   ";}
  lcd.print(myString);
  if(debugSerial){Serial.println(myString);}
  }//end Inflight Code
  
void postflightLCD(){
  //----------------------------------------------------------- 
  //Postflight Code
  //-----------------------------------------------------------
  if(flightPhase != 3){lcd.clear(); flightPhase = 3;}
  //----------------------------------------
  //barometric altitude - line 1
  //----------------------------------------
  String myString = "Max Alt: ";
  myString += String((long)(maxAltitude*3.2808));
  myString += " ft";
  for(byte i = myString.length()+1; i < 21; i++){myString += " ";}
  //----------------------------------------
  //integrated velocity - line 2
  //----------------------------------------
  myString += "Max Speed: ";
  myString += String((long)(maxVelocity*3.2808));
  myString += " fps";
  for(byte i = myString.length()+1; i < 41; i++){myString += " ";}
  //----------------------------------------
  //GPS Latitude - Line 3
  //----------------------------------------
  if(GPSlock == 0){GPSlatitude = 1234.1234F; charGPSlat = 'N';}
  parseCoord(GPSlatitude);
  myString += dataString;
  myString.trim();
  myString += (char)39;
  myString += charGPSlat;
  for(byte i = myString.length()+1; i < 61; i++){myString += " ";}
  //----------------------------------------
  //GPS Latitude - Line 4
  //----------------------------------------
  if(GPSlock == 0){GPSlongitude = 1234.1234F; charGPSlon = 'W';}
  parseCoord(GPSlongitude);
  myString += dataString;
  myString.trim();
  myString += (char)39;
  myString += charGPSlon;
  for(byte i = myString.length()+1; i < 81; i++){myString += " ";}
  lcd.print(myString);
  if(debugSerial){Serial.println(myString);}
  }//end Postflight Code

void signalLostLCD(){
  
    //Write to the LCD
    if(flightPhase != 4){lcd.clear(); flightPhase = 4;}
    lastRX = micros();
    //----------------------------------------
    //Message - Line 1    
    //----------------------------------------
    String myString = "Signal Lost";
    for(byte i = myString.length()+1; i < 21; i++){myString += " ";}
    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    parseCoord(lastGPSlat);
    myString += dataString;
    myString.trim();
    myString += (char)39;
    myString += charGPSlat;
    for(byte i = myString.length()+1; i < 41; i++){myString += " ";}
    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    parseCoord(lastGPSlon);
    myString += dataString;
    myString.trim();
    myString += (char)39;
    myString += charGPSlon;
    for(byte i = myString.length()+1; i < 61; i++){myString += " ";}
    //----------------------------------------
    //Max Values - Line 3
    //----------------------------------------
    float dispAlt = maxAltitude * unitConvert;
    myString += String(dispAlt, 0);
    myString += "ft ";
    float dispVel = maxVelocity * unitConvert;
    myString += String(dispVel, 0);
    myString += "fps ";
    float dispG = (float)(maxG) * 0.029927521 / 9.80655;
    myString += String(dispG, 1);
    myString += 'G';
    for(byte i = myString.length()+1; i < 81; i++){myString += " ";}
    lcd.print(myString);
    if(debugSerial){Serial.println(myString);}}

void errorLCD(){
  lcd.clear();
  lcd.print(F("Pkt Code Error: "));
  lcd.print(event);}

void changeFreqLCD(){
  lcd.clear();
  String myString = "Change Freq to Ch: ";
  myString += String(chnl1);
  myString += '\r';
  myString += "Frequency: ";
  myString += String(freq1, 3);
  lcd.print(myString);
  lastRX = micros();
  delay(1000);}

void parseCoord(float coord){
    dtostrf(coord, 2, 4, dataString);
    strPosn=0;
    while(dataString[strPosn]!='.'){strPosn++;}
    byte endPosn = strPosn+1;
    while(dataString[endPosn]!='\0'){endPosn++;}
    while(endPosn >= strPosn - 2){dataString[endPosn+1]=dataString[endPosn]; endPosn--;}
    dataString[strPosn-2]=(char)223;}
