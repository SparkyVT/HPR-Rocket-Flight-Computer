
void startupLCD(){

  //-----------------------------------------------------
  //Startup Display
  //-----------------------------------------------------
  //Start I2C
  Wire.begin();
  lcd.begin(Wire);
  delay(50);
  lcd.setBacklight(75, 75, 75); //Set backlight to bright med at (125, 125, 125)
  lcd.setContrast(50); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear();

  //Display the battery voltage
  lcd.print(F("Battery: "));
  float dispVolt = battVolt * 3.3 /(0.3125 * 1023.0);
  lcd.print(dispVolt);
  lcd.print('V');
  lcd.print('\r');
  if(SDinit){lcd.print(F("SD Card OK!"));}
  else{lcd.print(F("SD Card Failed!"));}
  lcd.print('\r');
  //Radio 1
  if(radio1.enable && radio1.status){lcd.print(F("Radio 1: "));lcd.print(radio1.frq,3);lcd.print(" MHz");}
  else if(radio1.enable && !radio1.status){lcd.print(F("Radio 1: Failed!"));}
  else if(!radio1.enable){lcd.print(F("Radio 1: Sleep"));}
  lcd.print('\r');
  //Radio 2
  if(radio2.enable && radio2.status){lcd.print(F("Radio 2: "));lcd.print(radio2.frq,3);lcd.print(" MHz");}
  else if(radio2.enable && !radio2.status){lcd.print(F("Radio 2: Failed!"));}
  else if(!radio2.enable){lcd.print(F("Radio 2: Sleep"));}
  delay(8000);
  
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
  if(isnan(lastGPSlat) || lastGPSlat == 0.0){lastGPSlat = 1234.5678;}
  parseCoord(lastGPSlat);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlat);
  //----------------------------------------
  //GPS Latitude - Line 4
  //----------------------------------------
  lcd.print('\r');
  if(isnan(lastGPSlon) || lastGPSlon == 0.0){lastGPSlon = 1234.5678;}
  parseCoord(lastGPSlon);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlon);}

void preflightLCD(){
  
  //-----------------------------------------------------
  //PreFlight Display
  //-----------------------------------------------------
  if(flightPhase != 1){lcd.clear();flightPhase = 1;}
  
  //Setup Display Text
  //-------------------------------------------------
  //line 0: Display rocket name
  String myString = rocketName;
  myString.trim();
  for(byte i = myString.length()+1; i < 21; i++){myString += " ";}
  //line 1
  myString += pyroTable[contCode];
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
  String myString = eventTable[event];
  myString.trim();
  for(byte i = myString.length()+1; i < 21; i++){myString += " ";}
  Serial.println(myString);
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
    if(isnan(lastGPSlat) || lastGPSlat == 0.0){lastGPSlat = 1234.5678;}
    parseCoord(lastGPSlat);
    myString += dataString;
    myString.trim();
    myString += (char)39;
    myString += charGPSlat;
    for(byte i = myString.length()+1; i < 41; i++){myString += " ";}
    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    if(isnan(lastGPSlon) || lastGPSlon == 0.0){lastGPSlon = 1234.5678;}
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
  Serial.println("---Pkt Error---");
  dispPktInfo();
  flightPhase = 0;
  lcd.clear();
  String myString = "Pkt Code Error: ";
  myString += String(event);
  lcd.print(myString);}

void changeFreqLCD(){
  flightPhase = 0;
  lcd.clear();
  String myString = "Change Freq to Ch: ";
  myString += String(activeRadio->chnl1);
  myString += '\r';
  myString += "Frequency: ";
  myString += String(activeRadio->frq, 3);
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
