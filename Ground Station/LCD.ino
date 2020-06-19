void startupLCD(){

  //-----------------------------------------------------
  //Startup Display
  //-----------------------------------------------------

  lcd.clear();

  //Display the battery voltage
  lcd.setCursor(0,0);
  lcd.print(F("Battery: "));
  lcd.print((float)battVolt * 33.33 / 1023.0);
  lcd.print('V');
  lcd.setCursor(0,1);
  if(SDinit){lcd.print(F("SD Card OK!"));}
  else{lcd.print(F("SD Card Failed!"));}
  lcd.setCursor(0,2);
  if(radioSetup){lcd.print(F("Radio OK!"));}
  else{lcd.print(F("Radio Failed!"));}
  lcd.setCursor(0,3);
  lcd.print(F("Frequency: "));
  lcd.print(radioFreq[chnl], 3);
  delay(4000);
  
  //Print to the LCD the last good coordinates from the previous flight
  
  lcd.clear();
  //----------------------------------------
  //Message - Line 1    
  //----------------------------------------
  lcd.setCursor(0,0);
  lcd.print(F("Last Packet From: "));
  //----------------------------------------
  //Rocket Name - Line 2
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print(rocketName);
  //----------------------------------------
  //GPS Latitude - Line 3
  //----------------------------------------
  lcd.setCursor(0,2);
  displayLat();
  
  //----------------------------------------
  //GPS Longitude - Line 4
  //----------------------------------------
  lcd.setCursor(0,3);
  displayLon();
  }

void preflightLCD(){
  //-----------------------------------------------------
  //PreFlight Display
  //-----------------------------------------------------

  //Setup Display Text
  //-------------------------------------------------
  //line 0: Display rocket name
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(rocketName);
  //line 1
  lcd.setCursor(0,1);
  if(contCode == 5){lcd.print(F("All 3 Pyros Detected"));}
  else if (contCode == 6){lcd.print(F("All 4 Pyros Detected"));}
  else if (contCode == 7){lcd.print(F("Pyro Apogee Only"));}
  else if (contCode == 8){lcd.print(F("Pyro Mains Only"));}
  else if (contCode == 9){lcd.print(F("Pyro Mains & Apogee"));}
  else if (contCode ==10){lcd.print(F("No Pyros Detected!"));}
  else{lcd.print(F("No Cont Pyro "));lcd.print(contCode);}
  //line 2
  lcd.setCursor(0,2);
  if(GPSlock == 1){
    lcd.print(F("GPS FIX OK! "));
    lcd.print(satNum);lcd.print(F(" SVs"));}
  else{
    lcd.print(F("NO GPS FIX: "));
    lcd.print(satNum);lcd.print(F(" SVs"));}
  //line 3
  lcd.setCursor(0,3);
  lcd.print(F("Base Alt: "));
  displayAlt(baseAlt);

  }//end Preflight Code

void inflightLCD(){
  //-----------------------------------------------------------
  //Inflight Code
  //-----------------------------------------------------------
  //-----------------------------------
  //event - line 0
  //-----------------------------------
  lcd.clear();
  lcd.setCursor(0,0);
  for(byte i = 0; i < 21; i++){dataString[i]= '\0';}
  strcpy_P(dataString, (char *)pgm_read_word(&(eventTable[event])));
  lcd.print(dataString);
  //light up the LCD if it has this capability
  /*for(byte i = 0; i < sizeof(greenEvents); i++){
    if(event == greenEvents[i]){
      litePin = GREENLITE;
      if(!liteUp){liteLED(litePin);}
      break;}}
  for(byte i = 0; i < sizeof(redEvents); i++){
    if(event == redEvents[i]){
      litePin = REDLITE;
      if(!liteUp){liteLED(litePin);}
      break;}}*/
    //----------------------------------------
  //display altitude - line 1
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print(F("Altitude: "));
  displayAlt(Alt);
  //----------------------------------------
  //display velocity - line 2
  //----------------------------------------
  lcd.setCursor(0,2);
  lcd.print(F("Speed: "));
  displayVel(velocity);
  //----------------------------------------
  //signal strength - line 3
  //----------------------------------------
  lcd.setCursor(0,3);
  lcd.print(F("Signal: "));
  lcd.print(signalStrength);
  lcd.print(F(" dBm"));
  if(micros() - lastGPSfix < 2000000UL){
    lcd.setCursor(17,3);
    lcd.print(F("GPS"));}
  }//end Inflight Code
  
void postflightLCD(){
 //----------------------------------------------------------- 
 //Postflight Code
 //-----------------------------------------------------------

  //----------------------------------------
  //barometric altitude - line 0
  //----------------------------------------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Max Alt: "));
  displayAlt(maxAltitude);
  
  //----------------------------------------
  //integrated velocity - line 1
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print(F("Max Speed: "));
  displayVel(maxVelocity);

  //----------------------------------------
  //GPS Latitude - Line 2
  //----------------------------------------
  lcd.setCursor(0,2);
  displayLat();
  
  //----------------------------------------
  //GPS Longitude - Line 2
  //----------------------------------------
  lcd.setCursor(0,3);
  displayLon();

  }//end Postflight Code

void signalLostLCD(){
  
    //Write to the LCD
    lcd.clear();
    lastRX = micros();
    //----------------------------------------
    //Message - Line 1    
    //----------------------------------------
    lcd.setCursor(0,0);
    lcd.print(F("Signal Lost"));

    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    lcd.setCursor(0,1);
    displayLat();
  
    //----------------------------------------
    //GPS Longitude - Line 3
    //----------------------------------------
    lcd.setCursor(0,2);
    displayLon();

    //----------------------------------------
    //Max Values - Line 4
    //----------------------------------------
    lcd.setCursor(0,3);
    displayAlt(maxAltitude);
    lcd.print(' ');
    displayVel(maxVelocity);
    lcd.print(' ');
    lcd.print(maxG * 0.00305176, 1);
    lcd.print('G');
}

void changeFreqLCD(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Change Freq to Ch: "));
  lcd.print(chnl);
  lcd.setCursor(0,1);
  lcd.print(F("Frequency: "));
  lcd.print(radioFreq[chnl], 3);
  lastRX = micros();
  delay(1000);
}
void parseCoord(float coord){
    dtostrf(coord, 2, 4, dataString);
    strPosn=0;
    while(dataString[strPosn]!='.'){strPosn++;}
    byte endPosn = strPosn+1;
    while(dataString[endPosn]!='\0'){endPosn++;}
    while(endPosn >= strPosn - 2){dataString[endPosn+1]=dataString[endPosn]; endPosn--;}
    dataString[strPosn-2]=(char)223;}

void displayAlt(int val){
  lcd.print(val * unitConvert, 0);
  if(displayStandard){lcd.print("ft");}
  else{lcd.print('m');}
}

void displayVel(int val){
  lcd.print(val * unitConvert, 0);
  if(displayStandard){lcd.print("fps ");}
  else{lcd.print("m/s ");}
}

void displayLat(){
  parseCoord(lastGPSlat);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlat);
}

void displayLon(){
  parseCoord(lastGPSlon);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlon);
}
/*void liteLED(byte pin){
  //analogWrite(pin, 255);
  liteStart = micros();
  liteUp = true;}*/
