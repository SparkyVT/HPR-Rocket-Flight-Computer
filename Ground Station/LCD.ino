
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
  delay(2500);
  
  //Print to the LCD the last good coordinates from the previous flight
  
  lcd.clear();
  //----------------------------------------
  //Message - Line 1    
  //----------------------------------------
  lcd.setCursor(0,0);
  lcd.print(F("Last Packet From:"));
  //----------------------------------------
  //Rocket Name - Line 2
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print(rocketName);
  //----------------------------------------
  //GPS Latitude - Line 3
  //----------------------------------------
  lcd.setCursor(0,2);
  //lcd.print(lastGPSlat,4);
  parseCoord(lastGPSlat);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlat);
  //----------------------------------------
  //GPS Latitude - Line 4
  //----------------------------------------
  lcd.setCursor(0,3);
  //lcd.print(lastGPSlon,4);
  parseCoord(lastGPSlon);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlon);}

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
  if(contCode == 4){lcd.print(F("All 4 Pyros Detected"));}
  else if (contCode == 6){lcd.print(F("Pyro Apogee Only"));}
  else if (contCode == 7){lcd.print(F("Pyro Mains Only"));}
  else if (contCode == 8){lcd.print(F("Pyro Mains & Apogee"));}
  else if (contCode == 9){lcd.print(F("No Pyros Detected!"));}
  else{lcd.print(F("No Cont Pyro "));
    if(contCode == 5){lcd.print(F("4"));}
    else{lcd.print(contCode);}}
  //line 2
  lcd.setCursor(0,2);
  if(GPSlock == 1){lcd.print(F("GPS Lock OK!"));}
  else{lcd.print(F("Awaiting GPS Lock"));}
  //line 3
  lcd.setCursor(0,3);
  lcd.print(F("Base Alt: "));
  lcd.print((long)(baseAlt*3.2808));
  lcd.print(F(" ft"));

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
  switch (event){
    case 1:
      lcd.print(F("Liftoff!"));
      litePin = GREENLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 2:
      lcd.print(F("Booster Burnout!"));
      break;
    case 3:
      lcd.print(F("Booster Separation!"));
      break;
    case 4:
      lcd.print(F("Firing 2nd Stage!"));
      litePin = GREENLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 5:
      lcd.print(F("Apogee Detected!"));
      break;
    case 6:
      lcd.print(F("Separation Detected!"));
      litePin = GREENLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 7:
      lcd.print(F("Mains Deployed!"));
      litePin = GREENLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 10:
      lcd.print(F("Rotation Exceeded"));
      litePin = REDLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 11:
      lcd.print(F("Below Alt Threshold"));
      litePin = REDLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 12:
      lcd.print(F("Rotn / Alt Limits"));
      litePin = REDLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    case 13:
      lcd.print(F("2nd Stage Ignition!"));
      break;
    case 14:
      lcd.print(F("2nd Stage Burnout!"));
      break;
    case 15:
      lcd.print(F("Under Chute!"));
      litePin = GREENLITE;
      if(!liteUp){liteLED(litePin);}
      break;
    default: 
      lcd.print(F("Event Error: "));lcd.print(event);}
  //----------------------------------------
  //display altitude - line 1
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print(F("Altitude: "));
  lcd.print((long)(Alt*3.2808));
  lcd.print(F(" ft"));
  //----------------------------------------
  //display velocity - line 2
  //----------------------------------------
  lcd.setCursor(0,2);
  lcd.print(F("Speed: "));
  lcd.print((long)(velocity*3.2808));
  lcd.print(F(" fps"));
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
  
 //Postflight Code
 //-----------------------------------------------------------

  //----------------------------------------
  //barometric altitude - line 0
  //----------------------------------------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Max Alt: "));
  lcd.print((long)(maxAltitude*3.2808));
  lcd.print(F(" ft"));

  //----------------------------------------
  //integrated velocity - line 1
  //----------------------------------------
  lcd.setCursor(0,1);
  lcd.print(F("Max Speed: "));
  lcd.print((long)(maxVelocity*3.2808));
  lcd.print(F(" fps"));

  //----------------------------------------
  //GPS Latitude - Line 2
  //----------------------------------------
  lcd.setCursor(0,2);
  //lcd.print(GPSlatitude,4);
  parseCoord(GPSlatitude);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlat);
  
  //----------------------------------------
  //GPS Latitude - Line 2
  //----------------------------------------
  lcd.setCursor(0,3);
  //lcd.print(GPSlongitude,4);
  parseCoord(GPSlongitude);
  lcd.print(dataString);
  lcd.print((char)39);
  lcd.print(charGPSlon);

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
    parseCoord(lastGPSlat);
    lcd.print(dataString);
    lcd.print((char)39);
    lcd.print(charGPSlat);
  
    //----------------------------------------
    //GPS Latitude - Line 2
    //----------------------------------------
    lcd.setCursor(0,2);
    parseCoord(lastGPSlon);
    lcd.print(dataString);
    lcd.print((char)39);
    lcd.print(charGPSlon);
}

void parseCoord(float coord){
    dtostrf(coord, 2, 4, dataString);
    strPosn=0;
    while(dataString[strPosn]!='.'){strPosn++;}
    byte endPosn = strPosn+1;
    while(dataString[endPosn]!='\0'){endPosn++;}
    while(endPosn >= strPosn - 2){dataString[endPosn+1]=dataString[endPosn]; endPosn--;}
    dataString[strPosn-2]=(char)223;}

void liteLED(byte pin){
  analogWrite(pin, 255);
  liteStart = micros();
  liteUp = true;}
