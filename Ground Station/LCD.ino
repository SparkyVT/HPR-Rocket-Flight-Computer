
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
  //Header - Line 3
  //----------------------------------------
  lcd.print('\r');
  lcd.print("GPS Location:");
  //----------------------------------------
  //GPS Latitude - Line 4
  //----------------------------------------
  lcd.print('\r');
  if(isnan(lastGPSlat) || lastGPSlon == 0.0){lastGPSlon = 12.34567;lastGPSlat = 12.34567;}
  dtostrf(lastGPSlat,2,5,dataString);
  lcd.print(dataString);
  lcd.print(",");
  dtostrf(lastGPSlon,3,5,dataString);
  lcd.print(dataString);}

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
  myString += "ft";
  for(byte i = myString.length()+1; i < 81; i++){myString += " ";}
  if(GPS.location.isUpdated()){myString.setCharAt(79, 'G');}
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
  //signal strength, bearing, distance, and GPS - line 4
  //----------------------------------------
  myString += String((int)signalStrength);
  myString += "dBm ";
  //display GNSS data
  if(GPSlock == 1){
    float bearing = TinyGPSPlus::courseTo(GPS.location.lat(), GPS.location.lng(), lastGPSlat, lastGPSlon);
    myString += String((int)bearing);
    myString += (char)39;
    myString += " ";
    float rktDist = TinyGPSPlus::distanceBetween(GPS.location.lat(), GPS.location.lng(), lastGPSlat, lastGPSlon);
    rktDist /= 1609.34;//convert to miles
    if(rktDist >99){rktDist = 99.9;}
    if(rktDist < 0.2){
      myString += String((int)(rktDist * 5280));
      myString += "ft";}
    else{ 
      myString += String((int)rktDist);
      myString += ".";
      myString += String((int)((rktDist - (int)rktDist)*10));
      myString += "mi";}}
  //no GNSS data
  else{
    for(byte i = myString.length()+1; i <70; i++){myString += " ";}
    myString += "No GPS Data";}
  for(byte i = myString.length()+1; i < 81; i++){myString += " ";}
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
  //GPS Coordinates - Line 3
  //----------------------------------------
  if(isnan(lastGPSlat) || lastGPSlat == 0.0){lastGPSlat = lastGPSlon = 12.34567;}
  dtostrf(lastGPSlat, 2, 5, dataString);
  myString += dataString;
  myString.trim();
  myString += ",";
  dtostrf(lastGPSlon, 2, 5, dataString);
  myString += dataString;
  myString.trim();
  for(byte i = myString.length()+1; i < 61; i++){myString += " ";}
  //----------------------------------------
  //Bearing and distance - Line 4
  //----------------------------------------
  if(GPS.location.isUpdated()){
    //bearing
    float bearing = TinyGPSPlus::courseTo(GPS.location.lat(), GPS.location.lng(), lastGPSlat, lastGPSlon);
    myString += String((int)bearing);
    myString += (char)39;
    myString += " ";
    //distance
    float dist2rkt = TinyGPSPlus::distanceBetween(GPS.location.lat(), GPS.location.lng(), lastGPSlat, lastGPSlon);
    dist2rkt /= 1609.34;//convert to miles
    if(dist2rkt > 99){dist2rkt = 99.9;}
    //convert to feet if below 0.2 miles
    if(dist2rkt < 0.2){
      dist2rkt *= 5280;
      myString += String((int)dist2rkt);
      myString += "ft";}
    //display miles
    else{
      dtostrf(dist2rkt, 2, 1, dataString);
      myString += dataString;
      myString += " mi";}}
  else{myString += "Receiver No GPS";}
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
    //GPS Coordinates - Line 2
    //----------------------------------------
    if(isnan(lastGPSlat) || lastGPSlat == 0.0){lastGPSlat = lastGPSlon = 12.34567;}
    dtostrf(lastGPSlat, 2, 5, dataString);
    myString += dataString;
    myString.trim();
    myString += ",";
    dtostrf(lastGPSlon, 2, 5, dataString);
    myString += dataString;
    myString.trim();
    for(byte i = myString.length()+1; i < 41; i++){myString += " ";}
    //----------------------------------------
    //GPS Distance & Bearing to Rocket
    //----------------------------------------
    if(GPS.location.isUpdated()){
      myString += "Dist: ";
      float dist2rkt = TinyGPSPlus::distanceBetween(GPS.location.lat(), GPS.location.lng(), lastGPSlat, lastGPSlon);
      dist2rkt /= 1609.34;//convert to miles
      if(dist2rkt > 99){dist2rkt = 99.9;}
      //convert to feet if below 0.2 miles
      if(dist2rkt < 0.2){
        dist2rkt *= 5280;
        myString += String((int)dist2rkt);
        myString += "ft, ";}
      //display miles
      else{
        dtostrf(dist2rkt, 2, 1, dataString);
        myString += dataString;
        myString += "mi, ";}
      //bearing
      float bearing = TinyGPSPlus::courseTo(GPS.location.lat(), GPS.location.lng(), lastGPSlat, lastGPSlon);
      myString += String((int)bearing);
      myString += (char)39;}
    else{myString += "Receiver No GPS";}
    for(byte i = myString.length()+1; i < 61; i++){myString += " ";}
    //----------------------------------------
    //Max Values - Line 4
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