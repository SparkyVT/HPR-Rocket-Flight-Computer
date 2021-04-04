//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//checkEvents(): this is the primary event logic.  It controls the firing of the output pyros and most of the event booleans.
//This logic is commonly referenced so it is broken out for convenience
//----------------------------

void checkEvents(){

//Check for timeout
    if (!events.timeOut && !pyroFire && fltTime.timeCurrent > settings.rcdTime) {
      events.timeOut = true;
      events.postFlight = true;
      events.inFlight = false;
      radio.event = 26;
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}
      radioInterval = RIpostFlight;
      RIsyncOffset = 600000UL;
      lastTX = micros();
      fltTime.touchdown = fltTime.timeCurrent;
      touchdownHour = GPS.time.hour();
      touchdownMin = GPS.time.minute();
      touchdownSec = GPS.time.second();
      touchdownMili = GPS.time.centisecond();}

    //Check false trigger until the flight time has passed the minimum time
    if (events.falseLiftoffCheck) {
      if (fltTime.timeCurrent > settings.detectLiftoffTime) {events.falseLiftoffCheck = false;}
      if (accel.z < settings.gTrigger && accelVel < 15.5F) {
        if(settings.testMode){Serial.println("False Trigger Reset");}
        //reset the key triggers
        events = resetEvents;
        fltTime.timeCurrent = 0UL;
        fltTime.gyro = 0UL;
        radio.event = 0;
        pktPosn = 0;
        radio.packetnum = 0;
        baroApogee = 0;
        baroApogeePosn = 0;
        for(byte i=0;i<5;i++){baroLast5[i]=0;}
        highGsmooth = 0;
        highGsum = 0;
        for(byte i=0; i<10; i++){highGfilter[i]=0;}
        baroTouchdown = 0;
        accelVel = 0;
        accelAlt = 0;
        radio.packetnum = 0;
        sampNum = 0;
        baroVelPosn = 10;
        radioInterval = RIpreLiftoff;}
    }//end falseLiftoffCheck

    //check for booster burnout: if the z acceleration is negative
    if (!events.boosterBurnout && events.liftoff && accel.z <= 0) {
      events.boosterBurnout = true;
      radio.event = 2;
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}
      events.boosterBurnoutCheck = true;
      fltTime.boosterBurnout = fltTime.timeCurrent;}
      
    //check for booster motor burp for 1 second after burnout is detected
    if (events.boosterBurnoutCheck){
      if(fltTime.timeCurrent - fltTime.boosterBurnout > boosterBurpTime){events.boosterBurnoutCheck = false;}
      else if (events.boosterBurnout && !settings.testMode && accel.z > 0){events.boosterBurnout = false; events.boosterBurnoutCheck = false; radio.event = 1;}}

    //2-Stage Flight Profile
    if(settings.fltProfile == '2'){

      //Fire separation charge if burnout is detected and time is past the separation delay
      if (!events.boosterSeparation &&  events.liftoff && events.boosterBurnout && !events.falseLiftoffCheck && fltTime.timeCurrent - fltTime.boosterBurnout > settings.boosterSeparationDelay) {
      events.boosterSeparation = true;
      fltTime.boosterSeparation = fltTime.timeCurrent;
      firePyros('B');
      radio.event = 8;
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}

      //Fire second stage
      if (!events.sustainerFireCheck && (!events.apogee || settings.testMode) && events.liftoff && events.boosterBurnout && events.boosterSeparation && !pyroFire && fltTime.timeCurrent - fltTime.boosterSeparation > settings.sustainerFireDelay) {
        events.sustainerFireCheck = true;
        postFlightCode = 1;
        fltTime.sustainerFireCheck = fltTime.timeCurrent;
        //Check for staging inhibit and fire if OK
        if (altOK && rotnOK) {
          events.sustainerFire = true;
          firePyros('I');
          radio.event = 9;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK && !altOK){postFlightCode = 4; radio.event = 20;}
        else if (!rotnOK) {postFlightCode = 3; radio.event = 18;}
        else if (!altOK) {postFlightCode = 2; radio.event = 19;}}
  
      // Check for sustainer ignition
      if(!events.apogee && !events.sustainerIgnition && events.sustainerFire && accelNow > 10.0){
        radio.event = 10; 
        if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}
        events.sustainerIgnition = true; 
        fltTime.sustainerIgnition = fltTime.timeCurrent;}
      
      //Check for sustainer burnout
      if(!events.apogee && !events.sustainerBurnout && events.sustainerIgnition && accelNow < 0.0 && fltTime.timeCurrent - fltTime.sustainerIgnition > 100000UL){
        radio.event = 11; 
        if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}
        events.sustainerBurnout = true; 
        fltTime.sustainerBurnout = fltTime.timeCurrent;}
      
    }//end 2-stage profile

    //Airstart Flight Profile
    if(settings.fltProfile == 'A'){

      //AirStart motor 1 if event 1 is main booster ignition
      if(settings.airStart1Event == 'I' && !events.airStart1Check && !events.falseLiftoffCheck && fltTime.timeCurrent > settings.airStart1Delay){
        events.airStart1Check = true;
        fltTime.airStart1Check = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for inflight ignition inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart1Fire = true;
          fltTime.airStart1Fire = fltTime.timeCurrent;
          firePyros('1');
          radio.event = 12;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK && !altOK){
          postFlightCode = 4; 
          radio.event = 20;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK) {
          postFlightCode = 3; 
          radio.event = 18;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!altOK) {
          postFlightCode = 2; 
          radio.event = 19;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}}
      
      //AirStart motor 1 if event 1 is main booster burnout
      if(settings.airStart1Event == 'B' && events.boosterBurnout && !events.airStart1Check && fltTime.timeCurrent > (fltTime.boosterBurnout + settings.airStart1Delay)){
        events.airStart1Check = true;
        fltTime.airStart1Check = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for inflight ignition inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart1Fire = true;
          fltTime.airStart1Fire = fltTime.timeCurrent;
          firePyros('1');
          radio.event = 12;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK && !altOK){
          postFlightCode = 4; 
          radio.event = 20;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK) {
          postFlightCode = 3; 
          radio.event = 18;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!altOK) {
          postFlightCode = 2; 
          radio.event = 19;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}}

      //Look for AirStart 1 Ignition
      if(events.airStart1Fire && !events.apogee && !events.airStart1Ignition && accelNow > 10.0){
        events.airStart1Ignition = true; 
        fltTime.airStart1Ignition = fltTime.timeCurrent; 
        radio.event = 13;
        if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}

      //Look for AirStart 1 Burnout with a check for a motor burp
      if(!events.airStart1Burnout && events.airStart1Ignition){
        if(!events.airStart1BurnoutCheck && !events.apogee && !events.airStart1Burnout && accelNow < 0.0){events.airStart1BurnoutCheck = true; fltTime.airStart1Burnout = fltTime.timeCurrent;}
        if(events.airStart1BurnoutCheck && fltTime.timeCurrent < (fltTime.airStart1Burnout + boosterBurpTime) && accel.z > 0){events.airStart1BurnoutCheck = false;}
        if(events.airStart1BurnoutCheck && fltTime.timeCurrent > (fltTime.airStart1Burnout + boosterBurpTime)){
          events.airStart1BurnoutCheck = false; events.airStart1Burnout = true; 
          radio.event = 14;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}}
      
      //AirStart motor 2 if event 2 is airstart1 motor ignition
      if(settings.airStart2Event == 'I' && !events.apogee && !events.airStart2Check && events.airStart1Ignition && fltTime.timeCurrent > fltTime.airStart1Ignition + settings.airStart2Delay){
        events.airStart2Check = true;
        fltTime.airStart2Check = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for inflight ignition inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart2Fire = true;
          fltTime.airStart2Fire = fltTime.timeCurrent;
          firePyros('2');
          radio.event = 15;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK && !altOK){
          postFlightCode = 4; 
          radio.event = 20;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK) {
          postFlightCode = 3; 
          radio.event = 18;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!altOK) {
          postFlightCode = 2; 
          radio.event = 19;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}}
        
      //Airstart motor 2 if event 2 is airstart1 motor burnout
      if(settings.airStart2Event == 'B' && !events.apogee && !events.airStart2Check && events.airStart1Burnout && fltTime.timeCurrent > (fltTime.airStart1Burnout + settings.airStart2Delay)){
        events.airStart2Check = true;
        fltTime.airStart2Check = fltTime.timeCurrent;
        postFlightCode = 1;
        //Check for inflight ignition inhibit and fire if OK
        if (altOK && rotnOK) {
          events.airStart2Fire = true;
          fltTime.airStart2Fire = fltTime.timeCurrent;
          firePyros('2');
          radio.event = 15;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK && !altOK){
          postFlightCode = 4; 
          radio.event = 20;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!rotnOK) {
          postFlightCode = 3; 
          radio.event = 18;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
        else if (!altOK) {
          postFlightCode = 2; 
          radio.event = 19;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}}

      //Look for AirStart 2 Ignition
      if(events.airStart2Fire && !events.apogee && !events.airStart2Ignition && accelNow > 10.0){
        events.airStart2Ignition = true; 
        fltTime.airStart2Ignition = fltTime.timeCurrent; 
        radio.event = 16;
        if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}

      //Look for AirStart 2 Burnout
      if(!events.airStart2Burnout && events.airStart2Ignition){
        if(!events.airStart2BurnoutCheck && !events.apogee && !events.airStart2Burnout && accelNow < 0.0){events.airStart2BurnoutCheck = true; fltTime.airStart2Burnout = fltTime.timeCurrent;}
        if(events.airStart2BurnoutCheck && fltTime.timeCurrent < (fltTime.airStart2Burnout + boosterBurpTime) && accel.z > 0){events.airStart2BurnoutCheck = false;}
        if(events.airStart2BurnoutCheck && fltTime.timeCurrent > (fltTime.airStart2Burnout + boosterBurpTime)){
          events.airStart2BurnoutCheck = false; 
          events.airStart2Burnout = true; 
          radio.event = 17;
          if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}}
    }//End Airstart Flight Mode

    //Check for apogee if the accelerometer velocity or barometric velocity < 0
    //Above 9000 meters, only use accelerometer velocity for apogee trigger
    if (!events.apogee && events.boosterBurnout && !events.boosterBurnoutCheck && !pyroFire && (accelVel < 0 || (baroVel < 0 && accelVel < 70 && (Alt + baseAlt) < 9000) || fusionVel < 0)) {
      events.apogee = true;
      fltTime.apogee = fltTime.timeCurrent;
      radio.event = 3;
      if(settings.fltProfile == 'B'){radio.event = 21;}
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
      
    //Fire apgogee charge if the current time > apogeeTime + apogeeDelay
    if (!events.apogeeFire && events.apogee && fltTime.timeCurrent - fltTime.apogee >= settings.apogeeDelay) {
      events.apogeeFire = true;
      fltTime.apogeeFire = fltTime.timeCurrent;
      firePyros('A');
      radio.event = 4;
      if(settings.fltProfile == 'B'){radio.event = 22;}
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}
      
    //Write the data to the card 3s after apogeeFire in case of crash or powerloss
    if(events.apogeeFire && !syncApogee && !settings.testMode && fltTime.timeCurrent - fltTime.apogeeFire >= 3000000UL){outputFile.sync();syncApogee = true;}

    //Detect separation after apogee
    if(events.apogeeFire && !events.mainDeploy && accel.z > 4*g && fltTime.timeCurrent - fltTime.apogeeFire <= 2000000UL){
      events.apogeeSeparation = true; 
      fltTime.apogeeSeparation = fltTime.timeCurrent; 
      radio.event = 5;
      if(settings.fltProfile == 'B'){radio.event = 23;}
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}

    //Fire main chute charge if the baro altitude is lower than the threshold and at least 1s has passed since apogee
    if (!events.mainDeploy && events.apogeeFire && Alt < settings.mainDeployAlt && fltTime.timeCurrent - fltTime.apogeeFire >= 1000000UL) {
      events.mainDeploy = true;
      fltTime.mainDeploy = fltTime.timeCurrent;
      firePyros('M');
      radio.event = 6;
      if(settings.fltProfile == 'B'){radio.event = 24;}
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}

    //Detect deployment of the mains
    if(events.mainDeploy && fltTime.timeCurrent - fltTime.mainDeploy > 50000UL && fltTime.timeCurrent - fltTime.mainDeploy < 3000000UL && accelNow > 50.0){
        radio.event = 7; 
        if(settings.fltProfile == 'B'){radio.event = 25;}
        if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}}

    //Write the data to the card 3s after mainDeploy in case of crash or powerloss
    if(events.mainDeploy && !syncMains && !settings.testMode && fltTime.timeCurrent - fltTime.mainDeploy >= 3000000UL){outputFile.sync();syncMains = true;}
    
    //Turn off the pyros after the allotted time
    if (pyroFire) {
      //Check if pseudo-PWM is required
      if(sensors.pyroPWM){pulsePyro();}
      //Turn off the pyros
      if(pyro1.fireStatus && fltTime.timeCurrent - pyro1.fireStart > settings.fireTime){digitalWrite(pyro1.firePin, LOW);pyro1.fireStatus = false;}
      if(pyro2.fireStatus && fltTime.timeCurrent - pyro2.fireStart > settings.fireTime){digitalWrite(pyro2.firePin, LOW);pyro2.fireStatus = false;}
      if(pyro3.fireStatus && fltTime.timeCurrent - pyro3.fireStart > settings.fireTime){digitalWrite(pyro3.firePin, LOW);pyro3.fireStatus = false;}
      if(pyro4.fireStatus && fltTime.timeCurrent - pyro4.fireStart > settings.fireTime){digitalWrite(pyro4.firePin, LOW);pyro4.fireStatus = false;}
      if(!pyro1.fireStatus && !pyro2.fireStatus && !pyro3.fireStatus && !pyro4.fireStatus){pyroFire = false;}}

    //Check for touchdown
    if (!events.touchdown && events.mainDeploy && !pyroFire && !settings.testMode && baroTouchdown > touchdownTrigger && Alt < 46) {
      events.touchdown = true;
      events.inFlight = false;
      events.postFlight = true;
      fltTime.touchdown = fltTime.timeCurrent;
      radioInterval = RIpostFlight;
      RIsyncOffset = 600000UL;
      radio.event = 27;
      if(settings.inflightRecover != 0){EEPROM.update(eeprom.lastEvent, radio.event);}
      touchdownHour = GPS.time.hour();
      touchdownMin = GPS.time.minute();
      touchdownSec = GPS.time.second();
      touchdownMili = GPS.time.centisecond();}
 }//end check events
