//----------------------------
//LIST OF FUNCTIONS & ROUTINES
//----------------------------
//getQuatRotn(): quaternion rotation at 100Hz using the gyroscope only
//magRotn(): roll and heading calculated from the magnetometer
//getRotnDCM2D(): Optimized DCM code to calculate the rotation on each cycle of the loop.  Used for active stabilization
//setCanards(): control code for active stabilization using canards
//-----------CHANGE LOG------------
//17 JUL 21: initial breakout created
//02 AUG 21: revision of PID control system, fixed bugs in DCM2D
//---------------------------------

void getQuatRotn(float dx, float dy, float dz){

//Local Vectors
float QuatDiff[5];
float Rotn1[4];
float Rotn2[4];
float Rotn3[4];

//Local rotation holders
static long prevRollZ = 0;
static long quatRollZ = 0;
static long fullRollZ = 0;

//Compute quaternion derivative
QuatDiff[1] = 0.5 * (-1 * dx * Quat[2] - dy * Quat[3] - dz * Quat[4]);
QuatDiff[2] = 0.5 * (     dx * Quat[1] - dy * Quat[4] + dz * Quat[3]);
QuatDiff[3] = 0.5 * (     dx * Quat[4] + dy * Quat[1] - dz * Quat[2]);
QuatDiff[4] = 0.5 * (-1 * dx * Quat[3] + dy * Quat[2] + dz * Quat[1]);

//Update the quaternion
Quat[1] += QuatDiff[1];
Quat[2] += QuatDiff[2];
Quat[3] += QuatDiff[3];
Quat[4] += QuatDiff[4];

//re-normalize
float quatLen = powf( Quat[1]*Quat[1] + Quat[2]*Quat[2] + Quat[3]*Quat[3] + Quat[4]*Quat[4], -0.5);
Quat[1] *= quatLen;
Quat[2] *= quatLen;
Quat[3] *= quatLen;
Quat[4] *= quatLen;
    
//compute the components of the rotation matrix
float a = Quat[1];
float b = Quat[2];
float c = Quat[3];
float d = Quat[4];
float a2 = a*a;
float b2 = b*b;
float c2 = c*c;
float d2 = d*d;
float ab = a*b;
float ac = a*c;
float ad = a*d;
float bc = b*c;
float bd = b*d;
float cd = c*d;
    
//Compute rotation matrix
Rotn1[1] = a2 + b2 - c2 - d2;
//Rotn1[2] = 2 * (bc - ad);
//Rotn1[3] = 2 * (bd + ac);
Rotn2[1] = 2 * (bc + ad);
//Rotn2[2] = a2 - b2 + c2 - d2;
//Rotn2[3] = 2 * (cd - ab);
Rotn3[1] = 2 * (bd - ac);
Rotn3[2] = 2 * (cd + ab);
Rotn3[3] = a2 - b2 - c2 + d2;

//compute 3D orientation
pitchX = speedAtan2(Rotn3[2], Rotn3[3]);
yawY = speedArcSin(-1*Rotn3[1]);

prevRollZ = quatRollZ;
quatRollZ = speedAtan2(Rotn2[1], Rotn1[1]);
if(quatRollZ - prevRollZ > 1800){fullRollZ--;}
else if(quatRollZ - prevRollZ < -1800){fullRollZ++;}
rollZ = (fullRollZ*3600 + quatRollZ)*.1;

//Compute angle off vertical
float tanYaw = speedTan(yawY);
float tanPitch = speedTan(pitchX);

float hyp1 = tanYaw*tanYaw + tanPitch*tanPitch;
float hyp2 = powf(hyp1, 0.5);
offVert = speedArcTan(hyp2);

//check to see if the maximum angle has been exceeded
if (!rotationFault && offVert > settings.maxAngle) {rotnOK = false;}
if (!rotationFault && offVert > 450){rotationFault = true; rotnOK = false;}
}//end getQuatRotn

//made global since these variables are also used by setCanards
float cosZ = 1.0F;
float sinZ = 0.0F;
  
void getDCM2DRotn(long dx, long dy, long dz){

  static float rawX = 0.0F;
  static float rawY = 0.0F;
  static float rawZ = 0.0F;
  float deg2rad = 3.141592653589793238462 / 180;
  float rad2deg = 1/deg2rad;
  
  //Calculate new Z angle from gyro data
  rawZ += dz;
  float radNewRoll = dz * gyro.gainZ * deg2rad  * mlnth;
  
  //compute sin(roll) and cos(roll)
  float sinNewRoll = sinSmallAngle(radNewRoll);
  float cosNewRoll = cosSmallAngle(radNewRoll);
  float prevCosZ = cosZ;
  float prevSinZ = sinZ;
  cosZ = prevCosZ * cosNewRoll - prevSinZ * sinNewRoll;
  sinZ = prevSinZ * cosNewRoll + prevCosZ * sinNewRoll;
  
  //Compute the new y and x angles
  rawX += (cosZ * dx - sinZ * dy);
  rawY += (cosZ * dy + sinZ * dx);

  //Compute Pitch, Roll, Yaw
  rollZ = (long)((rawZ * gyro.gainZ) * mlnth);
  pitchX = (int)((rawX * gyro.gainX) * mlnth * 10);
  yawY =   (int)((rawY * gyro.gainY) * mlnth * 10);

  //Compute off-vertical
  float tanYaw = speedTan(yawY);
  tanYaw *= tanYaw;
  float tanPitch = speedTan(pitchX);
  tanPitch *= tanPitch;
  offVert = speedArcTan(powf(tanYaw + tanPitch, 0.5));

  //Check if the max angle is exceeded, shutdown staging if angle > 45 is detected
  if ((!events.apogee) || settings.testMode){
      
    //check to see if the maximum angle has been exceeded
    if (!rotationFault && offVert > settings.maxAngle) {rotnOK = false;}
    if (!rotationFault && offVert > 450){rotationFault = true; rotnOK = false;}}
}//end void

float rollError;
float pitchError;
float yawError;
float rollPosn;

void setCanards(){

  const float KpYawPitch = 0.03;
  const float KiYawPitch = 0.0;
  const float KdYawPitch = 0.0;
  const float KpRoll = 0.0015;
  const float KiRoll = 0.0;
  const float KdRoll = 0.0005;
  const float rollSetPoint = 0.0F;
  const float pitchSetPoint = 0.0F;
  const float yawSetPoint = 0.0F;
  uint32_t cdt;
  static uint32_t lastUpdate = 0UL;
  static float rollErrorInt = 0.0F;
  static float pitchErrorInt = 0.0F;
  static float yawErrorInt = 0.0F;
  float sineRollThrowServo1;
  float sineRollThrowServo2;
  float sineRollThrowServo3;
  float sineRollThrowServo4;
  float sineYawThrowServo1;
  float sineYawThrowServo2;
  float sinePitchThrowServo3;
  float sinePitchThrowServo4;

  float rollTorqueInput;
  float pitchTorqueInput;
  float yawTorqueInput;

  const float rollTorqueArm = 0.046482;//1.83in = 0.0464m
  const float yawPitchTorqueArm =  0.46482;//18in = 0.464m
  float maxTorqueInput = 4.0;

  //Update time stamps
  uint32_t timeNow = micros();
  cdt = timeNow - lastUpdate;
  lastUpdate = timeNow;

  //Determine current position error
  rollError = rollZ%360 - rollSetPoint;
  pitchError = pitchX * 0.1 - pitchSetPoint;//pitch is reported in tenths of a degree
  yawError = yawY * 0.1 - yawSetPoint;//yaw is reported in tenths of a degree

  //Calculate air density
  const float R = 287.05; //R value for dry air
  float airDensity = (pressure * 100) / (R * (temperature + 273.15));

  //Calculate the fin drag force
  const float Cd = 0.5; //estimate of the fin Cd
  const float finArea = 0.00064516;//area of each canard in m^2
  float currentVel = fusionVel;
  if(fabs(currentVel)<1.0F){currentVel = 1.0F;}
  if(settings.testMode){currentVel = 100.0F;}
  float dragForce = 0.5 * Cd * currentVel*currentVel * finArea * airDensity;

  //-----------------------
  //Update roll correction
  //-----------------------
  if(settings.stableRotn){

    //Update Integral Terms
    rollErrorInt += rollError*cdt*mlnth;

    //Prevent Integral Wind-Up in bench-test mode
    if(settings.testMode){rollErrorInt = 0;}

    //Update roll position correction
    rollTorqueInput = KpRoll*rollError + KiRoll*rollErrorInt + KdRoll*gyro.z*gyro.gainZ;
    if(fabs(rollTorqueInput)>maxTorqueInput){rollTorqueInput = ((rollTorqueInput>0)? 1 : -1) * maxTorqueInput;}
    
    //Determine the required torque to correct for roll
    const float numRollFins = 4.0F;
    float rollTorqueServo1 = rollTorqueInput / numRollFins;
    float rollTorqueServo2 = rollTorqueInput / numRollFins;
    float rollTorqueServo3 = rollTorqueInput / numRollFins;
    float rollTorqueServo4 = rollTorqueInput / numRollFins;
    
    //---------------------------------------------
    //Calculate the sine of the required throw angle
    //---------------------------------------------
    //torque = controlForce * torqueArmDistance
    //controlForce = sin(canardThrow) * finDragForce
    //finDragForce = 0.5 * Cd * velocity^2 * finArea * airDensity
    //canardThrow = asin(controlForce / finDragForce)
    sineRollThrowServo1 = rollTorqueServo1 / (rollTorqueArm * dragForce);
    sineRollThrowServo2 = rollTorqueServo2 / (rollTorqueArm * dragForce);
    sineRollThrowServo3 = rollTorqueServo3 / (rollTorqueArm * dragForce);
    sineRollThrowServo4 = rollTorqueServo4 / (rollTorqueArm * dragForce);}
    
  else{sineRollThrowServo1 = sineRollThrowServo2 = sineRollThrowServo3 = sineRollThrowServo4 = 0.0F;}

  //--------------------------------------------------------------------------------------------------------------
  //Update off-vertical error terms, restrict input to only when the roll is close to the original starting point
  //--------------------------------------------------------------------------------------------------------------
  if(settings.stableVert && fabs(rollError) < 30){
    
    //Update integral terms
    pitchErrorInt += pitchError*cdt*mlnth;
    yawErrorInt   +=   yawError*cdt*mlnth;

    //Prevent Integral Wind-Up in bench-test mode
    if(settings.testMode){pitchErrorInt = yawErrorInt = 0;}

    //Update pitch & yaw correction positions, sinZ & cosZ are the sine and cosine of the roll angle
    pitchTorqueInput = KpYawPitch*pitchError + KiYawPitch*pitchErrorInt + KdYawPitch*(cosZ * gyro.gainX * gyro.x + sinZ * gyro.gainY * gyro.y);
    yawTorqueInput   = KpYawPitch*yawError   + KiYawPitch*yawErrorInt   + KdYawPitch*(cosZ * gyro.gainY * gyro.y - sinZ * gyro.gainX * gyro.x);
    if(fabs(pitchTorqueInput)>maxTorqueInput){pitchTorqueInput = ((rollTorqueInput>0)? 1 : -1) * maxTorqueInput;}
    if(fabs(yawTorqueInput)  >maxTorqueInput){yawTorqueInput   = ((rollTorqueInput>0)? 1 : -1) * maxTorqueInput;}
    
    //Determine the required torque to correct for pitch and yaw
    const float numYawPitchFins = 2.0F;
    float yawTorqueServo1   = (sinZ*pitchTorqueInput + cosZ*yawTorqueInput)   / numYawPitchFins;
    float yawTorqueServo2   = (sinZ*pitchTorqueInput + cosZ*yawTorqueInput)   / numYawPitchFins;
    float pitchTorqueServo3 = (sinZ*yawTorqueInput   + cosZ*pitchTorqueInput) / numYawPitchFins;
    float pitchTorqueServo4 = (sinZ*yawTorqueInput   + cosZ*pitchTorqueInput) / numYawPitchFins;
    
    //---------------------------------------------
    //Calculate the sine of the required throw angle
    //---------------------------------------------
    //torque = controlForce * torqueArmDistance
    //controlForce = sin(canardThrow)
    //finDragForce = 0.5 * Cd * velocity^2 * finArea * airDensity
    //canardThrow = asin(controlForce / finDragForce)
    sineYawThrowServo1   = (yawTorqueServo1   / yawPitchTorqueArm) / dragForce;
    sineYawThrowServo2   = (yawTorqueServo2   / yawPitchTorqueArm) / dragForce;
    sinePitchThrowServo3 = (pitchTorqueServo3 / yawPitchTorqueArm) / dragForce;
    sinePitchThrowServo4 = (pitchTorqueServo4 / yawPitchTorqueArm) / dragForce;}
    
  else{sineYawThrowServo1 = sineYawThrowServo2 = sinePitchThrowServo3 = sinePitchThrowServo4 = 0.0F;}

  //Determine the sine of the total canard throw
  float sineThrowServo1 = sineRollThrowServo1 + sineYawThrowServo1;
  float sineThrowServo2 = sineRollThrowServo2 - sineYawThrowServo2;
  float sineThrowServo3 = sineRollThrowServo3 + sinePitchThrowServo3;
  float sineThrowServo4 = sineRollThrowServo4 - sinePitchThrowServo4;
  
  //control input corrections are limited to 45 degrees
  const float sine45 = 0.70710678;//sqrt(2)/2
  if(fabs(sineThrowServo1) > sine45){sineThrowServo1 = ((sineThrowServo1>0)? 1 : -1) * sine45;}
  if(fabs(sineThrowServo2) > sine45){sineThrowServo2 = ((sineThrowServo2>0)? 1 : -1) * sine45;}
  if(fabs(sineThrowServo3) > sine45){sineThrowServo3 = ((sineThrowServo3>0)? 1 : -1) * sine45;}
  if(fabs(sineThrowServo4) > sine45){sineThrowServo4 = ((sineThrowServo4>0)? 1 : -1) * sine45;}

  //translate to degrees
  const float rad2deg = 180 / M_PI;
  float throwServo1 = 90 + asinf(sineThrowServo1) * rad2deg;
  float throwServo2 = 90 + asinf(sineThrowServo2) * rad2deg;
  float throwServo3 = 90 + asinf(sineThrowServo3) * rad2deg;
  float throwServo4 = 90 + asinf(sineThrowServo4) * rad2deg;

  //Set the fin positions
  //fin pointed in the positive-y world-frame
  canardYaw1.write(throwServo1-servo1trim);
  //fin pointed in the negative-y world-frame
  canardYaw2.write(throwServo2-servo2trim);
  //fin pointed in the positive-x world-frame
  canardPitch3.write(throwServo3-servo3trim);
  //fin pointed in the negative-x world-frame
  canardPitch4.write(throwServo4-servo4trim);

  //output debugging over serial
  const boolean serialPlotter = true;
  if(settings.testMode && !serialPlotter){
    Serial.print("***Time: ");Serial.println(fltTime.timeCurrent*mlnth, 2);
    Serial.print("Roll, Pitch, Yaw: ");Serial.print(rollZ);Serial.print(", ");Serial.print(pitchX*.1, 2);Serial.print(", ");Serial.println(yawY*.1, 2);
    Serial.print("Error from set point: ");Serial.print(rollError);Serial.print(", ");Serial.print(pitchError);Serial.print(", ");Serial.println(yawError);
    Serial.print("Roll Input: ");Serial.println(rollTorqueInput, 2);
    Serial.print("Yaw Input: ");Serial.println(yawTorqueInput, 2);
    Serial.print("Pitch Input: ");Serial.println(pitchTorqueInput, 2);
    Serial.print("DragForce: ");Serial.println(dragForce, 2);
    Serial.print("Servo1: ");Serial.println(throwServo1);
    Serial.print("Servo2: ");Serial.println(throwServo2);
    Serial.print("Servo3: ");Serial.println(throwServo3);
    Serial.print("Servo4: ");Serial.println(throwServo4);}

  if(settings.testMode && serialPlotter){
    //Error from set point
    //Serial.print("RollError:");Serial.print(rollError);Serial.print("\t");
    //Serial.print("PitchError:");Serial.print(pitchError);Serial.print("\t");
    //Serial.print("YawError:");Serial.print(yawError);Serial.print("\t");
    Serial.print("RollInput:");Serial.print(rollTorqueInput);Serial.print("\t");
    Serial.print("PitchInput:");Serial.print(pitchTorqueInput);Serial.print("\t");
    Serial.print("YawInput:");Serial.println(yawTorqueInput);}
}//end setCanards()

void magRotn(){

  long dotProd;
  long valVect1;
  long valVect2;
  long valVect3;
  float lenVect1;
  float lenVect2;
  float lenVect3;

  //compute roll
  dotProd = mag.x0*mag.x + mag.y0*mag.y;
  valVect1 = mag.x0*mag.x0 + mag.y0*mag.y0;
  valVect2 = mag.x*mag.x + mag.y*mag.y;
  lenVect1 = sqrtf(valVect1);
  lenVect2 = sqrtf(valVect2);
  
  float cosMagRoll = dotProd / (lenVect1 * lenVect2);
  magRoll = speedArcCos(cosMagRoll);

  //compute off vertical
  dotProd = valVect2;
  valVect3 = valVect2 + mag.z*mag.z;
  lenVect3 = sqrtf(valVect3);

  float cosOffVert = dotProd / (lenVect2 * lenVect3);
  magOffVert = speedArcCos(cosOffVert);

  //reduce roll to -360 to 360
  long tempRoll = magRoll;
  while(tempRoll > 3600){tempRoll -= 3600;}
  while(tempRoll < 3600){tempRoll += 3600;}
  
  //compute pitch
  magPitch = magOffVert * speedCos(tempRoll);

  //compute yaw
  magYaw = magOffVert * speedSin(tempRoll);
  
}
