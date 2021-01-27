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

const float pi = 3.14159265359;
const float radDeg = 57.295780;

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
  while(tempRoll > 3600){tempRoll - 3600;}
  while(tempRoll < 3600){tempRoll + 3600;}
  
  //compute pitch
  magPitch = magOffVert * speedCos(tempRoll);

  //compute yaw
  magYaw = magOffVert * speedSin(tempRoll);
  
}

void getRotnDCM2D(){

  float dx = 0.0F;
  float dy = 0.0F;
  static int64_t dz = 0;
  float deg2rad = 3.141592653589793238462 / 180;
  float rad2deg = 1/deg2rad;
  float cosZ;
  float sinZ;
  
  //Calculate new Z angle from gyro data
  int32_t newRoll = gyro.z * fltTime.gdt;
  dz += newRoll;
  float radNewRoll = newRoll * deg2rad * gyro.gainZ;
  
  //compute sin(roll) and cos(roll)
  float sinNewRoll = sinSmallAngle(radNewRoll);
  float cosNewRoll = cosSmallAngle(radNewRoll);
  float prevCosZ = cosZ;
  float prevSinZ = sinZ;
  cosZ = prevCosZ * cosNewRoll - prevSinZ * sinNewRoll;
  sinZ = prevSinZ * cosNewRoll + prevCosZ * sinNewRoll;
  
  //Compute the new y and x angles
  dx += (cosZ * gyro.x - sinZ * gyro.y) * fltTime.gdt;
  dy += (cosZ * gyro.y + sinZ * gyro.x) * fltTime.gdt;

  //Compute Pitch, Roll, Yaw
  const float convDeg = gyro.gainZ * mlnth;
  rollZ = (long)(dz * convDeg);
  pitchX = (int)(dx * convDeg * 10);
  yawY = (int)(dy * convDeg * 10);

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

void setCanards(){

  static float rollErrorInt = 0;
  static const float Kp = 0.10;
  static const float Ki = 0.05;
  static const float Kd = 0.10;
  static const float rollSetPoint = 0;
  static const float pitchSetPoint = 0;
  static const float yawSetPoint = 0;
  static unsigned long dt;
  static uint32_t lastUpdate = 0UL;
  static float pitchErrorInt = 0.0F;
  static float yawErrorInt = 0.0F;
  float rollRad;
  float rollPosn;
  float pitchPosn;
  float yawPosn;

  //Update time stamps
  uint32_t timeNow = micros();
  dt = timeNow - lastUpdate;
  lastUpdate = timeNow;
  
  //Determine current position error
  float rollError = rollZ - rollSetPoint;
  float pitchError = pitchX - pitchSetPoint;
  float yawError = yawY - yawSetPoint;

  //Update roll correction
  if(settings.stableRotn){

    //Update Integral Terms
    rollErrorInt += rollError*dt*mlnth;

    //Update roll position correction
    rollPosn = Kp*rollError + Ki*rollErrorInt + Kd*gyro.z*gyro.gainZ;}
  else{rollPosn = 0.0F;}
  
  //Update off-vertical error terms, restrict values to when the roll is only close to the original starting point
  if(settings.stableVert && abs(rollError) < 10){
    
    //Update integral terms
    pitchErrorInt += pitchError*dt*mlnth;
    yawErrorInt += yawError*dt*mlnth;

    //Update pitch & yaw correction positions
    long rollAngle = rollZ*10;
    pitchPosn = Kp*pitchError + Ki*pitchErrorInt + Kd * gyro.gainX * (speedCos(rollZ*10)*gyro.x + speedSin(rollZ)*gyro.y);
    yawPosn   = Kp*yawError   + Ki*yawErrorInt   + Kd * gyro.gainY * (speedCos(rollZ*10)*gyro.y - speedSin(rollZ)*gyro.x);
    
    //pitch and yaw corrections are limited to 2 degrees only
    pitchPosn > 2 ? 2 : pitchPosn;
    pitchPosn < -2 ? -2 : pitchPosn;
    yawPosn > 2 ? 2 : yawPosn;
    yawPosn < -2 ? -2 : yawPosn;}
  else{pitchPosn = yawPosn = 0;}

  //Set the fin positions
  //fin pointed in the positive-y world-frame
  canardYaw1.write(  rollPosn + yawPosn   * speedCos(rollZ) + pitchPosn * speedSin(rollZ));
  //fin pointed in the negative-y world-frame
  canardYaw2.write(  rollPosn - yawPosn   * speedCos(rollZ) - pitchPosn * speedSin(rollZ));
  //fin pointed in the positive-x world-frame
  canardPitch1.write(rollPosn + pitchPosn * speedCos(rollZ) + yawPosn   * speedSin(rollZ));
  //fin pointed in teh negative-x world-frame
  canardPitch2.write(rollPosn - pitchPosn * speedCos(rollZ) - yawPosn   * speedSin(rollZ));}
