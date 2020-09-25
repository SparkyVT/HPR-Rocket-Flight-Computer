void getRotnDCM2D(){

  //Calculate new Z angle from gyro data
  dz += gyro.z * fltTime.gdt;
  //Overflow Z data and recompute as needed
  counterSign = (dz > 0) ? 1 : -1;
  while (abs(dz) > oneDeg) {
    dz -= counterSign * oneDeg;
    rollZ += counterSign;
    //cos(A+B) = cosAcosB - sinAsinB A=big B=small cos(1deg)=0.999847695
    cosZ = PrevCosZ * 0.999847695 - PrevSinZ * (counterSign * 0.017452406);
    //sin(A+B) = sinAcosB + cosAsinB A=big B=small
    sinZ = PrevSinZ * 0.999847695 + PrevCosZ * (counterSign * 0.017452406);
    PrevCosZ = cosZ;
    PrevSinZ = sinZ;}
      
  //Compute the new y and x angles
  dx += cosZ * gyro.x * fltTime.gdt;
  dx += sinZ * gyro.y * fltTime.gdt;
  dy += cosZ * gyro.y * fltTime.gdt;
  dy -= sinZ * gyro.x * fltTime.gdt;
  //Overflow Y data and recompute as needed
  counterSign = (dy > 0) ? 1 : -1;
  while (abs(dy) > oneTenthDeg) {
    dy -= counterSign * oneTenthDeg;
    yawY += counterSign;
    calcOffVert = true;}
    
  //Overflow X data and recompute as needed
  counterSign = (dx > 0) ? 1 : -1;
  while (abs(dx) > oneTenthDeg) {
    dx -= counterSign * oneTenthDeg;
    pitchX += counterSign;
    calcOffVert = true;}

  //Check if the max angle is exceeded, shutdown staging if angle > 45 is detected
  if ((!events.apogee && calcOffVert) || settings.testMode){
    tanYaw = speedTan(yawY);
    tanPitch = speedTan(pitchX);
    //calculate the off-vertical rotation angle
    float hyp1 = tanYaw*tanYaw + tanPitch*tanPitch;
    float hyp2 = pow(hyp1, 0.5);
    offVert = speedArcTan(hyp2);
    calcOffVert = false;
      
    //check to see if the maximum angle has been exceeded
    if (offVert > settings.maxAngle) {rotnOK = false;}
    else if (!rotationFault){rotnOK = true;}
    if(!rotationFault && offVert > 450){rotationFault = true;}}
}//end void

void getQuatRotn(float dx, float dy, float dz){

//Local Vectors
float NewPoint[4];
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
float quatLen = pow( Quat[1]*Quat[1] + Quat[2]*Quat[2] + Quat[3]*Quat[3] + Quat[4]*Quat[4], -0.5);
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
float tanYaw;
float tanPitch;
tanYaw = speedTan(yawY);
tanPitch = speedTan(pitchX);

float hyp1;
float hyp2;
hyp1 = tanYaw*tanYaw + tanPitch*tanPitch;
hyp2 = pow(hyp1, 0.5);
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
  lenVect1 = sqrt(valVect1);
  lenVect2 = sqrt(valVect2);
  
  float cosMagRoll = dotProd / (lenVect1 * lenVect2);
  magRoll = speedArcCos(cosMagRoll);

  //compute off vertical
  dotProd = valVect2;
  valVect3 = valVect2 + mag.z*mag.z;
  lenVect3 = sqrt(valVect3);

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
