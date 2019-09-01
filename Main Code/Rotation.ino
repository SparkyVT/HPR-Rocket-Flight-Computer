void getRotnDCM2D(){

  //Calculate new X angle from gyro data
  dz += gyroZ * gdt;
  //Overflow X data and recompute as needed
  while (abs(dz) > oneDeg) {
    if (dz > 0) {counterSign = 1;}
    else {counterSign = -1;}
    dz -= counterSign * oneDeg;
    rollZ += counterSign;
    //cos(A+B) = cosAcosB - sinAsinB A=big B=small cos(1deg)=0.999847695~=1
    cosZ = PrevCosZ * 0.999847695 - PrevSinZ * (counterSign * 0.017452406);
    //sin(A+B) = sinAcosB + cosAsinB A=big B=small
    sinZ = PrevSinZ * 0.999847695 + PrevCosZ * (counterSign * 0.017452406);
    PrevCosZ = cosZ;
    PrevSinZ = sinZ;
    }
      
  //Compute the new y and z angles
  dx -= cosZ * gyroX * gdt;
  dx += sinZ * gyroY * gdt;
  dy += cosZ * gyroY * gdt;
  dy += sinZ * gyroX * gdt;
  //Overflow Y data and recompute as needed
  while (abs(dy) > oneTenthDeg) {
    if (dy > 0) {counterSign = 1;}
    else {counterSign = -1;}
    dy -= counterSign * oneTenthDeg;
    yawY += counterSign;
    calcOffVert = true;}
    
  //Overflow X data and recompute as needed
  while (abs(dx) > oneTenthDeg) {
    if (dx > 0) {counterSign = 1;}
    else {counterSign = -1;}
    dx -= counterSign * oneTenthDeg;
    pitchX += counterSign;
    calcOffVert = true;}

  //Check if the max angle is exceeded, shutdown staging if angle > 45 is detected
  if ((!apogee && calcOffVert) || testMode){
    tanYaw = speedTan(yawY);
    tanPitch = speedTan(pitchX);
    //calculate the off-vertical rotation angle
    float hyp1 = tanYaw*tanYaw + tanPitch*tanPitch;
    float hyp2 = pow(hyp1, 0.5);
    offVert = speedArcTan(hyp2);
    calcOffVert = false;
      
    //check to see if the maximum angle has been exceeded
    if (offVert > max_ang) {rotation_OK = false;}
    else if (!rotationFault){rotation_OK = true;}
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
int quatPitchX;
int quatYawY;
int quatRollZ;

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

//Compute angle off vertical
//quatPitchX = speedAtan2(Rotn3[2], Rotn3[3]);
//quatRollZ = speedAtan2(Rotn2[1], Rotn1[1]);
//quatYawY = speedArcSin(-Rotn3[1]);
pitchX = speedAtan2(Rotn3[2], Rotn3[3]);
rollZ = speedAtan2(Rotn2[1], Rotn1[1]);
yawY = speedArcSin(-Rotn3[1]);

//rollZ = (int)(quatRollZ*radDeg);
//yawY = (int)(quatYawY*radDeg*10);
//pitchX = (int)(quatPitchX*radDeg*10);

float tanYaw;
float tanPitch;
tanYaw = speedTan(yawY);
tanPitch = speedTan(pitchX);

float hyp1;
float hyp2;
hyp1 = tanYaw*tanYaw + tanPitch*tanPitch;
hyp2 = pow(hyp1, 0.5);
offVert = speedArcTan(hyp2);

//Check if the max angle is exceeded
if (!sustainerFireCheck && rotation_OK && offVert > max_ang){rotation_OK = false;}
}
