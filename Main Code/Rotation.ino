
void getRotnDCM2D(){

  //Calculate new X angle from gyro data
  dx += gyroX * gdt;
  //Overflow X data and recompute as needed
  while (abs(dx) > oneDeg) {
    if (dx > 0) {counterSign = 1;}
    else {counterSign = -1;}
    dx -= counterSign * oneDeg;
    rotnX += counterSign;
    //cos(A+B) = cosAcosB - sinAsinB A=big B=small cos(1deg)=0.999847695~=1
    cosX = PrevCosX * 0.999847695 - PrevSinX * (counterSign * 0.017452406);
    //sin(A+B) = sinAcosB + cosAsinB A=big B=small
    sinX = PrevSinX * 0.999847695 + PrevCosX * (counterSign * 0.017452406);
    PrevCosX = cosX;
    PrevSinX = sinX;}
      
  //Compute the new y and z angles
  dz += cosX * gyroZ * gdt;
  dz += sinX * gyroY * gdt;
  dy += cosX * gyroY * gdt;
  dy -= sinX * gyroZ * gdt;
  //Overflow Y data and recompute as needed
  while (abs(dy) > oneTenthDeg) {
    if (dy > 0) {counterSign = 1;}
    else {counterSign = -1;}
    dy -= counterSign * oneTenthDeg;
    rotnY += counterSign;
    calcOffVert = true;}
  //Overflow Z data and recompute as needed
  while (abs(dz) > oneTenthDeg) {
    if (dz > 0) {counterSign = 1;}
    else {counterSign = -1;}
    dz -= counterSign * oneTenthDeg;
    rotnZ += counterSign;
    calcOffVert = true;}

  //Check if the max angle is exceeded, shutdown staging if angle > 45 is detected
  if (!sustainerFireCheck && !rotationFault && calcOffVert){
    
    //calculate the off-vertical rotation angle
    if(abs(rotnY) > 10 && abs(rotnZ) > 10){offVert = sqrt(sq(rotnY) + sq(rotnZ));}
    else{offVert = max(abs(rotnY), abs(rotnZ));}
    calcOffVert = false;
      
    //check to see if the maximum angle has been exceeded
    if (offVert > max_ang) {rotation_OK = false;}
    else {rotation_OK = true;}
    if(offVert > 450){rotationFault = true;}}
}//end void

/*void getRotnQuat(float dx, float dy, float dz){

//Local Vectors
float NewPoint[4];
float QuatDiff[5];
float Rotn1[4];
float Rotn2[4];
float Rotn3[4];

const float pi = 3.14159265359;
    
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
pitchX = atan2(Rotn3[2], Rotn3[3]);
rollZ = atan2(Rotn2[1], Rotn1[1]);
yawY = asin(-Rotn3[1]);

float tanYaw;
float tanPitch;

if(yawY <= 0.176){tanYaw = yawY;}
else{tanYaw = tan(yawY);}

if(pitchX <= 0.176){tanPitch = pitchX;}
else{tanPitch = tan(pitchX);}

offVert = atan(pow(tanYaw*tanYaw + tanPitch*tanPitch, 0.5));

}*/
