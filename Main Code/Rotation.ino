
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
