void getRotn(float dx, float dy, float dz){

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
rollZ = atan2(Rotn3[2], Rotn3[3]);
pitchX = atan2(Rotn2[1], Rotn1[1]);
yawY = asin(-Rotn3[1]);

float tanYaw;
float tanPitch;

if(yawY <= 0.176){tanYaw = yawY;}
else{tanYaw = tan(yawY);}

if(pitchX <= 0.176){tanPitch = pitchX;}
else{tanPitch = tan(pitchX);}

offVert = atan(pow(tanYaw*tanYaw + tanPitch*tanPitch, 0.5));

}

void magRotn(float mX, float mY, float mZ){

//global variables
float magPitch;
float magYaw;
float magRoll;




  
}

