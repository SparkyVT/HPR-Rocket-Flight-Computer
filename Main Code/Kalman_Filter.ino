 /*Kalman strategy
 * 1) update acceleration, speed, and position each time a new accelerometer value comes in
 * 2) if it exceeds 20Gs, then use the high-G accelerometer
 * 3) kinematic equations: 
 *  - accel = accel * cos(offVert) - g
 *  - velocity = velocity + accel * dt
 *  - altitude = altitude + velocity * dt + 0.5 * accel * dt^2
 * 4) Update when new baro reading comes in (only when velocity < 300 m/s and altitude < 9000m)
 * 5) 
 * 
 * 

// A_new = A_old + V_old * dt * cos(OffVert) + .5 * A_old * dt^2

typedef struct{
  float altNew = 0.0F;
  float velNew = 0.0F;
  float accelNew = 0.0F;
}xKalman;
xKalman xNew;

//A matrix
float A_1[2] = {1.0F, 1.0F};
float A_2[2] = {0.0F, 1.0F};
A_1[2] *= dt;

//X matrix
xNew_11 = altNew;
xNew_12 = velNew;

//B matrix
float B1[1] = 0.5F;
float B2[1] = 1.0F;
B1[1] *= (accelNow * accelNow);
B1[2] *= accelNow;

//u matrix
float u1[1] = accelNow;

//P matrix
float P_1[2] ={1.0F, 0.0F}; 
float P_2[2] ={0.0F, 1.0F}; 

//R matrix

accelNew = (sensor readings *cos(offVert))- g

accelNew = accelNew
velNew = velOld + accelNew * dt
altNew = altOld + velNew * dt + .5 * accelNew * dt^2

x = posn
    vel

A = 
              1      dt 
              0      1     
              
           

B =   .5 * dt^2
      dt

u = accel

P = 1 0
    0 1

G= .5*dt^2
   dt

G' = .5*dt^2  dt

Q = sig*.25*dt^4  sig*.5*dt^3
    sig*.5*dt^3   sig*.25*dt^3

R = [baro_sigma 0
     0          Accel_Sigma]
Predict:
x' = A * x + B * u
P' = A * P * At + Q

Measure:
Baro: H = 1 0
Accel: H = 1 1
y = z - H * x'
S = H * P' *Ht + R
K = P' * Ht * inv(S)

Update:
x = x + K * y
P = (I - K * H) * P'
    
float kalmanHighG(float reading){

static float Pk_new=1;
static float Xk_new=0;
static float Xk_calculated=0;
static float Q=0.01;
static float R=1;
static float Kk=0;

static float Zk = reading;

Kk=Pk_new/(Pk_new+R);
Xk_calculated=Xk_new+Kk*(Zk-Xk_new);
Pk_new=(1.0-Kk)*Pk_new+fabs(Xk_new-Xk_calculated)*Q;
Xk_new=Xk_calculated;

return Xk_calculated;
}

float kalmanBaro(float reading){

static float Pk_new=1;
static float Xk_new=0;
static float Xk_calculated=0;
static float Q=0.01;
static float R=1;
static float Kk=0;

static float Zk = reading;

Kk=Pk_new/(Pk_new+R);
Xk_calculated=Xk_new+Kk*(Zk-Xk_new);
Pk_new=(1.0-Kk)*Pk_new+fabs(Xk_new-Xk_calculated)*Q;
Xk_new=Xk_calculated;

return Xk_calculated;
}*/
