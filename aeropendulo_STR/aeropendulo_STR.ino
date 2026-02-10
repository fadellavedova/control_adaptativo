#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Complex.h"

#define NA 2
#define NB 1
#define NC 1
#define NACL 3
#define NPARAMS (NA+NB+NC)

Adafruit_MPU6050 mpu;

float tita = 0;
unsigned long start;
unsigned long end;
float sesgo_tita = 0;
static float alpha = 0.005;

float dt = 0.02;
float y_hat;

sensors_event_t a, g, temp;

int motor_pin1 = 2;
int motor_pin2 = 4;

float theta[NPARAMS] = {0, 0, 0, 0};
float P[NPARAMS][NPARAMS] = {
  {100,0,0,0},
  {0,100,0,0},
  {0,0,100,0},
  {0,0,0,100},
};

int pwm_view = 0;
float a1 = 0;
float a2 = 0;
float b1 = 0;

float acl1 = -2.4;
float acl2 = 1.91;
float acl3 = -0.504;

float uf_k = 0;
float yf_k = 0;
float u_k1 = 0; 
float ref = 0.45;

// Historia (para los regresores)
float parametros[5] = {0,0,0,0,0};
float y_hist[NA] = {0, 0};
float u_hist[NB] = {0};
float e_hist[NC] = {0};
float yf_hist[NACL] = {0, 0, 0}; 
float uf_hist[NACL] = {0, 0, 0};

float lambda_f = 0.999;
int pwm=40;

float br0 = 0.1; float br1 = 0 ; float bs0 = 0 ; float bs1 = 0;
float r0 =0 ; float r1=0; float s0=0; float s1=0 ; float t0= 0;

float tita_d = 0.45;

float denom = 0;

int limite = 150;
float STR_update(float y_k, float u_k, float y_hist[], float u_hist[], float e_hist[], float yf_hist[], float uf_hist[],float ref,
                float lambda_f,float parametros[]) {

  const int n = NPARAMS;
  float phi[n];

  // phi = [y[k-1], y[k-2],u[k-1], e[k-1]]
  yf_k = -acl1*yf_hist[0] - acl2*yf_hist[1] - acl3*yf_hist[2] + y_hist[0];
  uf_k = -acl1*uf_hist[0] - acl2*uf_hist[1] - acl3*uf_hist[2] + u_hist[0];

  phi[0] = uf_k;
  phi[1] = uf_hist[0];
  phi[2] = yf_k;
  phi[3] = yf_hist[0];


  // y_hat = tita phi
  float y_hat = 0;
  for (int i = 0; i < n; i++) y_hat += theta[i] * phi[i];
  float eps = y_k - y_hat;

  // denominador = lambda + phit P phi
  float Pphi[n];
  for (int i = 0; i < n; i++) {
    Pphi[i] = 0;
    for (int j = 0; j < n; j++) Pphi[i] += P[i][j] * phi[j];
  }

  float denom = lambda_f;
  for (int i = 0; i < n; i++) denom += phi[i] * Pphi[i];
  //if (denom < 1e-9) denom += 1e-9; //por las dudas

  // Calculo la ganancia 
  float K[n];
  for (int i = 0; i < n; i++) K[i] = Pphi[i] / denom;

  // Paso de actualizacion de tita con el error y la ganancia calculada
  for (int i = 0; i < n; i++) theta[i] += K[i] * eps;

  //Actualizo la matriz de covarianza de los parametros

  float Kphi[n][n];
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      Kphi[i][j] = K[i] * phi[j];

  //Actualización de la incertidumbre
  float newP[n][n];
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      float sum = 0;
      for (int k = 0; k < n; k++)
        sum += Kphi[i][k] * P[k][j];
      newP[i][j] = P[i][j] - sum;
      newP[i][j] /= lambda_f;
    }

  // Guardo para atras
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      P[i][j] = newP[i][j];

  // Controladores tilde
  br0 = theta[0];

  //if(abs(br0) < 0.01){
  //  br0 = (br0/(abs(br0) + 1e-3))*0.01;
  //}
  br1 = theta[1];
  bs0 = theta[2];
  bs1 = theta[3];

  // Controladores ene a zeta i
  r0 = 1;
  r1 = br1/br0;
  s0 = bs0/br0;
  s1 = bs1/br0;
  t0 = (acl1 + acl2+ acl3 + 1)/br0;

  u_k1 = t0*ref - s0*y_k - s1*y_hist[0] - r1*u_k;
  //u_k1 = -u_k1;
  // Muevo los indices.
  y_hist[1] = y_hist[0];
  y_hist[0] = y_k;
  
  u_hist[0] = u_k;

  uf_hist[2] = uf_hist[1];
  uf_hist[1] = uf_hist[0];
  uf_hist[0] = uf_k; 

  yf_hist[2] = yf_hist[1];
  yf_hist[1] = yf_hist[0];
  yf_hist[0] = yf_k; 
  
  e_hist[0] = eps;

  parametros[0] = t0;
  parametros[1] = r0;
  parametros[2] = r1;
  parametros[3] = s0;
  parametros[4] = s1;

  return u_k1;
}





void setup() {
  Serial.begin(115200);
  delay(500);

  // Pines
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(10, OUTPUT);

  //IMU
  if (!mpu.begin()) {
 //   Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
  }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  for(int i = 0; i<50; i++){

    start = micros();

    tita = filtro_complementario(tita, alpha);
    sesgo_tita += tita;
    Serial.println(tita);
    
    end = micros();

    delay(19 - (end - start)/1000);
    delayMicroseconds(1000 - (end - start)%1000);



  }

  sesgo_tita = sesgo_tita/50;

  delay(500);

}

int count = 0;





void loop() {
  // put your main code here, to run repeatedly:

  start = micros();
  digitalWrite(motor_pin1, HIGH);
  digitalWrite(motor_pin2, LOW);
//PRBS cte para identificacion con los parametros.

  count++;

  if(count<=300) {
    lambda_f = 0.99;
    tita_d = 0.9;
  }else if(count<=1000) {
    lambda_f = 0.998;
  }else if(count<=2000) {
    tita_d = 1.1;
  }else if(count<=2500){
    tita_d = 1.5;
    lambda_f = 0.995;
  }else if(count<=3500) {
    tita_d = 1.7;
  }else if(count<=4000) {
    tita_d = 1.9;
    lambda_f = 0.9999;
  }else if(count<=4500){
    tita_d = 2.1;
  }else if(count<=5000) {
    tita_d = 2.3;
  }else if(count<=5500) {
    tita_d = 2.5;
  }else if(count<=6000){
    tita_d = 2.7;
  }
  else tita_d = 2.9;
  

/*
//Respuesta al impulso
  if(count <= 900){
    pwm = 100;
  }else{
    pwm = 130;
  }
*/

//Este fue para el ultimo ensayo
/*
  if(count < 200) {
    pwm = 0 ;
  } 
  else if(count >= 200 && count < 400) {
    pwm = 100 + generateGaussianNoise(0, 10);
  }
  else if(count >= 400 && count < 600){
    pwm = 130 + generateGaussianNoise(0, 10);
  }
  else if(count >= 600 && count < 800) {
    pwm = 130 + generateGaussianNoise(0, 10);
  }
  else if(count >= 800 && count < 1000){
    pwm = 0;
    count = 0;
  }
*/



  
 // Serial.println("mori1");
  
  tita = filtro_complementario(tita, alpha);

  pwm = STR_update(tita, pwm, y_hist, u_hist, e_hist, yf_hist, uf_hist,tita_d,lambda_f,parametros);
  pwm_view = pwm;

  pwm = int(pwm);
  if(count>1000){limite = 250;}
  pwm = saturacion(pwm,limite);
  analogWrite(10, pwm);
  // int pwm_viejo = pwm;
  end = micros();
  

  //float datos[8] = {tau2,tita,y_hat,pwm_view,Ki,theta[0],theta[1],theta[2]};
  //float datos[8] = {tita, y_hat, theta[0], theta[1], theta[2], denom, Ki, pwm};
  //float datos[8] = {tita, theta[0], theta[1], theta[2], theta[3], 0, tita_d, pwm_view};
  //float datos[8] = {tita,uf_hist[0],uf_hist[1],uf_hist[2],yf_hist[0],yf_hist[1],yf_hist[2], pwm_view};
  float datos[8] = {tita, tita_d, pwm, parametros[0],parametros[1],parametros[2],parametros[3],parametros[4]};
  //Serial.println("mori_datos");
  //float datos[8] = {tita, termino_1,termino_2,termino_3, theta[0], Ki, Kd, pwm_view};

  matlab_send(datos, sizeof(datos)/sizeof(float));

  delay(19 - (end - start)/1000);
  delayMicroseconds(1000 - (end - start)%1000);

  //Serial.println("termine_un_loop");

}

float filtro_complementario(float tita, float alpha) {
  mpu.getEvent(&a, &g, &temp);

  float tita_a = atan2(a.acceleration.x, a.acceleration.y);

  float tita_gc = tita + dt*g.gyro.z;
  float out = alpha*tita_a + (1-alpha)*tita_gc;
  return out;
}

void matlab_send(float* datos, int N){
  Serial.write("abcd");
  for (int i = 0; i < N; i++) {
    byte * b = (byte *) &datos[i];
    Serial.write(b,4);
  }
}

float generateGaussianNoise(float mean, float stdDev,float cant) {
  float sum = 0.0;
  int numSamples = 12; 

  for (int i = 0; i < numSamples; i++) {
    sum += random(0, 1000) / 1000.0;
  }

  if((sum - 6.0) * stdDev + mean >= 0)
    return cant;
  else
    return -cant;

}

int saturacion(int pwm,int limite) {
  return constrain(pwm, 0, limite);
}

