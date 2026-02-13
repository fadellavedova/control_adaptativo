#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Complex.h"

#define D 1
#define NA 3
#define NB 1
#define NC 1
#define NU (NB + 1 + D - 1)
#define NUF 1
#define NPARAMS (NA+NU+NUF)

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

float theta[NPARAMS] = {0, 0, 0, 0, 0.3};
float P[NPARAMS][NPARAMS] = {
  {100,0,0,0,0},
  {0,100,0,0,0},
  {0,0,100,0,0},
  {0,0,0,100,0},
  {0,0,0,0,100},
};

int pwm_view = 0;
float a1 = 0;
float a2 = 0;
float b1 = 0;

// Historia (para los regresores)
float parametros[5] = {0,0,0,0,0};
float y_hist[4] = {0, 0, 0, 0};
float u_hist[3] = {0, 0, 0};
float e_hist[NC] = {0};


float lambda_f = 0.995;
float lambda_b0 = 0.075;
int pwm=40;

float u_k1 = 0;

float tita_d = 0.45;

float denom = 0;

int limite = 160;
float MPC_update(float y_k, float u_k, float y_hist[], float u_hist[], float e_hist[], float r_kd,
                float lambda_f, float lambda_b0, float parametros[]) {

  const int n = NPARAMS;
  float phi[n];
  float phi_C[n];

  phi[0] = -y_hist[0];
  phi[1] = -y_hist[1];
  phi[2] = -y_hist[2];
  phi[3] = -(u_hist[1] - u_hist[2]); //-d-1
  phi[4] = y_k + lambda_b0*(u_hist[0] - u_hist[1]);

  // y_hat = tita phi
  float u_hat = 0;
  for (int i = 0; i < n; i++) u_hat += theta[i] * phi[i];
  float eps = (u_hist[0] - u_hist[1]) - u_hat;

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
  
  phi_C[0] = -y_k;
  phi_C[1] = -y_hist[0];
  phi_C[2] = -y_hist[1];
  phi_C[3] = -(u_k - u_hist[0]);
  phi_C[4] = r_kd;
  // Controladores tilde

  float u_k1 = u_k;
  for (int i = 0; i < n; i++) u_k1 += theta[i] * phi_C[i];

  //u_k1 = -u_k1;
  // Muevo los indices.
  y_hist[3] = y_hist[2];
  y_hist[2] = y_hist[1];
  y_hist[1] = y_hist[0];
  y_hist[0] = y_k;
  
  u_hist[2] = u_hist[1];
  u_hist[1] = u_hist[0];
  u_hist[0] = u_k;
  
  e_hist[0] = eps;
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

    tita_d = 0.7;
  }else if(count<=1000) {
    lambda_f = 0.995;
  }else if(count<=2000) {
    tita_d = 0.9;
  }else if(count<=2500){
    tita_d = 0.7;
  }else if(count<=5000) {
    tita_d = 0.5;
  }else if(count<=5500) {
    tita_d = 0.9;
  }else if(count<=6000){
    tita_d = 0.7;
  }
  else tita_d = 0.5;
  

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
  count++;

  pwm = MPC_update(tita, pwm, y_hist, u_hist, e_hist, tita_d, lambda_f, lambda_b0, parametros);
  pwm_view = pwm;

  pwm = int(pwm);
  if(count>1000){limite = 180;}
  pwm = saturacion(pwm,limite);
  analogWrite(10, pwm);
  // int pwm_viejo = pwm;
  end = micros();
  

  //float datos[8] = {tau2,tita,y_hat,pwm_view,Ki,theta[0],theta[1],theta[2]};
  //float datos[8] = {tita, y_hat, theta[0], theta[1], theta[2], denom, Ki, pwm};
  //float datos[8] = {tita, theta[0], theta[1], theta[2], theta[3], 0, tita_d, pwm_view};
  //float datos[8] = {tita,uf_hist[0],uf_hist[1],uf_hist[2],yf_hist[0],yf_hist[1],yf_hist[2], pwm_view};
  float datos[8] = {tita, tita_d, pwm_view, theta[0],theta[1],theta[2],theta[3],0};
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

