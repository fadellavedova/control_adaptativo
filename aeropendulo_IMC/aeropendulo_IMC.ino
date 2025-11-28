#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Complex.h"

#define NA 2
#define NB 1
#define NC 1
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

float theta[NPARAMS] = {0, 0,0, 0};
float P[NPARAMS][NPARAMS] = {
  {100,0,0,0},
  {0,100,0,0},
  {0,0,100,0},
  {0,0,0,100},
};

// Historia (para los regresores)
float y_hist[NA] = {0, 0};
float u_hist[NB] = {0};
float e_hist[NC] = {0};

float lambda_f = 0.995;
int pwm;

float Kp = 0;
float Ki = 0;
float Kd = 0;
float tita_d = 0.8;
float error = 0;
float error_anterior = 0;
float I = 0;
float D = 0;
float beta = 3;
float gamma_derivador = 2*beta;
float denom = 0;
float Kbeta = 0;
float tau2 = 0;
float cetatau = 0;
float b = dt;
float termino = 0;
float kappa = 0.5;

float ELS_update(float y_k, float u_k, float y_hist[], float u_hist[], float e_hist[],
                float lambda_f) {

  const int n = NPARAMS;
  float phi[n];

  // phi = [y[k-1], y[k-2],u[k-1], e[k-1]]
  phi[0] = y_hist[0];
  phi[1] = y_hist[1];
  phi[2] = u_hist[0];
  phi[3] = e_hist[0];

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
  //if (denom < 1e-9) denom += 1e-9;

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

  // Muevo los indices.
  y_hist[1] = y_hist[0];
  y_hist[0] = y_k;
  
  u_hist[0] = u_k;
  
  e_hist[0] = eps;

  return y_hat;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Pines
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(9, OUTPUT);

  //IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
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



  analogWrite(9, pwm);
  count++;


  tita = filtro_complementario(tita, alpha);

  
  if(count < 1500) {
    pwm = 100 + generateGaussianNoise(0, 10);

    y_hat = ELS_update(tita, pwm, y_hist, u_hist, e_hist, lambda_f);
  } 
  else {

      //if(termino < -0.9) termino = -0.99; 
      denom = -theta[0] - theta[1] + 1;
      if(denom<0.0001 && denom>-0.0001) denom=0.0001;
      Kbeta = beta*theta[2]/denom;
      tau2 = dt*dt/denom;
      cetatau = (0.5)*dt*(-theta[0] + 2)/denom;
      b = dt;

      Kp = kappa*(2*cetatau - b)/(Kbeta) + (1-kappa)*Kp;
      Ki = kappa*1/(Kbeta) + (1-kappa)*Ki;
      Kd = kappa*(tau2 - 2*b*cetatau + b*b)/(Kbeta) + (1-kappa)*Kd;

      //Kp = (4*cetatau - beta)/(4*Kbeta);
      //Ki = 0.5/Kbeta;
      //Kd = (4*tau2 - 4*beta*cetatau + beta*beta)/(8*Kbeta);

      error_anterior = error;
      error = tita_d - tita;
      if(error < 1e-2) error = 0;
      
      pwm = int(Kp*error + Ki*(I + (dt/2)*error + (dt/2)*error_anterior) + Kd*(2*(error-error_anterior)/dt - D));

      I = I + (dt/2)*error + (dt/2)*error_anterior;
      D = (error-error_anterior - D*(dt - 1))/gamma_derivador;
      pwm = saturacion(pwm);
      int pwm_viejo = pwm;
      
  }


  end = micros();
  
 // float datos[8] = {tita, y_hat, theta[0], theta[1], theta[2], denom, Ki, pwm};
  float datos[8] = {tita, theta[0], theta[1], theta[2], Kp, Ki, Kd, pwm};
  matlab_send(datos, sizeof(datos)/sizeof(float));

  delay(19 - (end - start)/1000);
  delayMicroseconds(1000 - (end - start)%1000);

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

float generateGaussianNoise(float mean, float stdDev) {
  float sum = 0.0;
  int numSamples = 12; 

  for (int i = 0; i < numSamples; i++) {
    sum += random(0, 1000) / 1000.0;
  }

  if((sum - 6.0) * stdDev + mean >= 0)
    return 30;
  else
    return -30;

}

int saturacion(int pwm) {
  return constrain(pwm, 0, 220);
}

