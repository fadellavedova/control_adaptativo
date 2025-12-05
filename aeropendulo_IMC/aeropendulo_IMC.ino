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

int pwm_view = 0;
float a1 = 0;
float a2 = 0;
float b1 = 0;

// Historia (para los regresores)
float y_hist[NA] = {0, 0};
float u_hist[NB] = {0};
float e_hist[NC] = {0};

float lambda_f = 0.99;
int pwm;

float Kp = 0;
float Ki = 0;
float Kd = 0;

bool promediando = false;
int count_promedio = 0;

float termino_1;float termino_2;float termino_3;
float tita_d = 0.45;
float error = 0;
float error_anterior = 0;
float I = 0;
float D = 0;

float beta = 3;

float denom = 0;
float K = 0;
float tau2 = 0;
float tau = 0;
float ceta = 0;
float b = dt;
float gamma_derivador = b;
float kappa = 1;
float dfactor = 1;

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



  analogWrite(10, pwm);
  count++;
 // Serial.println("mori1");
  if(count < 4500) tita_d = 1.2;
  /*
  else if (count < 5500) tita_d = 0.6;
  else if (count < 6500) tita_d = 0.8;
  else if (count < 7500) tita_d = 0.65;
  else if (count < 8500) tita_d = 0.50;
  else if (count < 9500) tita_d = 0.35;
  */
  tita = filtro_complementario(tita, alpha);
  //Serial.println("mori_imu");
  if(count < 1000) {
    pwm = 120 + generateGaussianNoise(0, 10, 40);
   // Serial.println("mori2");
    y_hat = ELS_update(tita, pwm, y_hist, u_hist, e_hist, lambda_f);
   // Serial.println("mori3");
  } 
  else if (count < 3000) {
    pwm = 120 + generateGaussianNoise(0, 10, 40);

    y_hat = ELS_update(tita, pwm, y_hist, u_hist, e_hist, lambda_f);
    a1 += theta[0];
    a2 += theta[1];
    b1 += theta[2];
  }
  else if (count < 3200) {
    theta[0] = a1/1999;
    theta[1] = a2/1999;
    theta[2] = b1/1999;

    pwm = 0;
    denom = -theta[0] - theta[1] + 1;
    K = theta[2]/denom;
    tau2 = dt*dt/denom;
    tau = sqrt(abs(tau2));
    ceta = (0.5)*dt*(-theta[0] + 2)/(denom*tau);
    b = dt;

    Kp = (2*ceta*tau - b)/(K*beta);
    Ki = 1/(K*beta);
    Kd = (tau2 - 2*b*ceta*tau + b*b)/(K*beta);

    count_promedio++;
  }
  else {
      //y_hat = ELS_update(tita, pwm, y_hist, u_hist, e_hist, lambda_f);
      //if(termino < -0.9) termino = -0.99; 
      // if(denom<0.0001 && denom>-0.0001) denom=0.0001;

      //Kp = (4*cetatau - beta)/(4*Kbeta);
      //Ki = 0.5/Kbeta;
      //Kd = (4*tau2 - 4*beta*cetatau + beta*beta)/(8*Kbeta);

      error_anterior = error;
      error = tita_d - tita;
    //  if(abs(error) < 1e-5) error = 0;
    //  pwm = Kp*error + Ki*(I + (dt/2)*error + (dt/2)*error_anterior) + Kd*(2*(error-error_anterior)/dt - D);


      D= (2/dt)*(error - error_anterior)*(1/(2/dt + 1)) - D * ((1-(2/dt*gamma_derivador))/(2/dt +1));
      I = I + (dt/2)*error + (dt/2)*error_anterior;

      termino_1 = Kp*error;
      termino_2 = Ki*I;
      termino_3 = Kd*D;
     // termino_3 = Kd*(2*(error-error_anterior)/dt - D);
      pwm = termino_1 + termino_2 + termino_3;

     //D = 2*(error-error_anterior)/dt - D;


      //pwm = 120;
      //D = (error-error_anterior - D*(dt - 1))/gamma_derivador;


      pwm_view = pwm;
      pwm = saturacion(pwm);
     // int pwm_viejo = pwm;
      
  }


  end = micros();
  

  //float datos[8] = {tau2,tita,y_hat,pwm_view,Ki,theta[0],theta[1],theta[2]};
  //float datos[8] = {tita, y_hat, theta[0], theta[1], theta[2], denom, Ki, pwm};
  float datos[8] = {tita, theta[0], theta[1],Kp, tita_d, Ki, Kd, pwm};
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
    return 2*cant;
  else
    return 0;

}

int saturacion(int pwm) {
  return constrain(pwm, 0, 220);
}

