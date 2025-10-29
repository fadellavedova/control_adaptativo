#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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

float dtime = 0.02;
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

// Histories (for regressors)
float y_hist[NA] = {0, 0};
float u_hist[NB] = {0};
float e_hist[NC] = {0};

float lambda_f = 0.995;
int pwm;
float ELS_update(float y_k, float u_k, float y_hist[], float u_hist[], float e_hist[],
                float lambda_f) {

  const int n = NPARAMS;
  float phi[n];

  // φ = [y[k-1], y[k-2], y[k-3], u[k-1], e[k-1]]
  phi[0] = y_hist[0];
  phi[1] = y_hist[1];
  phi[2] = u_hist[0];
  phi[3] = e_hist[0];

  // ŷ = θᵀ φ
  float y_hat = 0;
  for (int i = 0; i < n; i++) y_hat += theta[i] * phi[i];
  float eps = y_k - y_hat;

  // Compute denom = λ + φᵀ P φ
  float Pphi[n];
  for (int i = 0; i < n; i++) {
    Pphi[i] = 0;
    for (int j = 0; j < n; j++) Pphi[i] += P[i][j] * phi[j];
  }

  float denom = lambda_f;
  for (int i = 0; i < n; i++) denom += phi[i] * Pphi[i];
  //if (denom < 1e-9) denom += 1e-9;

  // K = P φ / denom
  float K[n];
  for (int i = 0; i < n; i++) K[i] = Pphi[i] / denom;

  // θ = θ + K * ε
  for (int i = 0; i < n; i++) theta[i] += K[i] * eps;

  // Update P = (P - K φᵀ P) / λ

  // outer(K, φ)
  float Kphi[n][n];
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      Kphi[i][j] = K[i] * phi[j];

  // (P - K φᵀ P)
  float newP[n][n];
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      float sum = 0;
      for (int k = 0; k < n; k++)
        sum += Kphi[i][k] * P[k][j];
      newP[i][j] = P[i][j] - sum;
      newP[i][j] /= lambda_f;
    }

  // Copy back
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      P[i][j] = newP[i][j];

  // shift histories for next step
  y_hist[1] = y_hist[0];
  y_hist[0] = y_k;
  

  //for (int i = 1; i >= 1; i--) u_hist[i] = u_hist[i-1];
  u_hist[0] = u_k; // update with current u later

  //for (int i = 1; i >= 1; i--) e_hist[i] = e_hist[i-1];
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

  //pwm = 100 + generateGaussianNoise(0, 10);

/*
  if(count <= 900){
    pwm = 100;
  }else{
    pwm = 130;
  }
*/


  if(count < 200) {
    pwm = 0 ;
  } 
  else if(count >= 200 && count < 400) {
    pwm = 100 + generateGaussianNoise(0, 10);
  }
  else if(count >= 400 && count < 600){
    pwm = 180 + generateGaussianNoise(0, 10);
  }
  else if(count >= 600 && count < 800) {
    pwm = 180 + generateGaussianNoise(0, 10);
  }
  else if(count >= 800 && count < 1000){
    pwm = 0;
    count = 0;
  }


  analogWrite(9, pwm);
  count++;


  tita = filtro_complementario(tita, alpha);

  end = micros();
  
  y_hat = ELS_update(tita, pwm, y_hist, u_hist, e_hist, lambda_f);

  float datos[8] = {tita, y_hat, theta[0], theta[1], theta[2], theta[3], 0, pwm};

  matlab_send(datos, sizeof(datos)/sizeof(float));

  delay(19 - (end - start)/1000);
  delayMicroseconds(1000 - (end - start)%1000);

}

float filtro_complementario(float tita, float alpha) {
  mpu.getEvent(&a, &g, &temp);

  float tita_a = atan2(a.acceleration.x, a.acceleration.y);

  float tita_gc = tita + dtime*g.gyro.z;
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
  int numSamples = 12; // A common choice for a reasonable approximation

  for (int i = 0; i < numSamples; i++) {
    sum += random(0, 1000) / 1000.0; // Generate uniform random numbers between 0 and 1
  }

  if((sum - 6.0) * stdDev + mean >= 0)
    return 0;
  else
    return 0;

}

