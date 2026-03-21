#define NA 2
#define NB 1
#define NC 1
#define NPARAMS (NA+NB+NC)

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

float lambda_f = 0.99;

static float a1 = 0.95895306;
static float a2 = 0.16227342;
static float a3 = -0.38178931;
static float a4 = 0.63440622;

static float b1 = 0.08723171;
static float b2 = 0.8113671;

static float a = 1e-8;
static float b = 1.5e-6;
static float c = 5e-5;

static float tita;
static float tita_punto;

static float tita_prev;
static float tita_punto_prev;
static int count = 0;
static float m = 0.05;


static float F_eq = (a*45*45*45*45 + b*45*45*45 + c*45*45) * m * 9.8;
static float F;
static float F_prev;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting online ELS identification...");
}



void ELS_update(float y_k, float y_hist[], float u_hist[], float e_hist[],
                float lambda_f) {

  const int n = NPARAMS;
  float phi[n];

  // φ = [y[k-1], y[k-2], y[k-3], u[k-1], e[k-1]]
  phi[0] = y_hist[0];
  phi[1] = y_hist[1];
  phi[3] = u_hist[0];
  phi[4] = e_hist[0];

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
  for (int i = 1; i >= 1; i--) y_hist[i] = y_hist[i-1];
  y_hist[0] = y_k;

  u_hist[0] = u_k; // update with current u later

  e_hist[0] = eps;
}


void loop() {

  if (count > 0) {
    F = 0.1 * F_eq;   // input force
  } else {
    count++;
  }

  // --- Plant simulation ---
  tita = a1 * tita_prev + a2 * tita_punto_prev + b1 * F_prev;
  tita_punto = a3 * tita_prev + a4 * tita_punto_prev + b2 * F_prev;

  // --- Update states ---
  tita_prev = tita;
  tita_punto_prev = tita_punto;
  F_prev = F;

  // --- Define signals for identification ---
  float y_k = tita;   // current output
  float u_k = F;      // current input

  // --- Recursive update ---
  ELS_update(y_k, y_hist, u_hist, e_hist, lambda_f);

  // --- Print current estimates ---
  Serial.print("Theta: ");
  for (int i = 0; i < NPARAMS; i++) {
    Serial.print(theta[i], 6);
    Serial.print(" ");
  }
  Serial.println();

  // --- Update histories ---
  u_hist[0] = u_k;  // feed latest input

  delay(100); // simulate sampling time
}

