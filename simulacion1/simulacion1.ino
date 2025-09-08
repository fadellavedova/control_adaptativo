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
  // put your setup code here, to run once:
  Serial.begin(115200);
	delay(100);

}

void loop() {
  if (count > 0) {
    F = 0.1*F_eq;
  }
  else {
    count++;
  }
  
  tita = a1*tita_prev + a2*tita_punto_prev + b1*F_prev;
  tita_punto = a3*tita_prev + a4*tita_punto_prev + b2*F_prev;

  tita_prev = tita;
  tita_punto_prev = tita_punto;
  F_prev = F;

  matlab_send(tita, F);
  
  delay(200);
}

void matlab_send(float dato1, float dato2){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);

  //etc con mas datos tipo float. Tambien podría pasarse como parámetro a esta funcion un array de floats.
}
