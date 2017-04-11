#define sel1 38
#define sel2 39
#define rst_ 40
#define clk 12

int prev_alpha1 = 0;
int prev_alpha2 = 0;
int d_alpha1 = 0;
int d_alpha2 = 0;
int k1 = 0;
int k2 = 0;
int k3 = 0;
int k4 = 0;
int output = 0;

int alpha;
byte alpha1, alpha2;

void setup() {
  pinMode(sel1, OUTPUT);
  pinMode(sel2, OUTPUT);
  pinMode(rst_, OUTPUT);
  pinMode(clk, OUTPUT);

  TCCR1B=0x01;
  analogWrite(clk, 127);

  digitalWrite(rst_, LOW);
  delay(1000);
  digitalWrite(rst_, HIGH);

  DDRC=0b00000000;

  Serial.begin(9600);

}

void loop() {
  digitalWrite(sel1, HIGH);
  digitalWrite(sel2, LOW);
  alpha1=PINC;
  digitalWrite(sel1, LOW);
  digitalWrite(sel2, LOW);
  alpha2=PINC;

  // need to divide by some constant?
  curr_micros = micros();
  d_alpha1 = (alpha1 - prev_alpha1) / (0.000001*(curr_micros - prev_micros));
  d_alpha2 = alpha1 - prev_alpha2 / (0.000001*(curr_micros - prev_micros));
  prev_micros = curr_micros;
  prev_alpha1 = alpha1;
  prev_alpha2 = alpha2;

  output = k1*alpha1 + k2*alpha2 + k3*d_alpha1 + k4*d_alpha2;

  // need to give this output somewhere
  alpha=word(alpha2, alpha1);
  Serial.println(alpha);
}
