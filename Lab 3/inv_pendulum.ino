#define sel1 38
#define sel2 39
#define rst_ 40
#define clk 12

float prev_alpha = 0;
float prev_theta = 0;
float d_alpha = 0;
float d_theta = 0;
float k1 = 4.2426;
float k2 = -60.3494;
float k3 = 2.2651;
float k4 = -7.8943;
float output = 0;
const float Pi = 3.14159;

int alpha_z;
int theta_z;
float alpha, theta;
byte alpha1, alpha2;
byte theta1, theta2;
long curr_micros = 0, prev_micros = 0;

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
  DDRA=0b00000000;

  Serial.begin(115200);

}

void loop() {
  digitalWrite(sel1, HIGH);
  digitalWrite(sel2, LOW);
  alpha1=PINC;
  digitalWrite(sel1, LOW);
  digitalWrite(sel2, LOW);
  alpha2=PINC;
  digitalWrite(sel1, HIGH);
  digitalWrite(sel2, LOW);
  theta1=PINA;
  digitalWrite(sel1, LOW);
  digitalWrite(sel2, LOW);
  theta2=PINA;
  // need to give this output somewhere
  alpha_z = word(alpha2, alpha1);
  theta_z = word(theta2, theta1);
  if (alpha_z > 0) {
     alpha = (alpha_z % 2000)*(360.0 / 2000.0);
     if (alpha > 180) alpha = alpha - 360;
  } else {
     alpha = -1*(-1*alpha_z % 2000)*(360.0 / 2000.0);
     if (alpha < -180) alpha = alpha + 360;
  }
  
  if (theta_z > 0) {
     theta = (theta_z % 2000)*(360.0 / 2000.0);
     if (theta > 180) theta = theta - 360;
  } else {
     theta = -1*(-1*theta_z % 2000)*(360.0 / 2000.0);
     if (theta < -180) theta = theta + 360;
  }
  
  // need to divide by some constant?
  curr_micros = micros();
  d_alpha = (alpha - prev_alpha) / ((1e-6)*(curr_micros - prev_micros));
  d_theta = (theta - prev_theta) / ((1e-6)*(curr_micros - prev_micros));
  prev_micros = curr_micros;
  prev_alpha = alpha;
  prev_theta = theta;

  output = (k1*theta + k2*alpha + k3*d_theta + k4*d_alpha)*(Pi / 180.0);
  output = output * 255 / 7.5;
  if (output > 254) output = 254;
  if (output < -254) output = -254;
  Serial.println(int(theta));
  if (output > 0)
  {
    analogWrite(8, output);
    analogWrite(9, 0);
  } else {
    analogWrite(8, 0);
    analogWrite(9, -output);
  }
  delay(50);
}
