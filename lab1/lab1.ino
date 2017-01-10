#include <PID_v1.h>


/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum=0, lastErr=0;
double kp, ki, kd;
double error=0;
void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
  
   /*Compute all the working error variables*/
   double e = Setpoint - Input;
   if (e >= 512) {
      error = e-1024;
   } else if (e >= -512 and e < 512) {
      error = e;
   } else {
      error = 1024+e; 
   }
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}

void setup()
{ 
  //initialize the variables we're linked to
  Input = analogRead(0);
  Setpoint = int(Input+512) % 1024;
  //0.05, 0.0001, 0.03
  SetTunings(0.1, 0, 0);
  lastTime = millis();
  Serial.begin(9600);
}

void loop()
{
  Input = analogRead(0);
  Compute();
  if (Output >= 0) {
    analogWrite(3, min(Output, 255));
    analogWrite(4, 0);
  } else {
    analogWrite(4, min(Output, 255));
    analogWrite(3, 0);  
  }
  Serial.print(millis());
  Serial.print(",");
  Serial.println(error);
}
