# PID
```
double Output_PID(double Kp, double Ki, double Kd, double error)
{

  static double lastmillisp;
  static double integError = 0, lastError;
  double dt;
  unsigned long tt;
  double compP, compI, compD;

  dt = (millis() - lastmillisp) / 1000.00;
  integError += error * dt;
  compP = error * Kp;
  compI = integError * Ki;
  compD = ((error - lastError) / dt) * Kd;
  lastError = error;
  lastmillisp = millis();
  return compP + compI + compD;
}
double Output_PID(double Kp, double Ki, double Kd, double error, double maxerror)
{
  static double lastmillisp;
  static double integError = 0, lastError;
  double dt;
  unsigned long tt;
  double compP, compI, compD;

  dt = (millis() - lastmillisp) / 1000.00;
  // if (error>maxerror) integError=0;
  // if (error<-maxerror) integError=0;

  if ((error > -maxerror) && (error < maxerror))integError += error * dt;
  compP = error * Kp;
  compI = integError * Ki;
  compD = ((error - lastError) / dt) * Kd;
  lastError = error;
  lastmillisp = millis();
  return compP + compI + compD;
}

double filter(double input, double f)
{
  static bool initialise = true;
  static double prev_x = 0, prev_y = 0;
  double y;
  double T;
  static unsigned long lastmillis;

  T = (millis() - lastmillis) / 1000.00;

  //f = 0.2; //Cutoff frequency in Hz, small value makes a slow response
  f = f * 2 * PI;
  //if (initialise) y = input; else
  y = f * T * prev_x + prev_y - f * T * prev_y;
  initialise = false;
  prev_x = input;
  prev_y = y;
  lastmillis = millis();
  return y;
}

```
