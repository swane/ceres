#include <SoftwareSerial.h>

//$steer,speed,buzz: e.g $15.0,0.0,1:
//PID speed tuning: #P,I,D:  e.g. #20,0.1,1:
//Need updaing to $odos,steers,spds,lts,lns,heads,batts,errors:
//at 56700 baud


//External switch RC / PC mode
//Wheel odometry is 5 pulses per revolution
//Measured over 50m (313pulses) this is 6.25 pulses per metre
#include <Wire.h>

#include <avr/wdt.h>  //For watchdog timer


//$steer angle,speed m/s:
//e.g. $10,0.5:steer





//Interface pins

#define Steer_Pot A0  // Steer potentiometer on Analogue pin 
//define STEER_PIN 3
#define HALL_SENSOR 2
#define HEART_LED 3
#define STEER_RC_PIN 4
#define Motor_PWM 5
#define SPEED_RC_PIN 6
#define RC_SWITCH 7
#define BUZZER_PIN 8
#define LCD_TX 12

SoftwareSerial lcd(13, LCD_TX); // RX, TX




//Program constants
#define SER_MON 0  //Debug mode
//Timings
#define MOTOR_TIME 50      //Interrupt time to send new data to steer and speed motors
#define LCD_TIME 500        //Time between LCD refreshhttps://www.msn.com/en-gb/feed
#define PC_WATCHDOG_TIME 1000  //Timeout to stop robot if no command rcvd
#define PC_COM_TIME 500     //Updates to the PC in ms
#define CALC_SPEED 1000     //How often to calculate the speed
#define TIMEOUTPERIOD 8000             // You can make this time as long as you want,


#define BUZZ_TIME 1000

//Wheel speed
//distance per pulse, 6.25 pulses per metre = 0.16m per pulse
#define wheel_ppd 0.16
unsigned int wheel = 0;
int wheele = 0, prevwheel = 0;
double wheelspeed = 0;
int steer_pot;

#define MIN_STEER_POT   100
#define MAX_STEER_POT   800

//PID Steer
double steer_k = 20;
double steer_i = 0.1;
double steer_d = 1.0;

//PID Speed
double ks = 30;
double kis = 10;
double kds = 0;
//Variables
//Global variables to send to PC
//From Radio transmitter
float RC_steer_angle, RC_speed_val;
//From PC
float PC_steer, PC_speed;
float buzz;
float squirt;
float current_speed, current_angle;
//Values to send to motor driver
float motor_driver_steer = 0, motor_driver_speed = 0;


String stringOne, stringTwo;

//MD04 set up
#define MAXTEMP              30                     //Max temp before error
#define ACCEL               25                      //acceleration (0-max, 255-min which speeds up over 8 secs
#define MAXCURRENT          0.1

#define ADDRESS             0x58                    // Address of MD03
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define ACCELREG            0x03                    // Set acceleration (0-max, 255-min which speeds up over 8 secs
#define TEMPREG             0x04                    // Byte to read temprature
#define CURRENTREG          0x05                    // Byte to read motor current

struct error_struct {
  bool MD04_comms;
  bool MD04_overtemp;
  bool MD04_overcurrent;
  bool MD04_minpos;
  bool MD04_maxpos;
};

error_struct error;
bool globalerror;

//GUI
bool RC_control = HIGH;
bool commandstate = 0; //0-Command state, 1-PID tuning mode

//Data from PC Serial
char readString[100];
char c;
int pos;
int reading;
//Counters for interrupt timers
unsigned long pc_watchdog_drive = 0, led_drive = 0, lcd_drive = 0, pc_com = 0, serial_led_count = 0, calcspeed = 0;
unsigned long buzz_timer = 0;
bool pc_watch_triggered = false;
unsigned long motor_intrpt = 0;
byte ledstate = 0;
bool serial_led = LOW;

bool BB = true;
//WDT
unsigned long resetTime = 0;
unsigned long LED_FLASH = 500;
bool buzzstate = false;
/* DISPLAY**/


void clear_LCD() {
  lcd.write(0xFE); lcd.write(0x01); //Clear LCD
}
void nextline_LCD() {
  lcd.write(0xFE); lcd.write(192); //Next LCD Line
}


void lcddisplay(String topLine, String bottomLine ) {

  static String lastMessage = "";
  static String lastMessage1, lastMessage2;
  if ((topLine != lastMessage1) || (bottomLine != lastMessage2)) {

    clear_LCD();
    lcd.print(topLine.substring(0, 16));
    nextline_LCD();
    lcd.print(bottomLine.substring(0, 16));

    lastMessage1 = topLine;
    lastMessage2 = bottomLine;
  }
}

/****RC CONTROL *****/

double RC_speed() {

  double spd;
  unsigned long t;

  t = millis();
  while ((!digitalRead(SPEED_RC_PIN)) && (millis() - t < 50))
  {

  }
  t = micros();

  while ((digitalRead(SPEED_RC_PIN)) && (micros() - t < 5000))
  {

  }
  unsigned long d = micros() - t;

  spd = 1 - (((double)d - 1000) * 2 / (1000) ); //Range -1 - 0 - +1
  if (spd > 1.5) spd = 2.0;
  return spd;
}
double RC_speedo()
{
  noInterrupts();
  unsigned long duration = pulseIn(SPEED_RC_PIN, HIGH, 25000);
  interrupts();
  //Serial.println(duration);
  double d = (double)duration;
  d = d - 1263;
  d = d / (235);
  d = d - 1.0;
  return -d;
}




double RC_steero()
{
  noInterrupts();
  unsigned long duration = pulseIn(STEER_RC_PIN, HIGH, 25000);
  interrupts();
  //Serial.println(duration);
  double d = (double)duration;
  d = d - 1263;
  d = d / (235);
  d = d - 1.0;
  return -d;
}
double RC_steer() {

  double spd;
  unsigned long t, b;

  t = millis();
  b = millis();
  while ((!digitalRead(STEER_RC_PIN)) && (b - t < 50))
  {
    b = millis();
  }
  if (b - t < 50)
  {
    t = micros();

    while ((digitalRead(STEER_RC_PIN)) && (micros() - t < 5000))
    {

    }
    unsigned long d = micros() - t;

    spd = (1 - (((double)d - 1000) * 2 / (1000) ))*3.5; //Range -1 - 0 - +1
    if (spd > 1.5) spd = 2.0;
  } else spd = 2.0;
  return spd;
}


void check_RC_switch()
{
  //Check switch
  if ((digitalRead(RC_SWITCH)) && !RC_control)
  {
    stringOne = "RC Mode";
    RC_control = HIGH;
    lcd_drive = 0; //Allows immediate showing of data
  }
  if ((!digitalRead(RC_SWITCH)) && RC_control)
  {
    stringOne = "Serial Mode";
    RC_control = LOW;
    lcd_drive = 0; //Allows immediate showing of data
  }
}

/***SELF TEST****/
void self_test()
{
  clear_LCD();
  lcd.print("Start self test");
  delay(2000);
  clear_LCD();
  lcd.print("Steer test");
  nextline_LCD();
  lcd.print("Steer right");

  for (int i = 0; i < 50; i++)
  {
    Steer(10);
    delay(100);
  }
  clear_LCD();
  lcd.print("Steer test");
  nextline_LCD();
  lcd.print("Steer straight");

  for (int i = 0; i < 50; i++)
  {
    Steer(0);
    delay(100);
  }
  clear_LCD();
  lcd.print("Steer OK");
  delay(1000);
  digitalWrite(HEART_LED, 1);

  clear_LCD();
  lcd.print("LEDs on");

  delay(2000);
  digitalWrite(HEART_LED, 0);


}

void setup1()
{
    pinMode(HEART_LED, OUTPUT);
      pinMode(HALL_SENSOR, INPUT_PULLUP);
      while(1) digitalWrite(HEART_LED,digitalRead(HALL_SENSOR));
}

void setup() {

  // disablewatchdog();
  Serial.begin(57600); // Opens serial communication up

  Wire.begin(); // Starts I2C communication
  delay(100); // Delays the start of the Loop form running

  lcd.begin(9600);

  pc_watchdog_drive = millis();

  motor_intrpt = 0;

  // pinMode(Motor_PWM, OUTPUT);
  pinMode(RC_SWITCH, INPUT_PULLUP);
  pinMode(HEART_LED, OUTPUT);

  pinMode(STEER_RC_PIN, INPUT);
  pinMode(SPEED_RC_PIN, INPUT);



  pinMode(Motor_PWM, OUTPUT);

  analogWrite(Motor_PWM, 0); // Makes the motor start off
  RC_control = digitalRead(RC_SWITCH);//RC_control=false;
  clear_LCD();
  if (RC_control) stringOne = "RC Control"; else stringOne = "Serial Control"; lcddisplay(stringOne, "");

  clear_LCD();
  lcddisplay("Initialised OK", "");
  delay(100);
  //self_test();
  // watchdogSetup();

  error.MD04_comms = false;
  error.MD04_overtemp = false;
  error.MD04_overcurrent = false;
  sendData(ACCELREG, ACCEL);
  Serial.println("Setup done");
  lcddisplay("Ceres Robot", "");
  // delay(200);
  Speed(0);
  // led_out(2, HIGH);
  //HALL
  pinMode(HALL_SENSOR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), countWheel, FALLING);
  if (SER_MON) Serial.println("Done");
  if ((digitalRead(RC_SWITCH)))
  {
    stringOne = "RC Mode";
    RC_control = HIGH;
    //clear_LCD();
    lcddisplay(stringOne, ""); delay(500);
  }
  if ((!digitalRead(RC_SWITCH)))
  {
    stringOne = "Serial Mode";
    RC_control = LOW;
    lcddisplay(stringOne, ""); delay(500);
  }
  lcd_drive = 0;
  wheel=0;

}



byte getData(byte reg) {                  // function for getting data from MD03
  unsigned long m;
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS, 1);         // Requests byte from MD03

  m = millis();
  while ((Wire.available() < 1) && (millis() - m < 2));   // Waits for byte to become availble
  if (Wire.available() < 1)  {
    error.MD04_comms = true; //comms error on I2C
  }
  byte data = Wire.read();
  return (data);
}






#define doggieTickle() resetTime = millis();  // This macro will reset the timer
void(* resetFunc) (void) = 0; //declare reset function @ address 0



void watchdogSetup()
{
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
  MCUSR &= ~(1 << WDRF); // because the data sheet said to
  /*
    WDTCSR configuration:
    WDIE = 1 :Interrupt Enable
    WDE = 1  :Reset Enable - I won't be using this on the 2560
    WDP3 = 0 :For 1000ms Time-out
    WDP2 = 1 :bit pattern is
    WDP1 = 1 :0110  change this for a different
    WDP0 = 0 :timeout period.
  */
  // Enter Watchdog Configuration mode:
  WDTCSR = (1 << WDCE) | (1 << WDE);
  // Set Watchdog settings: interrupte enable, 0110 for timer
  WDTCSR = (1 << WDIE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);
  sei();
  //Serial.println("finished watchdog setup");  // just here for testing
}

void disablewatchdog()
{
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
  MCUSR &= ~(1 << WDRF); // because the data sheet said to
  /*
    WDTCSR configuration:
    WDIE = 1 :Interrupt Enable
    WDE = 1  :Reset Enable - I won't be using this on the 2560
    WDP3 = 0 :For 1000ms Time-out
    WDP2 = 1 :bit pattern is
    WDP1 = 1 :0110  change this for a different
    WDP0 = 0 :timeout period.
  */
  // Enter Watchdog Configuration mode:
  WDTCSR = (1 << WDCE) | (0 << WDE);
  // Set Watchdog settings: interrupte enable, 0110 for timer
  WDTCSR = (0 << WDIE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);
  sei();

}

ISR(WDT_vect) // Watchdog timer interrupt.
{
  if (millis() - resetTime > TIMEOUTPERIOD) {
    //Serial.println("This is where it would have rebooted");  // just here for testing
    // doggieTickle();                                          // take these lines out
    resetFunc();     // This will call location zero and cause a reboot.
  }
}
/*LCD on Serial3.
  Used eg
  clear_LCD();
    Serial3.print("Harper Adams Uni");
    nextline_LCD();
    Serial3.print("Test LCD");
    delay(1000);
    clear_LCD();
    Serial3.print("Harper Adams Uni");
*/


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

double Steer_PID(double Kp, double Ki, double Kd, double error)
{

  static double lastmillisp;
  static double integError = 0, lastError;
  double dt;
  unsigned long tt;
  double compP, compI, compD;

  dt = (millis() - lastmillisp) / 1000.00;
  integError += error * dt;
  if ((error > -0.5) && (error < 0.5)) integError = 0.0; //Stop wind-up
  compP = error * Kp;
  compI = integError * Ki;
  compD = ((error - lastError) / dt) * Kd;
  lastError = error;
  lastmillisp = millis();
  double totpid = compP + compI + compD;
  if ((error > -2) && (error < 2)) totpid = 0.0;
  return totpid;
}


/* STEER CONTROLLER  */

void sendData(byte reg, byte val) {        // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS);         // Send data to MD03
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
/*
   Returns a value starting at theta_s and ending at theta_e over the time period tf
   Can be used to accelerate a motor speed where theta is the desired speed
   It resets the internal clock each time the end position (theta_e) changes
   Tried to make it a 'set and forget' function
   ang=theta(ts, te, 5);
*/

double theta(double theta_s, double theta_e, double tf)
{
  double a0, a1, a2, a3;
  static unsigned long ts;
  static double endt;
  static double thet;
  double t;

  if (endt != theta_e)
  {
    ts = millis();
    endt = theta_e;
  }
  a0 = theta_s;
  a1 = 0;
  a2 = 3 / (tf * tf) * (theta_e - theta_s);
  a3 = -2 / (tf * tf * tf) * (theta_e - theta_s);
  t = (double)(millis() - ts) / 1000;


  if (t <= tf)
  {

    thet = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  }

  return thet;
}


void move_steer(byte spd, byte dir)
{
  //dir=1 steer left, dir=2 steer right
  sendData(SPEEDBYTE, spd);             // Sets speed to i
  sendData(CMDBYTE, dir);          // Sets motor to direct, a value of 1 runs the motor forward and 2 runs backward
}


void check_MD04()
{
  error.MD04_comms = false;
  error.MD04_overtemp = false;
  error.MD04_overcurrent = false;
  error.MD04_minpos = false;
  error.MD04_maxpos = false;
  int temp = getData(TEMPREG);        // Gets temperature
  if (temp > MAXTEMP) error.MD04_overtemp = true; else error.MD04_overtemp = false;
  byte current = getData(CURRENTREG);  // Gets motor current
  float crt = float(float(current) / 186.0);
  if (crt > MAXCURRENT) error.MD04_overcurrent = true; else error.MD04_overcurrent = false;
  /* Serial.print("temprature: ");
    Serial.print(temp);                 // Prints temperature to LCD03
    Serial.print("  ");                 // Prints spaces to clear space after data


    Serial.print("Motor current: ");
    Serial.print(crt, 6);
    Serial.println("   ");*/
  if (steer_pot > MAX_STEER_POT) error.MD04_maxpos = true;
  if (steer_pot < MIN_STEER_POT) error.MD04_minpos = true;
  if (error.MD04_comms) stringOne = "Error MD04 comms";
}
int read_steerpot() {
  int POT_Value = analogRead(Steer_Pot);  // Reads linear actuators internal potentiometer
  return POT_Value; // Returns the Current steer angle to the function
}

void read_steerangle() //and filters
{
  int i = read_steerpot();
  double angle = map(i, 0, 1018, -30, 30); // Maps the potentiometers value to the vehicles steer angle

  current_angle = filter((double)angle, 0.8);
  //current_angle = (double)angle;
}

void Steer(double angle) {  // Function to contol steering

  byte dir;
  //<--Closed-Loop System-->//

  if (angle > 20) angle = 20;
  if (angle < -20) angle = -20;
  read_steerangle(); // Gets the current angle of the steering and call it current angle
  int i = read_steerpot();


  double errora = angle - current_angle; // A Closed-Loop System to create an error
  //Serial.print("Error: ");Serial.println(error);
  errora = Steer_PID(steer_k, steer_i, steer_d, errora);
  if (errora < 0) {
    dir = 1;
    errora = -errora;
  } else dir = 2;


  if (errora > 243) errora = 243;
  byte steerval = (byte)errora;


  //Only steer if no error, or if error is minpos and steer is right, or if error is maxpos and steer is left
  /* if ((!globalerror) ||
       ((globalerror) && (error.MD04_minpos) && (dir == 2)) ||
       ((globalerror) && (error.MD04_maxpos) && (dir == 1)))*/
  move_steer(steerval, dir);

}
void Steerorig(double angle) {  // Function to contol steering

  byte dir;
  //<--Closed-Loop System-->//

  if (angle > 20) angle = 20;
  if (angle < -20) angle = -20;
  read_steerangle(); // Gets the current angle of the steering and call it current angle
  int i = read_steerpot();
  /* if ((i < 30) || (i > 830))
    {
     //Error steer out of range
     sendData(SPEEDBYTE, 0);             // Sets speed to i
     sendData(CMDBYTE, 1);
    lcddisplay("*Steer error!*", "*Out of Range!*");
     delay(500);
    } else*/

  double errora = angle - current_angle; // A Closed-Loop System to create an error
  //Serial.print("Error: ");Serial.println(error);
  if (errora < 0) {
    dir = 1;
    errora = -errora;
  } else dir = 2;
  //errora = errora * K;
  errora = Steer_PID(steer_k, steer_i, steer_d, errora);
  if (errora > 243) errora = 243;
  byte steerval = (byte)errora;


  //Only steer if no error, or if error is minpos and steer is right, or if error is maxpos and steer is left
  /* if ((!globalerror) ||
       ((globalerror) && (error.MD04_minpos) && (dir == 2)) ||
       ((globalerror) && (error.MD04_maxpos) && (dir == 1)))*/
  move_steer(steerval, dir);

}

double Speed_PID(double Kp, double Ki, double Kd, double error, bool res)
{

  static double lastmillisp;
  static double integError = 0, lastError;
  double dt;
  unsigned long tt;
  double compP, compI, compD;

  dt = (millis() - lastmillisp) / 1000.00;
  if (res) integError = 0; else integError += error * dt;

  compP = error * Kp;
  compI = integError * Ki;
  compD = ((error - lastError) / dt) * Kd;
  lastError = error;
  lastmillisp = millis();

  double totpid = compP + compI + compD;
  if (res) totpid = 0;
  // if ((error>-3)&&(error<3)) totpid=0.0;
  return totpid;
}
void Speed(double sp)
{
  static bool rev = false;
  byte spdb;
  double spp;
  static double spd2 = 0;
  double error;
  //if (sp >= 0) {
  //if (rev) {analogWrite(Motor_PWM, 0);delay(500);}
  //digitalWrite(reverse_relay,LOW);rev=false;}
  //if (sp<0.2) spd2=0;
  error = sp - wheelspeed;
  //  Serial.print("error:");Serial.println(error);
  //PID
  //spd2=spd2+ error*5;//
  spd2=Speed_PID(ks,kis,kds,error,false);
  //spd2 = sp * 255;
  if (spd2 > 255) spd2 = 255;
  spp = spd2;
  if (spp < 0) spp = 0;
  spdb = (byte) spp;
  //Serial.print("Speed:");Serial.print(wheelspeed);Serial.print("  PWM:");Serial.println(spdb); //Testing PID
  //Serial.print("wheel:");  Serial.print(wheelspeed);Serial.print("sp:");  Serial.print(sp);Serial.print("error:");  Serial.print(error);
  //  Serial.print("spdb:");  Serial.print(spdb); Serial.print("spd2:");Serial.println(spd2);

  analogWrite(Motor_PWM, spdb);

  //current_speed = sp;
  //}
  if (sp < 0) {//commented out as there is no reverse
    sp = 0;
    Speed_PID(0, 0, 0, error, true);
    analogWrite(Motor_PWM, 0);
    spd2 = 0;
  }





}




void send_data_PC()
{
  int wheele = read_wheel();


  //Returns
  //odom,steerangle,act speed, Lat,lon,bearing \n
  //Need $odos,steers,spds,lts,lns,heads,batts,errors \n
  int batt = 0;
  int errors = 0;
  Serial.print("$");
  //Serial.print((double)(wheele) * 0.2025); Serial.print(",");
  Serial.print(wheele); Serial.print(",");
  Serial.print(current_angle); Serial.print(",");
  Serial.print(wheelspeed); Serial.print(",");
  Serial.print(batt);  Serial.print(",");
  Serial.print(errors);  Serial.println();
}




int string_split(char input[], char *delimiter) //Used for parsing Serial. from PC
//int string_split(String input,char *delimiter)
/*Returns string split in global variable:
  char str_split_result[20][15]; //20 strings of up to 15 characters each
  first item is in str_split_result[0]
  and value returned = number of strings split
*/

{ //$steer,speed:
  char str_split_result[20][15];
  char *token;
  int n = 0;

  // double spd; //speed is reserved

  token = strtok(input, delimiter);
  while (token != NULL)
  {
    strcpy(str_split_result[n], token);
    n++;
    token = strtok(NULL, delimiter);
  }
  if (commandstate == 0)
  {
    PC_steer = strtod(str_split_result[0], NULL);
    PC_speed = strtod(str_split_result[1], NULL);
    buzz = strtod(str_split_result[2], NULL);
    squirt = strtod(str_split_result[3], NULL);
  }
  if (commandstate == 1)
  {

    ks = strtod(str_split_result[0], NULL);
    kis = strtod(str_split_result[1], NULL);
    kds = strtod(str_split_result[2], NULL);
    Serial.println("PID values updated");
    Serial.print("K : "); Serial.print(ks); Serial.print(", I : "); Serial.print(kis); Serial.print(", D : "); Serial.println(kds); delay(500);
  }
}

double filtersteer(double input, double f)
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

void buzzer_on()
{
  digitalWrite(BUZZER_PIN, HIGH);
}
void buzzer_off()
{
  digitalWrite(BUZZER_PIN, LOW);
}
void countWheel() //HALL SENSOR interrupt
{
  if (!digitalRead(HALL_SENSOR)) wheel++;
}

void zero_wheel()
{
  wheel = 0;
}
unsigned long read_wheel()
{
  return wheel;
}


void check_error()
{
  static bool errorledstate = false;

  String eline1, eline2;
  globalerror = false;
  eline1 = ""; eline2 = "";
  //Check for any error
  if (
    (error.MD04_comms == true) ||
    (error.MD04_overtemp == true) ||
    (error.MD04_overcurrent == true) ||
    (error.MD04_minpos == true) ||
    (error.MD04_maxpos == true)) globalerror = true;
  if (globalerror)                    eline1 = "Error: Motor Drv";
  if (error.MD04_comms == true)       eline2 = "No i2c comms    ";
  if (error.MD04_overtemp == true)    eline2 = "Over temperature";
  if (error.MD04_overcurrent == true) eline2 = "Over current    ";
  if (error.MD04_minpos == true)      eline2 = "Min steer pos   ";
  if (error.MD04_maxpos == true)      eline2 = "Max steer pos   ";
  //lcddisplay("Error in motor driver");
  lcddisplay(eline1, eline2);
  // if (globalerror) led_out(ERROR_LED, errorledstate); errorledstate = !errorledstate;
}
void loop4() {
  unsigned long duration;
  noInterrupts();
  duration = pulseIn(4, HIGH);
  interrupts();
  Serial.println(duration);
}
void loop6()
{
  double d;
  d = RC_steer();
  Serial.println(d);//1263 - 1733
  delay(100);
}

int n = 0;
void loop()
{



  // doggieTickle();  // if you uncomment this line, it will keep resetting the timer.
  // if (!digitalRead(right_button)) self_test();
  // check_MD04();



  if (!RC_control)  //Serial data defines steer and speed
  {
    //Comment out below to test PID
    if (((millis() - pc_watchdog_drive) > PC_WATCHDOG_TIME) )  //Time out
    {
      motor_driver_speed = 0;
      Speed( motor_driver_speed);
      pc_watch_triggered = true;
      stringTwo = "TIMED OUT";
      // led_out(2, HIGH);
    }
  }
  if (Serial.available() > 0)
  {
    c = Serial.read();  //gets one byte from serial buffer
    if (c == '$') {
      reading = 1;
      pos = 0;
      commandstate = 0; //Usual steer,speed command
    }
    if (c == '#') {
      reading = 1;
      pos = 0;
      commandstate = 1; //PID tuning mode
    }
    if ((c != ':') && (c != '$') && (c != '#')) {
      readString[pos] = c;  //makes the string readString
      pos++;
    }
    if (c == ':')
    {
      readString[pos] = '\0';
      int n = string_split(readString, ",");
      Serial.flush();
      pc_watchdog_drive = millis();
      pc_watch_triggered = false;
      pos = 0;
      c = 0;
      reading = 0;
      // led_out(2, LOW);
      stringTwo = "Sr:" + String(PC_steer, 0) + ", Sd:" + String(PC_speed, 1);
      if (!RC_control)
      {
        motor_driver_steer = PC_steer;
        motor_driver_speed = PC_speed;
      }
      if (buzz > 0) {
        buzzstate = true;
        buzz_timer = millis();
        buzzer_on();
      }


    }

  }



  if ((millis() - calcspeed) > CALC_SPEED)
  {
    wheele = read_wheel();
    //Serial.println(wheele);
    wheelspeed = (double)((wheele - prevwheel)) * (wheel_ppd);
    prevwheel = wheele;
    calcspeed = millis(); //zero_wheel()
    //Serial.println(wheelspeed);
            stringOne = "IN:ST:" + String(motor_driver_steer, 0) + " SP:" + String(motor_driver_speed, 1);
        //stringTwo = "AC:ST:" + String(current_angle, 0) + " SP:" + String(wheelspeed, 1);
        stringTwo = "W:" + String((wheel*wheel_ppd),2)+ ", S:"+String(wheelspeed, 1);
  }

  if ((millis() - led_drive) > LED_FLASH)
  {
    led_drive = millis();

    switch (ledstate)
    {
      case 0: digitalWrite(HEART_LED, 0); LED_FLASH = 800; ledstate++; break;
      case 1: digitalWrite(HEART_LED, 1); LED_FLASH = 200; ledstate++; break;
      case 2: digitalWrite(HEART_LED, 0); ledstate++; break;
      case 3: digitalWrite(HEART_LED, 1); ledstate++; break;
      case 4: ledstate = 0; break;
    }
  }



  if (buzzstate)
  { if ((millis() - buzz_timer) > BUZZ_TIME)
    {
      buzzer_off();
    }
  }
  if ((millis() - pc_com) > PC_COM_TIME)
  {
    pc_com = millis();

    send_data_PC();
  }
  if ((millis() - motor_intrpt) > MOTOR_TIME)
  {
    if (RC_control)
    {

      double s = RC_steer();
      //Serial.println(s);
      if (s < 2.0) //2=no signal
      {
        motor_driver_steer = -s * 30;
        // motor_driver_steer = filtersteer(motor_driver_steer, 0.5);
        //Serial.print("RC Steer:"); Serial.println(RC_steer());
        motor_driver_speed = RC_speed();
        //if ((motor_driver_speed > -10) && (motor_driver_speed < 0.25)) motor_driver_speed = 0;
        stringOne = "IN:ST:" + String(motor_driver_steer, 0) + " SP:" + String(motor_driver_speed, 1);
        //stringTwo = "AC:ST:" + String(current_angle, 0) + " SP:" + String(wheelspeed, 1);
        stringTwo = "W:" + String((wheel*wheel_ppd),2)+ ", S:"+String(wheelspeed, 1);
      } else
      {
        motor_driver_steer = 0;
        motor_driver_speed = 0;
        stringOne = "RC Mode"; stringTwo = "*NO SIGNAL**";
      }
    }

    Steer(motor_driver_steer);//Reads the steer angle too
//motor_driver_speed=1.0; //Used in Testing PID
    Speed(motor_driver_speed);
    motor_intrpt = millis();
  }
  if ((millis() - lcd_drive) > LCD_TIME)
  {
    lcddisplay(stringOne, stringTwo);
    lcd_drive = millis();

  }
  check_RC_switch();

}
