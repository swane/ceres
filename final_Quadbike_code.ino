//$steer angle,speed m/s:
//e.g. $10,0.5:

//Returns
//Lat,lon,bearing,act speed, act steer \n
//kinematics: 345 cm diameter at 20 degrees (reported

#include <Wire.h>

#include <NMEAGPS.h>
#include <GPSport.h>



#include <avr/wdt.h>  //For watchdog timer





//Interface pins
#define SPEED_RC 22
#define STEER_RC 23
#define Steer_Pot A0  // Potentiometer on Analogue pin 0
//define sensorPin A1 // (used in theroy test) Pot for m/s
#define Motor_PWM 2 // PWM pin to control the Motor
#define STEER_PIN 3
#define HEART 6
#define SIG 5
#define RC_CONTROL 7

//Program constants
#define SER_MON 0  //Debug mode


#define MOTOR_TIME 100      //Interrupt time to send new data to steer and speed motors
#define LCD_TIME 500        //Time between LCD refresh
#define PC_WATCHDOG_TIME 1000  //Timeout to stop robot if no command rcvd
#define PC_COM_TIME 100     //Updates to the PC in ms
#define TIMEOUTPERIOD 8000             // You can make this time as long as you want,
// it's not limited to 8 seconds like the normal
// watchdog
#define CMPI2C_Address 0x60   // Defines address of CMPS10

//Variables
//Global variables to send to PC
//From Radio transmitter
float RC_steer_angle, RC_speed_val;
//From PC
float PC_steer, PC_speed;

float current_speed, current_angle;
//Values to send to motor driver
float motor_driver_steer = 0, motor_driver_speed = 0;


//GPS
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
//Compass
double bearing;

//GUI
bool RC_control = HIGH;
String stringOne = "Top LCD";
String stringTwo = "Bottom LCD";



//Data from PC Serial
char readString[100];
char c;
int pos;
int reading;
//Counters for interrupt timers
unsigned long pc_watchdog_drive = 0, led_drive = 0, lcd_drive = 0, pc_com = 0, serial_led_count = 0;
bool pc_watch_triggered = false;
unsigned long motor_intrpt = 0;
bool ledstate = LOW;
bool serial_led = LOW;


//WDT
unsigned long resetTime = 0;

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





void clear_LCD() {
  Serial3.write(0xFE); Serial3.write(0x01); //Clear LCD
}
void nextline_LCD() {
  Serial3.write(0xFE); Serial3.write(192); //Next LCD Line
}


void self_test()
{
  clear_LCD();
  Serial3.print("Start self test");
  delay(2000);
  clear_LCD();
  Serial3.print("Steer test");
  nextline_LCD();
  Serial3.print("Steer right");

  for (int i = 0; i < 50; i++)
  {
    Steer(10);
    delay(100);
  }
  clear_LCD();
  Serial3.print("Steer test");
  nextline_LCD();
  Serial3.print("Steer straight");

  for (int i = 0; i < 50; i++)
  {
    Steer(0);
    delay(100);
  }
  clear_LCD();
  Serial3.print("Steer OK");
  delay(1000);
  digitalWrite(HEART, 1);
  digitalWrite(SIG, 1);
  clear_LCD();
  Serial3.print("LEDs on");

  delay(2000);
  digitalWrite(HEART, 0);
  digitalWrite(SIG, 0);

}
void setup() {

  disablewatchdog();
  Serial.begin(57600); // Opens serial communication up
  Serial3.begin(9600);
  Wire.begin(); // Starts I2C communication
  delay(100); // Delays the start of the Loop form running


  pinMode(Motor_PWM, OUTPUT);
  pinMode(2, OUTPUT);
  analogWrite(Motor_PWM, 0); // Makes the motor start off
  pc_watchdog_drive = millis();

  motor_intrpt = 0;
  if (SER_MON) Serial.println("Done");
  pinMode(SPEED_RC, INPUT);
  pinMode(STEER_RC, INPUT);
  pinMode(RC_CONTROL, INPUT_PULLUP);
  pinMode(HEART, OUTPUT);
  pinMode(STEER_PIN, OUTPUT);
  pinMode(SIG, OUTPUT);
  RC_control = digitalRead(RC_CONTROL);//RC_control=false;
  clear_LCD();
  if (RC_control) stringOne = "RC Control"; else stringOne = "Serial Control"; Serial3.print(stringOne);
  gpsPort.begin(9600);
  clear_LCD();
  Serial3.print("Initialised OK");
  delay(100);
  //self_test();
  watchdogSetup();

}

void send_data_PC()
{
  //Returns
  //Lat,lon,bearing,act speed, act steer \n
  Serial.print("$");
  Serial.print(fix.latitude(), 6); Serial.print(",");
  Serial.print(fix.longitude(), 6); Serial.print(",");
  Serial.print(bearing); Serial.print(",");
  Serial.print(current_angle); Serial.print(",");
  Serial.print(current_speed); Serial.println();
}
double read_compass(void)
{
  byte highByte, lowByte, fine;              // highByte and lowByte store high and low bytes of the bearing and fine stores decimal place of bearing
  double bearing;                               // Stores full bearing

  Wire.beginTransmission(CMPI2C_Address);           //starts communication with CMPS10
  Wire.write(2);                             //Sends the register we wish to start reading from
  Wire.endTransmission();

  Wire.requestFrom(CMPI2C_Address, 2);              // Request 4 bytes from CMPS10
  int timeout = 0;
  while ((Wire.available() < 2) && (timeout < 50000))
    timeout ++;  // Wait for bytes to become available or timeout
  if (timeout < 50000)
  {
    highByte = Wire.read();
    lowByte = Wire.read();
    bearing = ((highByte << 8) + lowByte) / 10; // Calculate full bearing
  } else {
    bearing = 999.9; // a rogue value
  }

  return bearing;
}

void read_RC() {
  int duration1, duration2;
  duration1 = pulseIn(SPEED_RC, HIGH, 30000UL); //30 ms timeout
  duration2 = pulseIn(STEER_RC, HIGH, 30000UL);
  // Serial.println(duration2);
  if ((duration1 == 0) || (duration2 == 0)) {
    stringTwo = "NO SIGNAL";
    RC_steer_angle = 0; RC_speed_val = 0;

  } else
  {
    RC_steer_angle = -((float)duration2 - 1475) / 425 * 30;
    RC_speed_val = ((float)duration1 - 1420) / 414 * -1;
    /* if (SER_MON)
      {
      Serial.print(duration1);
      Serial.print(" , ");
      Serial.print(duration2);
       Serial.print(" : ");
        Serial.print(steer_angle);Serial.print(" , ");
         Serial.print(speed_val);
        Serial.println();
      }*/
    if (RC_steer_angle > 20) RC_steer_angle = 20;
    if (RC_steer_angle < -20) RC_steer_angle = -20;
    if (RC_speed_val < 0) RC_speed_val = 0;
    stringTwo = "Sr:" + String(RC_steer_angle, 0) + ", Sd:" + String(RC_speed_val, 1);
  }
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

void read_steerangle() {
  int POT_Value = analogRead(Steer_Pot);  // Reads linear actuators internal potentiometer

  double Filtered_POT;
  Filtered_POT = filter((double)POT_Value, 0.8);

  current_angle = map(Filtered_POT, 0, 1018, -30, 30); // Maps the potentiometers value to the vehicles steer angle
  // return current_angle; // Returns the Current steer angle to the function
}

void Steer(double angle) {  // Function to contol steering


  //<--Closed-Loop System-->//
  double K = 10; // Gain for the closed loop
  if (angle>20) angle=20;
  if (angle<-20) angle=-20;
  read_steerangle(); // Gets the current angle of the steering and call it current angle
  double error = angle - current_angle; // A Closed-Loop System to create an error

  //<--Actuators direction-->//
  double errorB;
  errorB = (error) * K;
  if (errorB > 127) errorB = 127;
  if (errorB < -127) errorB = -127;
  int e = (int)errorB;

  e = e + 128;
  //e=255-e;
  /*  Serial.print("Desired angle:");
    Serial.print(angle);
    Serial.print("  Current angle:");
    Serial.print(current_angle);
    Serial.print(",");
    Serial.println(e);*/
  analogWrite(STEER_PIN, e);

}


void Speed(double sp)
{
  byte spdb;
  double spd2;
  spd2 = sp * 255;
  if (spd2 > 255) spd2 = 255;
  spdb = (byte) spd2;
  analogWrite(Motor_PWM, spdb);
  current_speed = sp;
}



int string_split(char input[], char *delimiter) //Used for parsing Serial. from PC
//int string_split(String input,char *delimiter)
/*Returns string split in global variable:
  char str_split_result[20][15]; //20 strings of up to 15 characters each
  first item is in str_split_result[0]
  and value returned = number of strings split
*/

{ //$steer,speed,led1,led2,led3:
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

  PC_steer = strtod(str_split_result[0], NULL);
  PC_speed = strtod(str_split_result[1], NULL);
   if (PC_steer > 20) PC_steer = 20;
    if (PC_steer< -20) PC_steer = -20;


}


void loop()
{



  doggieTickle();  // if you uncomment this line, it will keep resetting the timer.


  if (RC_control != digitalRead(RC_CONTROL))
  {
    RC_control = digitalRead(RC_CONTROL);
    if (RC_control) stringOne = "RC Control"; else stringOne = "Serial Control";
    clear_LCD();  //Update to LCD immediately switch is thrown
    Serial3.print(stringOne);
    nextline_LCD();
    Serial3.print(stringTwo);
  }

  if (RC_control)
  {
    
    read_RC();
    motor_driver_steer = RC_steer_angle;
    motor_driver_speed = RC_speed_val;

  }

  if (!RC_control)  //Serial data defines steer and speed
  {
    if (((millis() - pc_watchdog_drive) > PC_WATCHDOG_TIME) && (!pc_watch_triggered))
    {
      motor_driver_speed = 0;
      Speed( motor_driver_speed);
      pc_watch_triggered = true;
      stringTwo = "TIMED OUT";
    }
    if (Serial.available() > 0)
    {
      c = Serial.read();  //gets one byte from serial buffer
      serial_led = HIGH; serial_led_count = millis() + 500;
      if (c == '$') {
        reading = 1;
        pos = 0;
      }

      if ((c != ':') && (c != '$')) {
        readString[pos] = c;  //makes the string readString
        pos++;
      }
      if (pos > 15)
      {
        reading = 0;
        pos = 0;
        c = 0;
      }
      if (c == ':') //End of command
      {
        readString[pos] = '\0';
        int n = string_split(readString, ",");
        Serial.flush();
        pc_watchdog_drive = millis();
        pc_watch_triggered = false;
        pos = 0;
        c = 0;
        reading = 0;
        stringTwo = "Sr:" + String(PC_steer, 0) + ", Sd:" + String(PC_speed, 1);
        motor_driver_steer = PC_steer;
        motor_driver_speed = PC_speed;
      }

    }
  }

  if ((millis() - motor_intrpt) > MOTOR_TIME)
  {
    Steer(motor_driver_steer);
    Speed(motor_driver_speed);
    motor_intrpt = millis();
  }
  if ((millis() - led_drive) > 500)
  {
    led_drive = millis();
    ledstate = !ledstate;

    digitalWrite(HEART, ledstate);
  }
  if (serial_led)
  {
    digitalWrite(SIG, HIGH);
    if (millis() > serial_led_count)
    {
      serial_led = false;
      digitalWrite(SIG, LOW);
    }
  }
  if ((millis() - lcd_drive) > LCD_TIME)
  {
    lcd_drive = millis();
    clear_LCD();
    Serial3.print(stringOne);
    nextline_LCD();
    Serial3.print(stringTwo);
  }

  //GPS

  if (gps.available( gpsPort )) fix = gps.read();

  if ((millis() - pc_com) > PC_COM_TIME)
  {
    pc_com = millis();
    bearing = read_compass();
    send_data_PC();
  }
}

