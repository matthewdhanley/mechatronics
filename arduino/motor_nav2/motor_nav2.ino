/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>
#include <Servo.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 9
#define AIN2 10
#define PWMA 11

#define PWMB 5
#define BIN1 7
#define BIN2 6

#define STBY 8

#define LIN_SERVO = 2
#define VERT_SERVO = 3


// Libraries for Adafruit BNO055 IMU:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// The absolute maximum this value can be is 250. 100 is a speed it won't
// get away from us.
const int max_speed = -100.0;

// Variable Initialization -- Line Follower
const int offsetA    = 1;
const int offsetB    = 1;
const int TAPE1_PIN  = A1; // Sensor output voltage
const int TAPE2_PIN  = A0; // Sensor output voltage
const int TAPE3_PIN  = A2; // Sensor output voltage

int proximityADC1    = 0;
int proximityADC2    = 0;
int proximityADC3    = 0;
float v_thresh       = 3.0;  // threshold for seeing tape


struct tape_proximity{
  int left = 0;
  int right = 0;
  int front = 0;
};

tape_proximity tape_sensors;
tape_proximity tape_sensors_prev;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//Motor Command from raspberry pi:
struct motor_command {
   int id;
   int motor_speed;
};

struct motor_command cur_motor_cmd1;
struct motor_command cur_motor_cmd2;

void setup()
{
  stop_motors();
  Serial.begin(9600);
  pinMode(TAPE1_PIN, INPUT);
  pinMode(TAPE2_PIN, INPUT);
  pinMode(TAPE3_PIN, INPUT);
//  Serial.println("Starting...")/;
  cur_motor_cmd1.motor_speed = 0;
  cur_motor_cmd2.motor_speed = 0;


  // setup imu:
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  stop_motors();

  imusetup();
  imusetup();
  imusetup();
  imusetup();
  
}

void go_straight(){
  motor1.drive(max_speed);
  motor2.drive(max_speed);
}

void turn_right(){
  motor1.drive(40);
  motor2.drive(-40);
}

void turn_left(){
  motor1.drive(-40);
  motor2.drive(40);
}

void stop_motors(){
  motor1.drive(0);
  motor2.drive(0);
}

void read_tape_sensors(){
  // Read in the ADC and convert it to a voltage:
  proximityADC1 = analogRead(TAPE1_PIN);
  tape_sensors.left = (float)proximityADC1 * 5.0 / 1023.0;

  proximityADC2 = analogRead(TAPE2_PIN);
  tape_sensors.right = (float)proximityADC2 * 5.0 / 1023.0;    

  proximityADC3 = analogRead(TAPE3_PIN);
  proximityADC3 = proximityADC3 - 100;
  tape_sensors.front = (float)proximityADC3 * 5.0 / 1023.0;    

  Serial.println(tape_sensors.front);
//  Serial.println(proximityADC3);
  tape_sensors_prev.left = tape_sensors.left;
  tape_sensors_prev.right = tape_sensors.right; // Twisty Wire
  tape_sensors_prev.front = tape_sensors.front;
}



void get_command_motor()
{

  // Get motor navigation command from raspberry pi:
  if (Serial.available() > 0) {
    
    int id = Serial.parseInt();
    // check to make sure its id not a speed:
    while(id > 10 || id <-4)
    {
      id = Serial.parseInt();
      Serial.println(id);    
    }
    Serial.println(id);
    
    if (Serial.available() > 0) {
      int motor_speed = Serial.parseInt();
        switch (id)
        {
          case 1:
            cur_motor_cmd1.motor_speed = motor_speed;
            break;
          case 2:  
            cur_motor_cmd2.motor_speed = motor_speed;
            break;
          case 5:
            turn_90right();
            break;
          case 6:
            turn_90left();
            break;
          default:
            break;
        }
        Serial.println(motor_speed);        
  }
  }
} 

void drive_motor()
{

  motor1.drive(cur_motor_cmd1.motor_speed);
  motor2.drive(cur_motor_cmd2.motor_speed);

}

void turn_90right()
{
  bno.begin();
  delay(100);
  sensors_event_t event;
  bno.getEvent(&event);
//  float heading  = wrapdeg( (float)event.orientation.x);
  float heading  = (float)event.orientation.x;

//  Serial.println(heading);
  while (heading < 85)
  {
    bno.getEvent(&event);
    heading = wrapdeg( (float)event.orientation.x);
    turn_right();
    delay(100);
    stop_motors();  
    
  }
    
}

void turn_90left()
{
  bno.begin();
  delay(100);
  sensors_event_t event;
  bno.getEvent(&event);

  float heading  = wrapdeg( (float)event.orientation.x);
  
   while (heading > -85)
  {
    bno.getEvent(&event);
    heading = wrapdeg( (float)event.orientation.x);
    turn_left();
    delay(100);
    stop_motors();
    
  }
    
}

float wrapdeg(float val)
{
  val = (double)val;
  if (val > 180)
  {  val -= 360;
//    val = -val;
  }
  return (float)val;
}
void test()
{
  /* Get a new sensor event */
  bno.begin();
  sensors_event_t event;

  for (int i = 0; i <10000; i++)
  {
    bno.getEvent(&event);
  
//    Serial.print(F("Orientation: "));
//    Serial.print((float)event.orientation.x);
//    Serial.print(F(" "));
//    Serial.print((float)event.orientation.y);
//    Serial.print(F(" "));
//    Serial.print((float)event.orientation.z);
//    Serial.println(F(""));
//    delay(300);
  }
    
}
void imusetup()
{
  /* Get a new sensor event */
  bno.begin();
  sensors_event_t event;

  for (int i = 0; i <100; i++)
  {
    bno.getEvent(&event);
  
//    Serial.print(F("Orientation: "));
//    Serial.print((float)event.orientation.x);
//    Serial.print(F(" "));
//    Serial.print((float)event.orientation.y);
//    Serial.print(F(" "));
//    Serial.print((float)event.orientation.z);
//    Serial.println(F(""));
  }
    
}

void loop()
{

//   Tape Sensors:
   read_tape_sensors(); // loads tape sensor values to proximity V1 and proximity V2
   if (tape_sensors.left <= v_thresh && tape_sensors.right > v_thresh){
    turn_right();
    delay(100);
    stop_motors();
    
//    Serial.println("left");
   }
   if (tape_sensors.left > v_thresh && tape_sensors.right <= v_thresh){
    turn_left();
    delay(100);
    stop_motors();
//    Serial.println("right");
   }

  get_command_motor();
  drive_motor();
}
