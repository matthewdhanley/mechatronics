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

// The absolute maximum this value can be is 250. 100 is a speed it won't
// get away from us.
const int max_speed = -100.0;

// Variable Initialization -- Line Follower
const int offsetA    = 1;
const int offsetB    = 1;
const int TAPE1_PIN  = A1; // Sensor output voltage
const int TAPE2_PIN  = A0; // Sensor output voltage
int proximityADC1    = 0;
int proximityADC2    = 0;
float v_thresh       = 3.0;  // threshold for seeing tape


struct tape_proximity{
  int left = 0;
  int right = 0;
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
//  Serial.println("Starting...")/;
  cur_motor_cmd1.motor_speed = 0;
  cur_motor_cmd2.motor_speed = 0;
}

void go_straight(){
  motor1.drive(max_speed);
  motor2.drive(max_speed);
}

void turn_left(){
  motor1.drive(-max_speed);
  motor2.drive(max_speed);
}

void turn_right(){
  motor1.drive(max_speed);
  motor2.drive(-max_speed);
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

  tape_sensors_prev.left = tape_sensors.left;
  tape_sensors_prev.right = tape_sensors.right; // Twisty Wire
}



void get_command_motor()
{  
  if (Serial.available() > 0) {
    int id = Serial.parseInt();
    // check to make sure its id not a speed:
    while(id > 4 || id <=0)
    {
      id = Serial.parseInt();
      Serial.println(id);    
    }
    Serial.println(id);
//
//    while (Serial.available() == 0){
//        delay(1);
//    }
  
  if (Serial.available() > 0){
      int motor_speed = Serial.parseInt();
        switch (id)
        {
          case 1:
            cur_motor_cmd1.motor_speed = motor_speed;
            break;
          case 2:  
            cur_motor_cmd2.motor_speed = motor_speed;
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
void loop()
{
//   read_tape_sensors(); // loads tape sensor values to proximity V1 and proximity V2
//   if (tape_sensors.left > v_thresh && tape_sensors.right > v_thresh){
//    go_straight();
//    Serial.println("straight");
//   }
//   else if (tape_sensors.left <= v_thresh && tape_sensors.right > v_thresh){
//    turn_left();
//    Serial.println("left");
//   }
//   else if (tape_sensors.left > v_thresh && tape_sensors.right <= v_thresh){
//    turn_right();
//    Serial.println("right");
//   }
//   else{
//    stop_motors();
//   }

  get_command_motor();
  drive_motor();
//  motor1.drive(max/_speed);
//  motor2.drive(max_speed);
      
  delay(100);
   
}
