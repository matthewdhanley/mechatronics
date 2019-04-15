/******************************************************************************
******************************************************************************/

// Pins for all input
int DIR2   = 2; //define Direction pin
int PUL2   = 3; //define Pulse pin

int DIR  = 4; //define Direction pin
int PUL  = 5; //define Pulse pin
int ENA   = 7; //define Enable Pin

//Motor Command from raspberry pi:
struct motor_command {
   int id;
   int motor_speed; // Steps
};

struct motor_command cur_motor_cmd3; // Horizontal
struct motor_command cur_motor_cmd4; // Vertical

void setup()
{
  Serial.begin(9600);

  pinMode (ENA, OUTPUT);
  
  // Horizontal Motor
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);

  // Vertical Motor
  pinMode (PUL2, OUTPUT);
  pinMode (DIR2, OUTPUT);

  // Set initial speed to 0
  cur_motor_cmd3.motor_speed = 0;
  cur_motor_cmd4.motor_speed = 0;
}

void horizontal_forward(int forward_maxstep){
  int step_size = 700;
  for (int i=0; i<forward_maxstep; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR,LOW);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(step_size);
    digitalWrite(PUL,LOW);
    delayMicroseconds(step_size);
  }
}

void horizontal_backward(int backwards_maxstep){
  int step_size = 700;
  for (int i=0; i<backwards_maxstep; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR,HIGH);
    digitalWrite(PUL,HIGH);
    delayMicroseconds(step_size);
    digitalWrite(PUL,LOW);
    delayMicroseconds(step_size);
  }
}

void vertical_up(int up_maxstep){
  int step_size = 500;  
  for (int i=0; i<up_maxstep; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR2,LOW);
    digitalWrite(PUL2,HIGH);
    delayMicroseconds(step_size);
    digitalWrite(PUL2,LOW);
    delayMicroseconds(step_size);
  }
}

void vertical_down(int up_maxstep){
  int step_size = 500;    
  for (int i=0; i<up_maxstep; i++)    //Forward 5000 steps
  {
    digitalWrite(DIR2,HIGH);
    digitalWrite(PUL2,HIGH);
    delayMicroseconds(step_size);
    digitalWrite(PUL2,LOW);
    delayMicroseconds(step_size);    
  }
}


void get_command_motor()
{

  // Get motor navigation command from raspberry pi:
  if (Serial.available() > 0) {
    
    int id = Serial.parseInt();
    // check to make sure its id not a speed:
    while(id > 5 || id <-4)
    {
      id = Serial.parseInt();
      Serial.println(id);    
    }
    Serial.println(id);
    
    if (Serial.available() > 0) {
      int motor_speed = Serial.parseInt();
        switch (id)
        {
          // Horizontal Actuator
          case 3:
            cur_motor_cmd3.motor_speed = motor_speed;
            if (cur_motor_cmd3.motor_speed > 10)
            {
              horizontal_forward(400);
              vertical_up(600);
              horizontal_backward(400);
              vertical_down(6000);
              horizontal_forward(400);
              vertical_up(6000);
              horizontal_backward(400);
              vertical_down(6000);
              vertical_up(100);
            }
            else if (cur_motor_cmd3.motor_speed < -10)
            {
              horizontal_backward(cur_motor_cmd3.motor_speed);  
            }
            else
            {
              //do nothing
            }
            break;
          // Vertical Actuator
          case 4:  
            cur_motor_cmd4.motor_speed = motor_speed;
            if (cur_motor_cmd4.motor_speed > 10)
            {
              vertical_up(cur_motor_cmd4.motor_speed);
            }
            else if (cur_motor_cmd4.motor_speed < -10)
            {
              vertical_down(cur_motor_cmd4.motor_speed);  
            }
            else
            {
              //do nothing
            }
            break;
          default:
            break;
        }
        Serial.println(motor_speed);        
      
    }
  
  }
  
}

void loop()
{
  delay(5000);
//  horizontal_forward(550);
//  vertical_up(5250);
//  horizontal_backward(550);
//  vertical_down(5250);
//  vertical_up(5000);
//  vertical_down(5000);
//  horizontal_forward(400);
//  vertical_up(6000);
//  horizontal_backward(400);
//  vertical_down(6000);
//  vertical_up(100);  

//  get_command_motor();
//  delay(5000);
//  horizontal_forward(450);
//  vertical_up(8000);
//  horizontal_backward(450);
//  vertical_down(7000);
  delay(5000);
   
}
