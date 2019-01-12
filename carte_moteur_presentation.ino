#include <AccelStepper.h>
#include <Wire.h>
#include <Servo.h>

#define RESET_RIGHT 9
#define SLEEP_RIGHT 10
#define STEP_RIGHT 11
#define DIR_RIGHT 12

#define RESET_LEFT 13
#define SLEEP_LEFT 15
#define STEP_LEFT 0
#define DIR_LEFT 1

#define PIN_SERVO_RIGHT 6
#define PIN_SERVO_LEFT 5

#define PIN_SERVO_SENSOR 4

#define IR_BALISE 3
#define IR_LEFT 21
#define IR_RIGHT 20

#define DIAM_ROUE 74
#define DIST_ROUE 230
#define RATIO 1/4

#define ACCELERATION 1400
#define SPEED 14000

//diametre = 74mm
//distance 1 tour : Pi * diametre = 232 mm
//1 tour = 200 step
//1 step = distance 1 tour / 200
//1 step * 5 (motoreduction) = 1.16 mm
//1 step = 0.23mm
// 1/2 step = 0.12mm
// 1/4 step = 0.06mm

//distance entre les deux roues = 230mm
//diametre tour complet = 230*Pi
// 1° = (230*pi)/180
// 1° = 4mm

const float distOneStep = (((PI*DIAM_ROUE)/200)/5)*RATIO;
const float distOneDegree = (PI*DIST_ROUE*2)/360;

int numTransmit = 0;

int side;

typedef struct {
  int left;
  int right;
}stepMotors;

AccelStepper motor_left(1, STEP_RIGHT, DIR_RIGHT);
AccelStepper motor_right(1, STEP_LEFT, DIR_LEFT);

int stepForRotation(int degre){
  return (int)((distOneDegree*degre)/distOneStep)*0.978;
}

int stepForDistance(int distance){ //distance en mm
  return (int)(distance/distOneStep)*0.97;
}

stepMotors strat[42] ;

stepMotors turn(int degree, int side){
  stepMotors result;
  result.left = -stepForRotation(degree)/2;
  result.right = -stepForRotation(degree)/2;
  return result;
}

stepMotors move(int distance){
  stepMotors result;
  result.left = -stepForDistance(distance);
  result.right = stepForDistance(distance);
  return result;
}

void initTabs(stepMotors strat[], int side){

  if(side == 1){
    strat[0] = move(500); 
  
    strat[1] = turn(360, side);
  
    strat[2] = move(-500); 
    
    strat[3] = turn(-360, side);
  }
  else{
    strat[0] = move(540); 
    
    strat[1] = turn(45, side);

    strat[2] = move(400);
  
    strat[3] = turn(-60, side);
  
    strat[4] = move(-620);
  
    strat[5] = turn(-30, side);
  
    strat[6] = turn(30, side);
  
    strat[7] = move(170);
  
    strat[8] = turn(60, side);
  
    strat[9] = move(350);
  
    strat[10] = turn(-90, side);
  
    strat[11] = move(-380);
  
    strat[12] = move(150);
  
    strat[13] = turn(-30, side);
  
    strat[14] = move(1400);
  
    strat[15] = turn(-15, side);
    
    strat[16] = move(300);
    
    strat[17] = turn(-10, side);
    
    strat[18] = move(25);
    
    strat[19] = turn(-10, side);
    
    strat[20] = move(25);
    
    strat[21] = turn(-10, side);
    
    strat[22] = move(25);
    
    strat[23] = turn(-10, side);
  
    strat[24] = move(150);

    strat[25] = move(-200);
    
    strat[26] = turn(-5, side);
  
    strat[27] = move(25);
  
    strat[28] = turn(-10, side);
    
    strat[29] = move(25);
    
    strat[30] = turn(-10, side);
    
    strat[31] = move(25);
    
    strat[32] = turn(-10, side);
    
    strat[33] = move(25);
    
    strat[34] = turn(-10, side);
    
    strat[35] = move(25);
  
    strat[36] = turn(-45, side);
  
    strat[37] = move(250);
    
    strat[38] = turn(-20, side);
  
    strat[39] = move(1500);
  }
  
}

int start = 0;
int count = 0;

Servo servo_right;
Servo servo_left;
Servo servo_sensor;
boolean stopped;
boolean check_obstacle;

void setup()
{      
    pinMode(IR_BALISE, INPUT);

    pinMode(RESET_RIGHT, OUTPUT); digitalWrite(RESET_RIGHT, HIGH);
    pinMode(SLEEP_RIGHT, OUTPUT); digitalWrite(SLEEP_RIGHT, HIGH);
    pinMode(RESET_LEFT, OUTPUT); digitalWrite(RESET_LEFT, HIGH);
    pinMode(SLEEP_LEFT, OUTPUT); digitalWrite(SLEEP_LEFT, HIGH);

    servo_right.attach(PIN_SERVO_RIGHT);
    servo_left.attach(PIN_SERVO_LEFT);

    servo_sensor.attach(PIN_SERVO_SENSOR);

    servo_right.write(0);
    servo_left.write(70);
    
    Wire.begin(1);
    Wire.onReceive(receiveEvent);
    
    Serial.begin(9600);
    
    motor_left.setMaxSpeed(SPEED);
    motor_left.setAcceleration(ACCELERATION);
    
    motor_right.setMaxSpeed(SPEED);
    motor_right.setAcceleration(ACCELERATION);
  
}

void receiveEvent(int Bytes){

  int data = Wire.read();
  int centaine = (int)(data/100);
  int dizaine = (int)((data%100)/10);
  int unite = (int)(data%10);
  if(data != 0){
    start = 1;
    if(centaine == 1){
      side = 1; //orange
    }
    else{
      side = -1; //vert
    }
    if(dizaine == 1){
      stopped = true;
    }
    else{
      stopped = false;
    }
    if(unite == 1){
      check_obstacle = false;
    }
    else{
      check_obstacle = true;
    }
    initTabs(strat, side);
  }else{
    start = 0;
  }
}

stepMotors stepBuffer;


unsigned long currentMillis;
unsigned long previousMillis;
int angle = 0;
int sens = 1;
unsigned long currentMillis2;
unsigned long previousMillis2;

boolean already_stopped = false;

int dep_actio = 0;

int  distance_left;
int  distance_right;

int capteur = 1;
boolean test;

unsigned long startTimer;
unsigned long currentTimer;
unsigned long startDelay;
unsigned long currentDelay;
boolean blind = false;

void loop()
{

  if(start == 1){

    if(capteur == 1){
      test = !(digitalRead(IR_BALISE) == LOW && motor_left.distanceToGo()<=0 && motor_right.distanceToGo()>=0);
    }
    else if(capteur == 2){
      distance_left = 13*pow(analogRead(IR_LEFT)*0.0048828125, 1);
      test =  !(motor_left.distanceToGo()>0 && motor_right.distanceToGo()<0 && distance_left >10);
    }
    else if(capteur == 3){
      distance_right = 13*pow(analogRead(IR_RIGHT)*0.0048828125, 1);
      test = !(motor_right.distanceToGo()>0 && motor_right.distanceToGo()<0 && distance_right >10);
    }
    
    if(blind || !check_obstacle || test) {

      if(blind){
        currentDelay = millis();
        if(currentDelay - startDelay >= 1000){
          blind = false;
        }
      }

      if( already_stopped ){
        motor_left.setAcceleration(ACCELERATION);
        motor_right.setAcceleration(ACCELERATION);
        motor_left.move(stepBuffer.left);
        motor_right.move(stepBuffer.right);
      }
      already_stopped = false;
      servo_sensor.write(angle);


      currentMillis = millis();
      if (currentMillis - previousMillis >= 10) {
        previousMillis = currentMillis;
        
        if(angle == 103)
          sens = -1;
          
        if(angle == 3)
          sens = 1;
          
        angle+=sens;
      }

      if(side==1){
        currentMillis2 = millis();     
        if (currentMillis2 - previousMillis2 >= 500) {
          if(servo_right.read() == 0){
            servo_right.write(45);  //60 
            servo_left.write(70);
            
          }
          else{
            servo_right.write(0);
            servo_left.write(20); // 
          }
          previousMillis2 = millis();
        }
      }
        
      if(motor_right.distanceToGo() == 0 && motor_left.distanceToGo() == 0 && count < (int)(sizeof(strat)/sizeof(strat[0])))
      {
        if(side==1){
          motor_left.move(strat[count].left);
          motor_right.move(strat[count].right);
          count++;
          if(count == 4)
            count = 0;

        }else{
          if(count == 25){
            servo_right.write(45);
    
            currentMillis2 = millis();
            if(dep_actio == 0){
              dep_actio = 1;
              previousMillis2 = currentMillis2;
            }
            if (currentMillis2 - previousMillis2 >= 1000) {
              motor_left.move(strat[count].left);
              motor_right.move(strat[count].right);
              count++;
            }
          }
          else{
            motor_left.move(strat[count].left);
            motor_right.move(strat[count].right);
            count++;
          }
          if(count == 27){
            if(side == -1)
              servo_right.write(0);
          }
        }
      }
    }
    else{
      if(!already_stopped){
        startTimer = millis();
        stepBuffer.left = motor_left.distanceToGo();
        stepBuffer.right = motor_right.distanceToGo();
        motor_left.setAcceleration(SPEED);
        motor_right.setAcceleration(SPEED);
        motor_right.stop();
        motor_left.stop();
        already_stopped = true;
      }
      currentTimer = millis();
      if(currentTimer - startTimer >= 3000){
        already_stopped = false;
        motor_left.move(stepBuffer.left);
        motor_right.move(stepBuffer.right);
        blind = true;
        startDelay = millis();
      }
    }
    
    if(!stopped){
      motor_left.run();
      motor_right.run();
    }
    if(!already_stopped){
      capteur++;
      if(capteur>3){
        capteur = 1;
      }
    }
  }
}
