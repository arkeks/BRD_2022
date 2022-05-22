// Authors 2022:   Guschin Gleb          nevidimka787
//                 Taralo Anton          Anton4822
//                 Dobrotin Arseny       arkeks
//                 Serebrennikov Andrey  seradya

#include <Servo.h>
#include <Wire.h>

// если digitalRead(TRACK_SENSOR) == LOW - это чёрный
#define LEFT 0
#define RIGHT 1

#define SPEED 40

#define SERIAL_SPEED 115200

enum SENSORS {
    LEFT_SENSOR     = 0,
    LEFT_C_SENSOR   = 1,
    RIGHT_C_SENSOR  = 2,
    RIGHT_SENSOR    = 3,
    
    SENSORS_COUNT
};

typedef enum {
  // МОТОРЫ
  RIGHT_MOTOR_GO = 8,
  RIGHT_MOTOR_BACK = 7,
  RIGHT_MOTOR_PWM = 6,
  
  LEFT_MOTOR_GO = 2,
  LEFT_MOTOR_BACK = 4,
  LEFT_MOTOR_PWM = 5,

  // СЕРВА
  SERVO = 3, // PWM
  
  // ДАТЧИКИ ЛИНИИ
    
  RIGHT_TRACK        = 18,
  RIGHT_CENTER_TRACK = 17,
  LEFT_CENTER_TRACK  = 15,
  LEFT_TRACK         = 16
} pin_t;

enum STATES {
  S_NO_CHANGE_S = 255,
  
  S_ON_LINE   = 0,
  S_ON_CROSS  = 1,

  S_ON_INV_LINE   = 2,
  S_ON_INV_CROSS  = 3,

  S_ON_START  = 4,
  S_ON_POINTS = 5,
  S_ON_FINISH = 6,

  STATES_COUNT
};  // byte

enum EVENTS {
/* on -- BLACK color detected */
  
  E_CRL = 0,  // close right and close left on
  E_CR  = 1,  // close right on
  E_CL  = 2,  // close left on
  E_RL  = 3,  // right and left on
  E_R   = 4,  // right on
  E_L   = 5,  // left on

  E_NO_R    = 6,  // left, close left and close right on
  E_NO_L    = 7,  // close left, close right and right on
  E_ALL     = 8,  // all sensors on
  E_NOTHING = 9,  // all sensors off

  EVENTS_COUNT
};  // byte

enum CROSS_ACTION {
  CROSS_FORWARD = 0,
  CROSS_LEFT    = 1,
  CROSS_BACKWARD = 2,
  CROSS_RIGHT   = 3,
    
  CROSS_TO_POINTS = 4,
  CROSS_TO_LINE   = 5,
    
  CROSSES_COUNT = 9
};

typedef byte (*action_t)(void);
typedef byte state_t;
typedef byte event_t;

namespace gv {  // global variables -----------------------------------------------------------------------------------------------------------------------------
  state_t state;
  event_t event;
  byte cross_count;

  bool sensors_data[SENSORS_COUNT] = {0, 0, 0 ,0};
  byte crosses_count;
    
  byte crosses_list[CROSSES_COUNT] = {CROSS_FORWARD, CROSS_TO_POINTS, CROSS_TO_LINE, CROSS_RIGHT, CROSS_FORWARD, CROSS_LEFT, CROSS_FORWARD, CROSS_LEFT, CROSS_RIGHT};
    
  byte last_active_center_sensor;
  byte cross_activate_sensor;
};

void run()
{

  //left motor go
  digitalWrite(LEFT_MOTOR_GO, HIGH);   //enable left motor go
  digitalWrite(LEFT_MOTOR_BACK, LOW);  //prohibit left motor back
  analogWrite(LEFT_MOTOR_PWM, SPEED);    

  //right motor go
  digitalWrite(RIGHT_MOTOR_GO, HIGH);  //enable right motor go
  digitalWrite(RIGHT_MOTOR_BACK, LOW); //prohibit right motor back
  analogWrite(RIGHT_MOTOR_PWM, SPEED);   

}


void brake()
{
  digitalWrite(LEFT_MOTOR_GO, LOW);
  digitalWrite(LEFT_MOTOR_BACK, LOW);
  digitalWrite(RIGHT_MOTOR_GO, LOW);
  digitalWrite(RIGHT_MOTOR_BACK, LOW);

}



void turn(byte side){   // 0 - left // 1 - right 
  if(side == RIGHT){ // -------------------------
    //left motor back
    digitalWrite(LEFT_MOTOR_GO, LOW);     //prohibit left motor go
    digitalWrite(LEFT_MOTOR_BACK, LOW);   //prohibit left motor back
    analogWrite(LEFT_MOTOR_PWM, 0);       

    digitalWrite(RIGHT_MOTOR_GO, HIGH);  //enable right motor go
    digitalWrite(RIGHT_MOTOR_BACK, LOW); //prohibit right motor back
    analogWrite(RIGHT_MOTOR_PWM, SPEED);   
  } else {
    //left motor go
    digitalWrite(LEFT_MOTOR_GO, HIGH);   //enable left motor go
    digitalWrite(LEFT_MOTOR_BACK, LOW);  //prohibit left motor back
    analogWrite(LEFT_MOTOR_PWM, SPEED);

    //right motor back 
    digitalWrite(RIGHT_MOTOR_GO, LOW);   //prohibit right motor go
    digitalWrite(RIGHT_MOTOR_BACK, LOW); //prohibit right motor back
    analogWrite(RIGHT_MOTOR_PWM, 0);
  }
}



void back()
{
  //left motor back
  digitalWrite(LEFT_MOTOR_GO, LOW);     //prohibit left motor go
  digitalWrite(LEFT_MOTOR_BACK, HIGH);  //enable left motor back
  analogWrite(LEFT_MOTOR_PWM, SPEED);

  //right motor back
  digitalWrite(RIGHT_MOTOR_GO, LOW);    //prohibit right motor go
  digitalWrite(RIGHT_MOTOR_BACK, HIGH); //enable right motor back
  analogWrite(RIGHT_MOTOR_PWM, SPEED);
}

void flash_led() {
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  delay(300);
  for(int i = 0; i < 3; i++) { 
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  delay(100);
  digitalWrite(11, LOW);
  }
  digitalWrite(9, HIGH);
  delay(100);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

void fan() {
  digitalWrite(A5, LOW);
}

event_t get_event(){

    gv::sensors_data[LEFT_SENSOR] = !digitalRead(LEFT_TRACK);
    gv::sensors_data[LEFT_C_SENSOR] = !digitalRead(LEFT_CENTER_TRACK);
    gv::sensors_data[RIGHT_C_SENSOR] = !digitalRead(RIGHT_CENTER_TRACK);
    gv::sensors_data[RIGHT_SENSOR] = !digitalRead(RIGHT_TRACK);
    
    if(gv::sensors_data[RIGHT_C_SENSOR] == 1 && gv::sensors_data[LEFT_C_SENSOR] == 1 && gv::sensors_data[LEFT_SENSOR] == 0 && gv::sensors_data[RIGHT_SENSOR] == 0){
        return E_CRL;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 1 && gv::sensors_data[LEFT_C_SENSOR] == 0 && gv::sensors_data[LEFT_SENSOR] == 0 ){
        gv::last_active_center_sensor = RIGHT_C_SENSOR;
        return E_CR;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 0 && gv::sensors_data[LEFT_C_SENSOR] == 1 && gv::sensors_data[RIGHT_SENSOR] == 0){
        gv::last_active_center_sensor = LEFT_C_SENSOR;
        return E_CL;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 0 && gv::sensors_data[LEFT_C_SENSOR] == 0 && gv::sensors_data[LEFT_SENSOR] == 1 && gv::sensors_data[RIGHT_SENSOR] == 1){
        return E_RL;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 0 && gv::sensors_data[LEFT_C_SENSOR] == 0 && gv::sensors_data[LEFT_SENSOR] == 0 && gv::sensors_data[RIGHT_SENSOR] == 1){
        return E_R;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 0 && gv::sensors_data[LEFT_C_SENSOR] == 0 && gv::sensors_data[LEFT_SENSOR] == 1 && gv::sensors_data[RIGHT_SENSOR] == 0){
        return E_L;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 1 && gv::sensors_data[LEFT_C_SENSOR] == 1 && gv::sensors_data[LEFT_SENSOR] == 1 && gv::sensors_data[RIGHT_SENSOR] == 0){
        return E_NO_R;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 1 && gv::sensors_data[LEFT_C_SENSOR] == 1 && gv::sensors_data[LEFT_SENSOR] == 0 && gv::sensors_data[RIGHT_SENSOR] == 1){
        return E_NO_L;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 1 && gv::sensors_data[LEFT_C_SENSOR] == 1 && gv::sensors_data[LEFT_SENSOR] == 1 && gv::sensors_data[RIGHT_SENSOR] == 1){
        return E_ALL;
    }

    if(gv::sensors_data[RIGHT_C_SENSOR] == 0 && gv::sensors_data[LEFT_C_SENSOR] == 0 && gv::sensors_data[LEFT_SENSOR] == 0 && gv::sensors_data[RIGHT_SENSOR] == 0){
        return E_NOTHING;
    }

}

namespace af {  // action functions
  state_t goFront();   // drive forward
  state_t goFrontS();
  state_t turnRight(); // drive right
  state_t turnLeft();  // drive left
  state_t detectInv();  // now the robot drive on invert line
  state_t detectNor();  // now the robot drive on normal line
  state_t scanCross();  // check cross
  state_t scanArea();   // later
  state_t scanBorder(); // later
  state_t stop();
  state_t onLine();
  void hardTurn(byte side);
};


action_t actions[STATES_COUNT][EVENTS_COUNT] {
                      /* E_CRL */       /* E_CR */      /* E_CL */      /* E_RL */      /* E_R */       /* E_L */       /* E_NO_R */      /* E_NO_L */      /* E_ALL*/      /* E_NOTHING */
/* S_ON_LINE */       { af::goFront,    af::turnRight,  af::turnLeft,   af::detectInv,  af::scanCross,  af::scanCross,  af::scanCross,    af::scanCross,    af::scanArea,   af::scanArea  },
/* S_ON_CROSS */      { af::goFront,    af::turnRight,  af::turnLeft,   af::detectInv,  af::scanCross,  af::scanCross,  af::scanCross,    af::scanCross,    af::scanCross,  af::scanCross },
/* S_ON_INV_LINE */   { af::detectNor,  af::turnLeft,   af::turnRight,  af::goFront,    af::scanCross,  af::scanCross,  af::scanCross,    af::scanCross,    af::scanCross,  af::scanCross },
/* S_ON_INV_CROSS */  { af::detectNor,  af::turnLeft,   af::turnRight,  af::goFront,    af::scanCross,  af::scanCross,  af::scanCross,    af::scanCross,    af::scanCross,  af::scanCross },
/* S_ON_START */      { af::onLine,     af::scanBorder, af::scanBorder, af::scanBorder, af::scanBorder, af::scanBorder, af::scanBorder,   af::scanBorder,   af::goFront,    af::goFront   },
/* S_ON_POINTS */     { af::goFrontS,   af::goFrontS,   af::goFrontS,   af::goFrontS,   af::goFrontS,   af::goFrontS,   af::goFrontS,     af::goFrontS,     af::scanCross,  af::goFrontS  },
/* S_ON_FINISH */     { af::stop,       af::stop,       af::stop,       af::stop,       af::stop,       af::stop,       af::stop,         af::stop,         af::stop,       af::stop      }
};



void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_SPEED);
  // ДАТЧИКИ ЛИНИИ
  pinMode(RIGHT_TRACK, INPUT);
  pinMode(RIGHT_CENTER_TRACK, INPUT);
  pinMode(LEFT_CENTER_TRACK, INPUT);
  pinMode(LEFT_TRACK, INPUT);

  // МОТОРЫ
  pinMode(LEFT_MOTOR_GO, OUTPUT);
  pinMode(LEFT_MOTOR_BACK, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  
  pinMode(RIGHT_MOTOR_GO, OUTPUT);
  pinMode(RIGHT_MOTOR_BACK, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // СЕРВА
  pinMode(SERVO, OUTPUT);
    
  //Светодиоды
  pinMode(9, OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(A5, OUTPUT);
    
  gv::crosses_count = 0;
  gv::state = S_ON_START;
  gv::event = get_event();
  gv::crosses_count = 0;
    
 flash_led(); //Мигание светодиодами
 fan();
}

void printState(byte state);
void printEvent(byte event);



state_t prev_st;
event_t prev_ev;

void loop() {  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  
  gv::event = get_event();
    
  if (gv::event != prev_ev) {
    Serial.print("EVENT: ");
    printEvent(gv::event);
  }
    
  prev_ev = gv::event;
    
    
  byte new_state = actions[gv::state][gv::event]();
  if (new_state != S_NO_CHANGE_S) {
    gv::state = new_state;
  }
  
  if (gv::state != prev_st) {
    Serial.print("STATE: ");
    printState(gv::state);
  }
    
  prev_st = gv::state;
    
  if (gv::state == S_ON_FINISH) {
      return;
  }
}

state_t af::goFront() {
    run();
    if (gv::state == S_ON_LINE || gv::state == S_ON_CROSS) {
    return S_ON_LINE;
    } else {
      return S_ON_INV_LINE;
    }
    return S_NO_CHANGE_S;
}

state_t af::goFrontS() {
    run();
    return S_NO_CHANGE_S;
}

state_t af::turnRight() {
  turn(RIGHT);
  return S_NO_CHANGE_S;
}

state_t af::turnLeft() {
  turn(LEFT);
  return S_NO_CHANGE_S;
}

state_t af::detectInv() {
    return S_ON_INV_LINE;
}

state_t af::detectNor() {
    return S_ON_LINE;
}

state_t af::scanCross() {
  if (gv::state != S_ON_CROSS && gv::state != S_ON_INV_CROSS) {
    ++(gv::crosses_count);
  }
    
  if (gv::crosses_count > CROSSES_COUNT) {
    delay(500);
    return S_ON_FINISH;
  }
    
  switch (gv::crosses_list[gv::crosses_count - 1]) {
    case CROSS_RIGHT:
        af::hardTurn(RIGHT);
      break;
    case CROSS_LEFT:
        af::hardTurn(LEFT);
      break;
    case CROSS_FORWARD:
        af::goFront();
      break;
    case CROSS_BACKWARD:
      af::turnLeft();
    case CROSS_TO_POINTS:
      af::goFront();
      delay(500);
      return S_ON_POINTS;
    case CROSS_TO_LINE:
      af::goFront();
      delay(500);
      return S_ON_LINE;
      break;
  }

  return (gv::state == S_ON_INV_LINE || gv::state == S_ON_INV_CROSS) ? S_ON_INV_CROSS : S_ON_CROSS;
}

state_t af::scanBorder() {
  if (gv::sensors_data[RIGHT_SENSOR] && !gv::sensors_data[LEFT_SENSOR]) {
    turnRight();
  }
  if (!gv::sensors_data[RIGHT_SENSOR] && gv::sensors_data[LEFT_SENSOR]) {
    turnLeft();
  }
    
  
    
  goFront();
  return S_NO_CHANGE_S;
}

state_t af::onLine() {
  return S_ON_LINE;
}

state_t af::stop() {
    brake();
    return S_ON_FINISH;
}

void printState(byte state)
{
  switch(state) {
    case S_NO_CHANGE_S: 
      Serial.println("S_NO_CHANGE_S");
      break;
    case  S_ON_LINE: 
      Serial.println("S_ON_LINE");
      break;
    case  S_ON_CROSS: 
      Serial.println("S_ON_CROSS");
      break;
    case  S_ON_INV_LINE: 
      Serial.println("S_ON_INV_LINE");
      break;
    case  S_ON_INV_CROSS: 
      Serial.println("S_ON_INV_CROSS");
      break;
    case  S_ON_START: 
      Serial.println("S_ON_START");
      break;
    case  S_ON_POINTS: 
      Serial.println("S_ON_POINTS");
      break;
    case  S_ON_FINISH: 
      Serial.println("S_ON_FINISH");
      break;
  }
}

void printEvent(byte event)
{
  switch(event) {
    case  E_CRL: Serial.println("E_CRL");         break;
    case  E_CR: Serial.println("E_CR");           break;
    case  E_CL: Serial.println("E_CL");           break;
    case  E_RL: Serial.println("E_RL");           break;
    case  E_R: Serial.println("E_R");             break;
    case  E_L: Serial.println(" E_L");            break;
    case  E_NO_R: Serial.println("E_NO_R");       break;
    case  E_NO_L: Serial.println("E_NO_L");       break;
    case  E_ALL: Serial.println("E_ALL");         break;
    case  E_NOTHING: Serial.println("E_NOTHING"); break;
  }    
}

state_t af::scanArea() {
    if (gv::last_active_center_sensor == LEFT_C_SENSOR) {
        af::turnLeft();
    } else if (gv::last_active_center_sensor == RIGHT_C_SENSOR) {
        af::turnRight();
    }
    
    return (gv::state == S_ON_INV_LINE || gv::state == S_ON_INV_CROSS) ? S_ON_INV_LINE : S_ON_LINE;
}


void af::hardTurn(byte side){   // 0 - left // 1 - right 
  if(side == RIGHT){ // -------------------------
    //left motor back
    digitalWrite(LEFT_MOTOR_GO, LOW);     //prohibit left motor go
    digitalWrite(LEFT_MOTOR_BACK, HIGH);   //prohibit left motor back
    analogWrite(LEFT_MOTOR_PWM, SPEED * 1.2);       

    digitalWrite(RIGHT_MOTOR_GO, HIGH);  //enable right motor go
    digitalWrite(RIGHT_MOTOR_BACK, LOW); //prohibit right motor back
    analogWrite(RIGHT_MOTOR_PWM, SPEED);   
  } else {
    //left motor go
    digitalWrite(LEFT_MOTOR_GO, HIGH);   //enable left motor go
    digitalWrite(LEFT_MOTOR_BACK, LOW);  //prohibit left motor back
    analogWrite(LEFT_MOTOR_PWM, SPEED);

    //right motor back 
    digitalWrite(RIGHT_MOTOR_GO, LOW);   //prohibit right motor go
    digitalWrite(RIGHT_MOTOR_BACK, HIGH); //prohibit right motor back
    analogWrite(RIGHT_MOTOR_PWM, SPEED * 1.2);
  }
}
