// Authors 2022:   Guschin Gleb          nevidimka787
//                 Taralo Anton          Anton4822
//                 Dobrotin Arseny       arkeks
//                 Serebrennikov Andrey  seradya

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <PCF8574.h>

typedef enum {
  FIRST_POINT   = 0,
  POINTS_COUNT  = 11,
  CENTER_POINT  = POINTS_COUNT / 2
}

#define LIDAR_AMPLITUDE       180.0
#define ANGLE_BETWEEN_POINTS  LIDAR_AMPLITUDE / (POINTS_COUNT - 1)

#define ROAD_HIGH                           300.0               // mm
#define ROBOT_LENGTH                        250.0               // mm
#define ROBOT_DISTANCE_FROM_LIDAR_TO_CENTER ROBOT_LENGTH / 2.0  // mm
#define LIDAR_DETECTION_AREA_RADIUS         130.0               // mm

typedef enum {
  // пины для серво-приводов
  LIDAR_SERV  = 9,
  FRONT_SERV  = 10,
  BACK_SERV   = 11,

  //пины для моторов редукторов
  M_FRONT_FORWARD  = 1,  // AIN2
  M_FRONT_BACKWARD = 2,  // AIN1
  FRONT_PWM   = 3,       // PWMA

  M_BACK_FORWARD   = 7,  // BIN2
  M_BACK_BACKWARD  = 8,  // BIN1
  BACK_PWM    = 6        // PWMB
} pin_id_t;

// скорости
//#define DEFAULT_SPEED ?
//#define BACK_SPEED ?
//#define ROTATE_SPEED ?

PCF8574 PCF_1(0x20);  // создаём объект класса PCF8574 для использования расширителя

typedef enum {
  SEN_ON_ROOF_ID  = 0,
  SEN_FL_ID       = 1,  // front left
  SEN_FR_ID       = 2,  // front right
  SEN_L_ID        = 3,  // left
  SEN_R_ID        = 4,  // front right
  SEN_BL_ID       = 5,  // back left
  SEN_BR_ID       = 6,  // back right

  SENS_COUNT,

  FIRST_SEN       = SEN_FL_ID,
  LAST_SEN        = SEN_BR_ID,
  LOW_SENS_COUNT  = LAST_SEN - FIRST_SEN + 1
} sensor_id_t;

VL53L0X sensors[SENS_COUNT];  // датчик на крыше: 0; датчики на днище: 1-6;

Servo front_serv;
Servo back_serv;
Servo lidar_serv;

typedef enum {  // состояния
  S_NOT_CHANGE_S    = -2,
  S_SOLUTION_ERROR  = -1,

  S_BEGIN   = 0,  // default state after start of the program
  S_DRIVE_F = 1,  // robot drives forward
  S_DRIVE_B = 2,  // robot drives backward
  S_TURN_L  = 3,  // robot rurns left
  S_TURN_R  = 4,  // robot turns right
  S_STAY    = 5,  // robot stays
  S_BARIER  = 6,  // robot avoid a barier
  
  STATES_COUNT    // alwayse on the last position
} state_t;

typedef enum {  // события
  E_NO_CLIF_F = 0,    // front                  clif detected everywhere without front
  E_CLIF_E    = 1,    // everywhere             clif detected everywhere
  E_CLIF_F    = 1,    // front                  lidar detected a clif front
  E_CLIF_BC   = 2,    // back  close            back sensors detected a clif
  E_CLIF_FC   = 3,    // front close            front sensors detected a clif
  E_CLIF_RC   = 4,    // right close            right sensor detected a clif
  E_CLIF_LC   = 5,    // left  close            left sensor detected a clif
  E_WALL_R    = 6,    // right                  lidar detected a barier
  E_WALL_L    = 7,    // left                   lidar detected a barier
  E_TURN_L    = 8,    // to left                lidar detected no clif
  E_TURN_R    = 9,    // to right               lidar detected no clif
  E_BARIER_C  = 10,   // all directions close   sensor detected a low barier
  E_BARIER_CO = 11,   // complete avoid         sensor didn't detect a low barier
  
  EVENTS_COUNT  // alwayse on the last position
} event_t;

typedef enum {
  RIGHT   = 0,
  LEFT    = 1,
  
  SIDES_COUNT
} side_t;

typedef enum {
  CLIF    = 0,
  WALL    = 1,

  EDGES_COUNT
} edge_t;

namespace pl {  // parameters list
  event_t event;
  state_t state;
  
  double distances[POINTS_COUNT];
  
  size_t edges_positions[EDGES_COUNT][SIDES_COUNT];
  
  double turn_radius;
  
  double distance_from_lower[LOW_SENS_COUNT];
} // namespace pl

namespace fl {  // functions list
state_t missionStart();
state_t avoidBarier();      // Push barier between wheels.
state_t badSituation();     // I don't know (perhaps) how to solve the situation.
state_t turnLeft();         // Change the drive direction and drive forward.
state_t turnLeftHard();     // Change the drive direction only.
state_t turnRight();        // Change the drive direction and drive forward.
state_t turnRightHard();    // Change the drive direction only.
state_t driveFront();       // Don't change the drive direction and drive forward.
state_t driveFrontSlow();   // Don't change the drive direction and drive forward slow.
state_t driveBackward();    // Don't change the drive direction and drive backward.
state_t avoidToLeft();      // Don't change the drive direction and drive leftward.
state_t avoidToRight();     // Don't change the drive direction and drive rightward.
state_t missionComplete();

}  // namespace fl

typedef state_t (*action_t)(void /*use only global variables*/);

action_t* actions[STATES_COUNT][EVENTS_COUNT] {
                      /* S_BEGIN */       /* S_DRIVE_F */       /* S_DRIVE_B */     /* S_TURN_L */      /* S_TURN_R */      /* S_STAY */          /* S_BARIER */
/* E_NO_CLIF_F  */  { fl::missionStart,   fl::driveFront,       fl::driveBackward,  fl::driveFront,     fl::driveFront,     fl::missionComplete,  fl::avoidBarier    },
/* E_CLIF_E     */  { fl::badSituation,   fl::badSituation,     fl::badSituation,   fl::badSituation,   fl::badSituation,   fl::missionComplete,  fl::badSituation   },
/* E_CLIF_F     */  { fl::badSituation,   fl::driveFrontSlow,   fl::driveBackward,  fl::turnLeftHard,   fl::turnRightHard,  fl::missionComplete,  fl::driveBackward  },
/* E_CLIF_BC    */  { fl::missionStart,   fl::driveFront,       fl::badSituation,   fl::driveFrontSlow, fl::driveFrontSlow, fl::missionComplete,  fl::avoidBarier    },
/* E_CLIF_FC    */  { fl::badSituation,   fl::missionComplete,  fl::driveBackward,  fl::turnLeftHard,   fl::turnRightHard,  fl::missionComplete,  fl::driveBackward  },
/* E_CLIF_R     */  { fl::badSituation,   fl::avoidToLeft,      fl::avoidToLeft,    fl::avoidToLeft,    fl::avoidToLeft,    fl::missionComplete,  fl::avoidToLeft    },
/* E_CLIF_L     */  { fl::badSituation,   fl::avoidToRight,     fl::avoidToRight,   fl::avoidToRight,   fl::avoidToRight,   fl::missionComplete,  fl::avoidToRight   },
/* E_WALL_R     */  { fl::missionStart,   fl::turnLeft,         fl::turnLeft,       fl::turnLeftHard,   fl::turnLeft,       fl::missionComplete,  fl::avoidBarier    },
/* E_WALL_L     */  { fl::missionStart,   fl::turnRight,        fl::turnRight,      fl::turnRightHard,  fl::turnRight,      fl::missionComplete,  fl::avoidBarier    },
/* E_TURN_L     */  { fl::missionStart,   fl::turnLeft,         fl::turnLeft,       fl::turnLeft,       fl::turnLeft,       fl::missionComplete,  fl::avoidBarier    },
/* E_TURN_R     */  { fl::missionStart,   fl::turnRight,        fl::turnRight,      fl::turnRight,      fl::turnRight,      fl::missionComplete,  fl::avoidBarier    },
/* E_BARIER_C   */  { fl::avoidBarier,    fl::avoidBarier,      fl::avoidBarier,    fl::avoidBarier,    fl::avoidBarier,    fl::missionComplete,  fl::avoidBarier    },
/* E_BARIER_CO  */  { fl::badSituation,   fl::missionComplete,  fl::driveBackward,  fl::turnLeftHard,   fl::turnRightHard,  fl::missionComplete,  fl::driveFrontSlow }
};

void getEdges(size_t point);

event_t getEvent();

void getEdges();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // подключаем расширитель PCF8574
  if (!PCF_1.begin()) {
    Serial.println("could not initialize...");
  }
  
  if (!PCF_1.isConnected()) {
    Serial.println("=> not connected");
  } else {
    Serial.println("=> connected!");
  }

  // инициализируем датчики
  set_sens_addr();

  for(size_t i = 0; i < 7; ++i) {
    sensors[i].startContinuous();
  }

  // подключаем серво-приводы к пинам
  front_serv.attach(FRONT_SERV);
  back_serv.attach(BACK_SERV);
  lidar_serv.attach(LIDAR_SERV);
    
  pl::event = getEvent();
  pl::state = S_BEGIN;
}

void loop() {
  pl::state = actions[pl::state][pl::event]();
  if (pl::state < S_BEGIN || pl::state == S_STAY) {
    exit();
  }
}


// назначает разные i2c адреса датчикам
// и инициализирует их
void set_sens_addr() {
  // каждую итерацию оставляем включенным только один датчик
  // и присваеваем ему адрес по общей шине i2c
  for(size_t i = 0; i < 7; ++i) {
    PCF_1.write(i, LOW);  // включенный датчик (подаём LOW на XSHUT)
    
      for (size_t j = 0; j < 7; ++j) {
        if (j != i) {
          PCF_1.write(j, HIGH);  // остальные выключенные датчики
        }
      }
    
    sensors[i].setAddress(0xC1 + i);
    
    if(!sensors[i].init()) {
      Serial.println("Failed to detect and initialize sensor!");
      while (1) {}
    }
  }
  
  for(size_t i = 0; i < 7; ++i) {
    PCF_1.write(i, LOW);  // включаем все датчики
  }
}

namespace af { // aditional functions
// возвращает дистанцию в мм
double Distance(sensor_id_t sensor_num) {   
    return sensors[sensor_num].readRangeContinuousMillimeters();
}

void ScanLower(){
  for(bite i = FIRST_SEN; i <= LAST_SEN; ++i){
    distance_from_lower[i - FIRST_SEN] = sensors[i].readRangeContinuousMillimeters();
  }
}

double getServoAngle(size_t point_id) {
  return point_id * ANGLE_BETWEEN_POINTS;
}

double getInTurnRadius(double angle) {
  return (ROBOT_DISTANCE_FROM_LIDAR_TO_CENTER + LIDAR_DETECTION_AREA_RADIUS * (sin(angle) + abs(cos(angle))) + ROAD_HIGH / 2.0) / 2.0;
}

double getOutTurnRadius(double angle) {
  double cyrcle_delta_y = ROBOT_WIDTH / 2.0 - LIDAR_DETECTION_AREA_RADIUS * sin(angle);
  double cyrcle_delta_x = ROBOL_LENGTH / 2.0 + LIDAR_DETECTION_AREA_RADIUS * cos(angle);
  return (cyrcle_delta_y * cyrcle_delta_y + cyrcle_delta_x * cyrcle_delta_x) / (2.0 * cyrcle_delta_y);
}

void Go(double front_degree, double back_degree, uint_8t speed) {

  front_serv.write(front_degree);
  back_serv.write(back_degree); 

  digitalWrite(FRONT_F, HIGH); // front 
  digitalWrite(FRONT_B, LOW);  // @arkeks: почему такие пины?

  digitalWrite(BACK_F, HIGH); // back
  digitalWrite(BACK_B, LOW);  // F
  analogWrite(FRONT_PWM, speed); //front
  analogWrite(BACK_PWM, speed); //back
}

// задаём углы поворота колёс и едем назад с низкой скоростью
void GoBackward(double front_degree, double back_degree) {

    front_serv.write(front_degree);
    back_serv.write(back_degree); 

    digitalWrite(FRONT_F, LOW); // front 
    digitalWrite(FRONT_B, HIGH);  //

    digitalWrite(BACK_F, LOW); // back
    digitalWrite(BACK_B, HIGH);  // 

    analogWrite(FRONT_PWM, SPEED_LOW); //front
    analogWrite(BACK_PWM, SPEED_LOW); //back

}

void Turn(double rad, side_t side) {
  double degree = 90.0 - atan(rad / (LENGHT_OF_ROBOT / 2)); //надо задефайнить /* TODO (Nevidimka): не надо) */

  if(side == RIGHT) {
    Go(degree, 180.0 - degree, SPEED_LOW);
  }
  if(side == LEFT) {
    Go(180.0 - degree, degree, SPEED_LOW);
  }
}
} // namespace af

namespace fl {
state_t missionStart() {
  af::Go(90.0, 90.0, SPEED_LOW);
  return S_DRIVE_F;
}

void avoidBarier() {
  for(bite i = FIRST_SEN; i <= LAST_SEN; ++i) {
    if(i < MIN_LOWER_DISTANCE) {
      if(i == 0) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(150, 90, SPEED_LOW);
        }
      }
      if(i == 1) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(30, 90, SPEED_LOW);
        }
      }
      if(i == 2) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(150, 150, SPEED_LOW);
        }
      }
      if(i == 3) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(30, 30, SPEED_LOW);
        }
      }
      if(i == 4) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(30, 90, SPEED_LOW);
        }
      }
      if(i == 5) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(90, 150, SPEED_LOW);
        }
      }
      if(i == 6) {
        while(sensors[i].readRangeContinuousMillimeters() < MIN_LOWER_DISTANCE) {
          Go(90, 30, SPEED_LOW);
        }
      }
    }
  }
}

state_t badSituation() {
  return S_SOLUTION_ERROR;
}

state_t turnLeft() {
  af::Turn(turn_radius, LEFT);
  return S_TURN_L;
}

state_t turnLeftHard() {
  af::Go(180, 0, SPEED_LOW);
  return S_TURN_L;
}

state_t turnRight() {
  af::Turn(turn_radius, RIGHT);
  return S_TURN_R;
}

state_t turnRightHard() {
  af::Go(0, 180, SPEED_LOW);
  return S_TURN_R;
}

state_t driveFront() {
  af::Go(90, 90, SPEED_MAX);
  return S_DRIVE_F;
}

state_t driveFrontSlow() {
  af::Go(90, 90, SPEED_LOW);
  return S_DRIVE_F;
}

state_t driveBackward() {
  af::GoBackward(90.0, 90.0);
  return S_DRIVE_B;
}

state_t avoidToLeft() {
  while (Distance() > MAX_DISTANCE) {
      af::Go(front_serv, back_serv, 180, 180, SPEED_LOW);
  }
  return S_NOT_CHANGE_S;
}

state_t avoidToRight() {
  while (Distance() > MAX_DISTANCE) {
      af::Go(front_serv, back_serv, 0, 0, SPEED_LOW);
  }
  return S_NOT_CHANGE_S;
}

state_t missionComplete() {
  return S_STAY;
}
}  // namespace fl

void getEdges() {
  barier_detected = false;
  for (side_t side = 0; side < SIDES_COUNT; ++side) {
    for (edge_t edge = 0; edge < EDGES_COUNT; ++edge) {
      pl::edges_positions[edge][side] = POINTS_COUNT;
    }
  }
  
  if (pl::distances[CENTER_POINT] > MIN_DISTANCE && pl::distances[CENTER_POINT] < MAX_DISTANCE) {
    getEdges(CENTER_POINT);
    return;
  }

  for (size_t dp = 1; dp < CENTER_POINT; ++dp) {
    if (pl::distances[CENTER_POINT] > MIN_DISTANCE && pl::distances[CENTER_POINT] < MAX_DISTANCE) {
      getEdges(CENTER_POINT - dp);
      return;
    }
    
    if (pl::distances[CENTER_POINT] > MIN_DISTANCE && pl::distances[CENTER_POINT] < MAX_DISTANCE) {
      getEdges(CENTER_POINT + dp);
      return;
    }
  }
}

void getEdges(size_t point) {
  for (size_t p = point + 1; p < POINTS_COUNT; ++p) {
    if (distances[p] > MAX_DISTANCE && pl::edges_positions[CLIF][LEFT] == POINTS_COUNT) {
      pl::edges_positions[CLIF][LEFT] = p;
    }
    
    if (pl::distances[p] < MID_DISTANCE) {
      if (pl::distances[p] < MIN_DISTANCE && pl::edges_positions[WALL][LEFT] == POINTS_COUNT) {
        edges_positions[WALL][LEFT] = p;
      }
    }
  }
  
  for (size_t p = point; p > 0; --p) {
    if (pl::distances[p - 1] > MAX_DISTANCE && pl::edges_positions[CLIF][RIGHT] == POINTS_COUNT) {
      edges_positions[CLIF][RIGHT] = p;
    }
    if (pl::distances[p] < MID_DISTANCE) {
      if (pl::distances[p] < MIN_DISTANCE && pl::edges_positions[WALL][RIGHT] == POINTS_COUNT) {
        edges_positions[WALL][RIGHT] = p;
      }
    }
  }
}

event_t getEvent() {
  for (edge_t edge = 0; edge < EDGES_COUNT; ++edge) {
    for (side_t side = 0; side < SIDES_COUNT; ++side) {
      if (edges_positions[edge][side] != POINTS_COUNT) {
        goto SOMETHING_DETECTED
      }
    }
  }
  return E_CLIF_E;
  
  SOMETHING_DETECTED:
  
  if (pl::edges_positions[CLIF][RIGHT] != POINTS_COUNT && pl::edges_positions[CLIF][RIGHT] > CENTER_POINT) {
    double angle = 
      (pl::edges_positions[CLIF][LEFT] != POINTS_COUNT) ?
      getServoAngle(edges_positions[CLIF][LEFT]) :
      getServoAngle(edges_positions[CLIF][RIGHT]);
    
    turn_radius = getInTurnRadius(angle);
    return E_TURN_R;
  }
  if (pl::edges_positions[CLIF][LEFT] != POINTS_COUNT && pl::edges_positions[CLIF][LEFT] < CENTER_POINT) {
    double angle = 
      (pl::edges_positions[CLIF][RIGHT] != POINTS_COUNT) ?
      getServoAngle(edges_positions[CLIF][RIGHT]) :
      getServoAngle(edges_positions[CLIF][LEFT]);
    
    turn_radius = getInTurnRadius(angle);
    return E_TURN_L;
  }
  
  if (pl::edges_positions[WALL][RIGHT] != POINTS_COUNT && pl::edges_positions[WALL][RIGHT] > CENTER_POINT) {
    double angle = 
      (pl::edges_positions[WALL][LEFT] != POINTS_COUNT) ?
      getServoAngle(edges_positions[WALL][LEFT]) :
      getServoAngle(edges_positions[WALL][RIGHT]);
    
    turn_radius = getOutTurnRadius(angle);
    return E_WALL_R;
  }
  if (pl::edges_positions[WALL][LEFT] != POINTS_COUNT && pl::edges_positions[WALL][LEFT] > CENTER_POINT) {
    double angle = 
      (pl::edges_positions[WALL][RIGHT] != POINTS_COUNT) ?
      getServoAngle(edges_positions[WALL][RIGHT]) :
      getServoAngle(edges_positions[WALL][LEFT]);
    
    turn_radius = getOutTurnRadius(angle);
    return E_WALL_L;
  }
  
  if (distance_from_lower[SEN_FL_ID] > NORMAL_DISTANCE || distance_from_lower[SEN_FR_ID] > NORMAL_DISTANCE) {
    return E_CLIF_FC;
  }
  if (distance_from_lower[SEN_BL_ID] > NORMAL_DISTANCE || distance_from_lower[SEN_BR_ID] > NORMAL_DISTANCE) {
    return E_CLIF_BC;
  }
  if (distance_from_lower[SEN_L_ID] > NORMAL_DISTANCE) {
    return E_CLIF_L;
  }
  if (distance_from_lower[SEN_R_ID] > NORMAL_DISTANCE) {
    return E_CLIF_R;
  }
  
  for (size_t sensor = 0; sensor < LOW_SENS_COUNT; ++sensor) {
    if (distance_from_lower[sensor] < MIN_LOWER_DISTANCE) {
      return E_BARIER_C;
    }
  }
  
  return E_BARIER_CO;
}
