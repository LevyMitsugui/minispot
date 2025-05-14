/* 
* -----------------------------------------------------------------------------
* Example: Two way communication between ESP32 and Python using WIFI
* -----------------------------------------------------------------------------
* Author: Radhi SGHAIER: https://github.com/Rad-hi
* -----------------------------------------------------------------------------
* Date: 07-05-2023 (7th of May, 2023)
* -----------------------------------------------------------------------------
* License: Do whatever you want with the code ...
*          If this was ever useful to you, and we happened to meet on 
*          the street, I'll appreciate a cup of dark coffee, no sugar please.
* -----------------------------------------------------------------------------
*/


#include "config.h"
#include <Kinematics.hpp>
#include <MyButton.h>
#include "SpotServo.hpp"
#include <Utilities.hpp>
#include "Spot.hpp"
#include <cstring>

// #define CONFIG_FREERTOS_USE_TRACE_FACILITY              1
// #define CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS  1

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


// ----------------- ADAPT VARIABLES ------------------

#define I_TO_A(i) (i+48)

#include <ArduinoEigen.h>
#include "Kinematics.hpp"
#include "SpotModel.hpp"
#include "LieAlgebra.hpp"


#define PRINT false
#define DEBUG_set_stance_wspeed false
#define COMMAND_BUFFER_SIZE 256

bool SERIAL_FORWARDING = false;
bool pos_feedback_toggle = false;
bool log_toggle = false;

int cycle = 0;
float speedShoulder,speedElbow,speedWrist,speedcalc,positionShoulder,positionElbow,positionWrist,loadShoulder,loadElbow,loadWrist = 0;

char inputBuffer[COMMAND_BUFFER_SIZE] = {0};
int bufferPos= 0;

void processPiCommand(const char* cmd);
void processTerminalCommand(const char* cmd);
void processCompoundCommand(const char* cmd);
void processPiCommandServoFrame(const char* cmd);
void processMotionCommand(const char* cmd);

enum CONTROL_STATES{
  SET_SERVO_FRAME,      //0
  SET_COMPOUND_CONTROL, //1
  SET_MOTION_COMMAND,   //2
  SET_STANCE_STRAIGHT,  //3
  SET_STANCE_PRONE,     //4
  TOGGLE_LOG,           //5
  SET_FEEDBACK          //6
};
enum SERVOS_CONTROL_IDX{// Follows indexes from the declaration "SpotServo * Servos[12]" below
  FL_SHOULDER, FL_ELBOW, FL_WRIST,
  FR_SHOULDER, FR_ELBOW, FR_WRIST,
  RL_SHOULDER, RL_ELBOW, RL_WRIST,
  RR_SHOULDER, RR_ELBOW, RR_WRIST
};

typedef struct PI_COMPOUND_FRAME{
  bool new_command;
  int command;
  float rpy[3];       //Roll, Pitch, Yaw
  float speed_rpy[3]; //Roll, Pitch, Yaw
  float xyz[3];       //X, Y, Z
  float speed_xyz[3]; //X, Y, Z

  float feet[4][3];   //X, Y, Z
  float speed_feet[4][3]; //X, Y, Z
} PI_COMPOUND_COMMAND;

typedef struct PI_SERVO_FRAME{
  bool new_command;
  int command;
  float pos[12];
  float speed[12];
} PI_FAST_COMMAND;

typedef struct MOTION_COMMAND{
  float rpy[3];       //Roll, Pitch, Yaw
  float speed_vector[3]; //X, Y, Z (relative to the robot torso)
} MOTION_COMMAND;

CONTROL_STATES control_state = SET_COMPOUND_CONTROL;
SERVOS_CONTROL_IDX servo_control_idx = FL_SHOULDER;
PI_FAST_COMMAND pi_frame_command;
PI_COMPOUND_COMMAND pi_compound_command;
MOTION_COMMAND motion_command;

SpotModel model = SpotModel();

// ----------------------------------------------------


typedef struct struct_message {
  int ID_send;
  int POS_send;
  int Spd_send;
} struct_message;


// Create a struct_message called myData
struct_message myData;

uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x93, 0x5F, 0xA8};


//#define ENABLE_DEBUG /* <-- Commenting this line will remove any trace of debug printing */
//#include <MacroDebugger.h>

// Button object to simulate input events
MyButton my_btn(BTN_PIN, NORMAL_DOWN, 50);

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED   23
#define NUMPIXELS 10

// Communication messages
char incoming_msg[MAX_BUFFER_LEN] = {0};
char response[MAX_BUFFER_LEN] = {0};
char flag[MAX_BUFFER_LEN] = {0};
char LoadString[MAX_BUFFER_LEN] = {0};
String Incoming_String;

#include "RGB_CTRL.h"
#include "BOARD_DEV.h"

byte ID[3];
s16 Position[3];
u16 Speed[3];
byte ACC[3];
double speeds_FL[3] = {0.0, 0.0, 0.0};
double speeds_FR[3] = {0.0, 0.0, 0.0};
double speeds_RL[3] = {0.0, 0.0, 0.0};
double speeds_RR[3] = {0.0, 0.0, 0.0};
double angles_FL[3] = {0.0, 0.0, 0.0};
double angles_FR[3] = {0.0, 0.0, 0.0};
double angles_RL[3] = {0.0, 0.0, 0.0};
double angles_RR[3] = {0.0, 0.0, 0.0};

double movementTime = 0.0;
double movementTimeAux = 0.0;
double movementTime_prev = 0.0;

unsigned long startTime = 0;
unsigned long endTime = 0;
unsigned long elapsedTime;
String strElapsedTime;
bool received = false;
int x=0.0;
int y=0.01;
int z= 0.09;

int Speed_Norm = 1500;
bool ini = true;


//Definir as Joints

SpotServo FL_Shoulder, FL_Elbow, FL_Wrist, FR_Shoulder, FR_Elbow, FR_Wrist, RL_Shoulder, RL_Elbow, RL_Wrist, RR_Shoulder, RR_Elbow, RR_Wrist;
SpotServo * Shoulders[4] = {&FL_Shoulder, &FR_Shoulder, &RL_Shoulder, &RR_Shoulder};
SpotServo * Elbows[4] = {&FL_Elbow, &FR_Elbow, &RL_Elbow, &RR_Elbow};
SpotServo * Wrists[4] = {&FL_Wrist, &FR_Wrist, &RL_Wrist, &RR_Wrist};
SpotServo * FR_Servos[3] = {&FR_Shoulder, &FR_Elbow, &FR_Wrist};
SpotServo * FL_Servos[3] = {&FL_Shoulder, &FL_Elbow, &FL_Wrist};
SpotServo * RL_Servos[3] = {&RL_Shoulder, &RL_Elbow, &RL_Wrist};
SpotServo * RR_Servos[3] = {&RR_Shoulder, &RR_Elbow, &RR_Wrist};
SpotServo * Front_Servos[1] = {&FR_Wrist};
SpotServo * Back_Servos[6] = {&RL_Shoulder, &RL_Elbow, &RL_Wrist, &RR_Shoulder, &RR_Elbow, &RR_Wrist};
SpotServo * Servos[12] = {&FL_Shoulder, &FL_Elbow, &FL_Wrist, &FR_Shoulder, &FR_Elbow, &FR_Wrist, &RL_Shoulder, &RL_Elbow, &RL_Wrist, &RR_Shoulder, &RR_Elbow, &RR_Wrist};
SpotServo * Test_Servos [9] = {&FR_Shoulder, &FR_Elbow, &FR_Wrist, &RL_Shoulder, &RL_Elbow, &RL_Wrist, &RR_Shoulder, &RR_Elbow, &RR_Wrist};
Spot Complete_Spot,Front_spot, Test_Spot;
Kinematics IK;
Utilities util;
double Load = 0;

const int nCyclePoints = 19;

/*
double shoulder_list_F[4][nCyclePoints] = {0};
double elbow_list_F[4][nCyclePoints] = {0};
double wrist_list_F[4][nCyclePoints] = {0};

double shoulder_list_B[4][nCyclePoints] = {0};
double elbow_list_B[4][nCyclePoints] = {0};
double wrist_list_B[4][nCyclePoints] = {0};

double shoulder_list_R[4][nCyclePoints] = {0};
double elbow_list_R[4][nCyclePoints] = {0};
double wrist_list_R[4][nCyclePoints] = {0};

double shoulder_list_L[4][nCyclePoints] = {0};
double elbow_list_L[4][nCyclePoints] = {0};
double wrist_list_L[4][nCyclePoints] = {0};

double shoulder_list_RR[4][nCyclePoints] = {0};
double elbow_list_RR[4][nCyclePoints] = {0};
double wrist_list_RR[4][nCyclePoints] = {0};

double shoulder_list_RL[4][nCyclePoints] = {0};
double elbow_list_RL[4][nCyclePoints] = {0};
double wrist_list_RL[4][nCyclePoints] = {0};
*/

double shoulder_list_F[4][nCyclePoints] = {{8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,8},{-8,-9,-9,-9,-9,-9,-9,-9,-9,-8,-8,-8,-8,-8,-8,-8,-9,-9,-9},{8,9,9,9,9,9,9,9,9,8,8,8,8,8,8,8,9,9,9},{-8,-8,-8,-8,-8,-8,-8,-8,-9,-9,-9,-9,-9,-9,-9,-9,-9,-9,-8}};
double elbow_list_F[4][nCyclePoints] = {{-66,-43,-54,-64,-73,-79,-83,-86,-87,-87,-85,-77,-68,-57,-46,-39,-37,-42,-43},{66,87,84,78,69,58,48,39,36,41,52,66,74,79,83,86,88,87,87},{-66,-87,-84,-78,-69,-58,-48,-39,-36,-41,-52,-66,-74,-79,-83,-86,-88,-87,-87},{66,43,54,64,73,79,83,86,87,87,85,77,68,57,46,39,37,42,43}};
double wrist_list_F[4][nCyclePoints] = {{101,95,100,101,100,97,94,94,96,100,105,109,109,107,102,97,94,95,95},{-101,-101,-106,-109,-109,-107,-103,-98,-94,-95,-99,-101,-100,-96,-94,-94,-96,-101,-101},{101,101,106,109,109,107,103,98,94,95,99,101,100,96,94,94,96,101,101},{-101,-95,-100,-101,-100,-97,-94,-94,-96,-100,-105,-109,-109,-107,-102,-97,-94,-95,-95}};

double shoulder_list_B[4][nCyclePoints] = {{8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,9,8,8},{-8,-9,-9,-9,-9,-9,-9,-9,-8,-8,-8,-8,-8,-8,-9,-9,-9,-9,-9},{8,9,9,9,9,9,9,9,8,8,8,8,8,8,9,9,9,9,9},{-8,-8,-8,-8,-8,-8,-9,-9,-9,-9,-9,-9,-9,-9,-9,-9,-9,-8,-8}};
double elbow_list_B[4][nCyclePoints] = {{-66,-70,-68,-63,-60,-61,-62,-64,-66,-67,-69,-72,-73,-75,-76,-76,-74,-72,-70},{66,68,70,72,74,76,76,75,72,71,68,64,60,61,62,63,65,66,68},{-66,-68,-70,-72,-74,-76,-76,-75,-72,-71,-68,-64,-60,-61,-62,-63,-65,-66,-68},{66,70,68,63,60,61,62,64,66,67,69,72,73,75,76,76,74,72,70}};
double wrist_list_B[4][nCyclePoints] = {{101,101,101,101,102,103,105,107,108,109,109,109,109,109,108,106,104,102,101},{-101,-109,-109,-109,-109,-109,-108,-105,-102,-101,-101,-101,-101,-103,-105,-106,-108,-108,-109},{101,109,109,109,109,109,108,105,102,101,101,101,101,103,105,106,108,108,109},{-101,-101,-101,-101,-102,-103,-105,-107,-108,-109,-109,-109,-109,-109,-108,-106,-104,-102,-101}};

double shoulder_list_R[4][nCyclePoints] = {{8,-1,2,6,11,15,19,21,21,20,18,14,10,5,0,-1,-4,-4,-1},{-8,2,0,-4,-9,-14,-18,-20,-20,-19,-15,-11,-6,-2,2,4,5,4,2},{8,19,17,14,9,4,0,-3,-4,-2,1,6,10,14,18,20,21,21,19},{-8,-17,-15,-10,-6,-1,2,4,5,3,0,-4,-9,-13,-17,-19,-20,-20,-17}};
double elbow_list_R[4][nCyclePoints] = {{-66,-71,-69,-67,-64,-61,-58,-58,-60,-62,-65,-68,-72,-74,-76,-77,-76,-73,-71},{66,79,78,75,72,68,64,60,58,57,60,64,67,70,73,75,78,79,79},{-66,-63,-65,-69,-72,-75,-77,-77,-75,-72,-70,-67,-65,-61,-58,-58,-59,-61,-63},{66,58,60,64,68,71,73,77,79,79,78,75,72,69,65,62,59,57,58}};
double wrist_list_R[4][nCyclePoints] ={{101,109,107,103,99,94,90,90,92,95,99,104,109,113,115,116,115,112,109},{-101,-116,-115,-113,-109,-105,-100,-95,-91,-90,-94,-99,-103,-107,-109,-112,-115,-117,-116},{101,97,100,105,109,113,116,116,114,110,107,104,100,95,91,90,91,93,97},{-101,-91,-95,-99,-104,-107,-110,-114,-116,-117,-116,-113,-110,-106,-101,-98,-93,-90,-91}};

double shoulder_list_L[4][nCyclePoints] = {{8,17,15,12,7,3,-1,-3,-5,-4,-2,1,6,10,15,18,20,20,19},{-8,-19,-17,-15,-10,-6,-1,1,4,4,1,-3,-7,-12,-16,-19,-21,-21,-20},{8,-2,0,3,8,12,16,19,20,20,18,13,9,4,0,-3,-4,-5,-3},{-8,1,-2,-5,-9,-14,-18,-19,-21,-21,-19,-16,-12,-8,-3,0,3,4,3}};
double elbow_list_L[4][nCyclePoints] = {{-66,-58,-60,-63,-67,-70,-73,-75,-78,-79,-79,-77,-74,-71,-67,-63,-61,-58,-57},{66,63,65,68,71,74,76,77,76,74,71,69,66,63,60,58,58,60,62},{-66,-79,-78,-76,-73,-70,-66,-63,-59,-57,-58,-62,-65,-69,-72,-74,-76,-79,-79},{66,71,69,68,65,62,59,58,59,60,63,66,70,73,75,77,77,75,72}};
double wrist_list_L[4][nCyclePoints] = {{101,91,95,98,102,106,109,111,115,117,116,115,112,108,104,99,96,92,90},{-101,-97,-100,-103,-108,-112,-115,-116,-116,-113,-109,-106,-102,-98,-93,-90,-90,-92,-95},{101,116,115,114,110,107,102,98,94,91,91,96,101,105,108,111,113,116,117},{-101,-109,-107,-104,-100,-96,-91,-90,-91,-93,-97,-101,-106,-110,-114,-116,-116,-114,-111}};

double shoulder_list_RR[4][nCyclePoints] = {{8,22,13,9,4,-1,-4,-4,-3,-1,0,3,8,14,19,21,20,18,17},{-8,15,6,-3,-12,-18,-25,-29,-30,-27,-20,-11,-3,5,14,19,22,17,14},{8,-14,-6,3,12,18,25,30,31,28,20,11,3,-5,-15,-21,-23,-18,-15},{-8,0,-3,-7,-14,-19,-20,-20,-20,-19,-17,-13,-8,-1,3,4,3,1,1}};
double elbow_list_RR[4][nCyclePoints] = {{-66,-62,-69,-66,-65,-60,-52,-47,-46,-50,-57,-65,-71,-72,-72,-73,-73,-72,-71},{66,73,74,72,72,71,64,56,50,51,60,65,65,62,64,67,71,74,73},{-66,-98,-91,-82,-67,-51,-37,-28,-24,-31,-46,-61,-76,-87,-93,-96,-98,-97,-97},{66,84,77,67,56,45,36,34,35,41,51,63,73,84,91,94,92,86,85}};
double wrist_list_RR[4][nCyclePoints] = {{101,82,95,101,105,107,106,106,107,109,112,113,110,104,96,90,85,85,88},{-101,-120,-118,-114,-107,-99,-88,-76,-69,-73,-88,-99,-105,-109,-113,-116,-119,-121,-120},{101,115,117,114,107,98,83,69,63,69,86,99,105,107,108,111,114,117,117},{-101,-103,-105,-102,-95,-87,-83,-84,-87,-92,-98,-105,-110,-115,-115,-111,-106,-104,-105}};

double shoulder_list_RL[4][nCyclePoints] = {{8,22,13,9,2,-3,-4,-4,-2,-1,1,4,10,16,20,21,20,18,17},{-8,-24,-23,-15,-7,0,10,18,19,15,4,-5,-13,-20,-28,-32,-32,-31,-29},{8,24,22,14,7,0,-9,-17,-18,-14,-4,5,13,20,27,31,32,30,28},{-8,-22,-12,-9,-2,2,2,0,-1,-2,-3,-5,-10,-16,-19,-19,-17,-15,-15}};
double elbow_list_RL[4][nCyclePoints] = {{-66,-63,-69,-66,-64,-57,-50,-46,-48,-52,-60,-68,-72,-72,-72,-73,-73,-71,-70},{66,71,68,72,71,62,55,52,53,55,61,65,68,66,58,53,54,58,61},{-66,-43,-54,-65,-75,-85,-91,-91,-87,-83,-77,-70,-59,-48,-38,-36,-38,-42,-45},{66,30,47,64,78,88,92,93,94,95,92,84,70,54,37,24,20,24,29}};
double wrist_list_RL[4][nCyclePoints] = {{101,82,95,101,106,107,106,106,108,110,112,112,109,102,94,88,86,86,88},{-101,-81,-90,-103,-111,-114,-115,-113,-112,-110,-109,-104,-96,-84,-67,-61,-63,-71,-77},{101,87,93,104,111,116,119,118,116,113,110,104,97,87,74,69,70,76,81},{-101,-77,-94,-101,-105,-102,-95,-91,-93,-98,-106,-110,-109,-100,-90,-81,-78,-80,-84}};

double shoulder_list_custom[4][nCyclePoints] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
double elbow_list_custom[4][nCyclePoints] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
double wrist_list_custom[4][nCyclePoints] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

bool custom_gait_ini = false;

bool get_message(char *msg){
  if (Serial.available() > 0) {
    Serial.readBytes(msg, MAX_BUFFER_LEN);
    return true;
  } else {
    return false;
  }
}

int read_message(char *msg, int *current_len){
  if (Serial.available() > 0) {
    msg[*current_len] = Serial.read();
    *current_len += 1;
    if(msg[*current_len-1] == '\n'){
      return *current_len;
    }
    return 0;
  } else {
    return -1;
  }
}

void send_message(char *msg){
  String msg_str = "esp/msg:";
  msg_str = msg_str + String(msg);
  Serial.write(msg);
}

void update_servos() //considerar mudar para usar o syncwrite
{
  Complete_Spot.Update_Spot(50);
}
 
 
bool all_goals_reached(){
  //int pos = 2047 + ((goal_pose + offset) / 0.087912); //converts angle from degrees to pulses
  
  if (FL_Shoulder.GoalReached() &&
      FL_Elbow.GoalReached() &&
      FL_Wrist.GoalReached() &&
      FR_Shoulder.GoalReached() &&
      FR_Elbow.GoalReached() &&
      FR_Wrist.GoalReached() &&
      RL_Shoulder.GoalReached() &&
      RL_Elbow.GoalReached() &&
      RL_Wrist.GoalReached() &&
      RR_Shoulder.GoalReached() &&
      RR_Elbow.GoalReached() &&
      RR_Wrist.GoalReached()){

    return true;
  } else {
    return false;
  }
}
 
 
 void set_stance(const double & l_shoulder_stance = 0.0, const double & l_elbow_stance = 0.0, const double & l_wrist_stance = 0.0,
                 const double & r_shoulder_stance = 0.0, const double & r_elbow_stance = 0.0, const double & r_wrist_stance = 0.0)
 {
   // Legs
   FL_Shoulder.SetGoal(l_shoulder_stance, 500);
   FL_Elbow.SetGoal(l_elbow_stance, 500);
   FL_Wrist.SetGoal(l_wrist_stance, 500);
 
   FR_Shoulder.SetGoal(r_shoulder_stance, 500);
   FR_Elbow.SetGoal(r_elbow_stance, 500);
   FR_Wrist.SetGoal(r_wrist_stance, 500);
 
   RL_Shoulder.SetGoal(l_shoulder_stance, 500);
   RL_Elbow.SetGoal(l_elbow_stance, 500);
   RL_Wrist.SetGoal(l_wrist_stance, 500);
 
   RR_Shoulder.SetGoal(r_shoulder_stance, 500);
   RR_Elbow.SetGoal(r_elbow_stance, 500);
   RR_Wrist.SetGoal(r_wrist_stance, 500);
   
   Complete_Spot.Update_Spot(0);
   // Loop until goal reached - check BR Wrist (last one)
   while (!all_goals_reached())
   {
     Serial.print("Shoulder: ");
     Serial.println(FL_Shoulder.GetPoseEstimate());
     Serial.print("Elbow: ");
     Serial.println( FL_Elbow.GetPoseEstimate());
     Serial.print("Wrist: ");
     Serial.println(FL_Wrist.GetPoseEstimate());
     Complete_Spot.Update_Spot(50);
   }
 }
 
double Leg_Joint_Speeds(double (& speed) [3],double angles[3],int leg, int speed_const=400){  
  //Calcula as velocidades para cada joint para que acabem todas ao mesmo tempo, atingindo um movimento mais fluido
  double shoulder_dist = abs(angles[0] - (*Shoulders[leg]).GetPoseEstimate());
  double elbow_dist = abs(angles[1] - (*Elbows[leg]).GetPoseEstimate());
  double wrist_dist = abs(angles[2] - (*Wrists[leg]).GetPoseEstimate());

  double scaling_factor = util.max(shoulder_dist, elbow_dist, wrist_dist);

  double dt_movement = scaling_factor/(speed_const * 0.087912);

  shoulder_dist /= scaling_factor;
  elbow_dist /= scaling_factor;
  wrist_dist /= scaling_factor;

  double s_speed = 0.0;
  double e_speed = 0.0;
  double w_speed = 0.0;

  s_speed = speed_const / shoulder_dist;
  e_speed = speed_const / elbow_dist;
  w_speed = speed_const / wrist_dist;

  speed[0] = s_speed;
  speed[1] = e_speed;
  speed[2] = w_speed;

  return dt_movement;
}
 
//void set_stance_wspeed(double shoulder_list[4][nCyclePoints],double elbow_list[4][nCyclePoints],double wrist_list[4][nCyclePoints],bool &received,char (& incoming_msg)[MAX_BUFFER_LEN]){
void set_stance_wspeed(const double & l_shoulder_stance, const double & l_elbow_stance, const double & l_wrist_stance ,
  const double & r_shoulder_stance, const double & r_elbow_stance, const double & r_wrist_stance, double & speed){

  float speedShoulder,speedElbow,speedWrist,speedcalc,positionShoulder,positionElbow,positionWrist,loadShoulder,loadElbow,loadWrist = 0;

  //set_stance(l_shoulder_stance, l_elbow_stance, l_wrist_stance, r_shoulder_stance, r_elbow_stance, r_wrist_stance);
  
  FL_Shoulder.SetGoal(l_shoulder_stance, 500);
  FL_Elbow.SetGoal(l_elbow_stance, speed);
  FL_Wrist.SetGoal(l_wrist_stance, speed);

  FR_Shoulder.SetGoal(r_shoulder_stance, 500);
  FR_Elbow.SetGoal(r_elbow_stance, speed);
  FR_Wrist.SetGoal(r_wrist_stance, speed);

  RL_Shoulder.SetGoal(l_shoulder_stance, 500);
  RL_Elbow.SetGoal(l_elbow_stance, speed);
  RL_Wrist.SetGoal(l_wrist_stance, speed);

  RR_Shoulder.SetGoal(r_shoulder_stance, 500);
  RR_Elbow.SetGoal(r_elbow_stance, speed);
  RR_Wrist.SetGoal(r_wrist_stance, speed);
  
  Complete_Spot.Update_Spot(0);
  // Loop until goal reached - check BR Wrist (last one)
  while ((!all_goals_reached()) && DEBUG_set_stance_wspeed)
  {
    FL_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
    FL_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
    FL_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);

    Serial.print("ESP/FL/Shoulder:");
    Serial.println(positionShoulder);
    Serial.print("ESP/FL/Elbow:");
    Serial.println(positionElbow);
    Serial.print("ESP/FL/Wrist:");
    Serial.println(positionWrist);
  }
}

void dynamic_pose(
  double (& angles_FR)[3], double (& angles_FL)[3], double (& angles_RL)[3], double (& angles_RR)[3],
  double (& speed_FR)[3], double (& speed_FL)[3], double (& speed_RL)[3], double (& speed_RR)[3]
  ){
  Serial.println("ESP/DEBUG/function:Dynamic Pose");

  FR_Shoulder.SetGoal(angles_FR[0], speed_FR[0]);
  FR_Elbow.SetGoal(angles_FR[1], speed_FR[1]);
  FR_Wrist.SetGoal(angles_FR[2], speed_FR[2]);
  FL_Shoulder.SetGoal(angles_FL[0], speed_FL[0]);
  FL_Elbow.SetGoal(angles_FL[1], speed_FL[1]);
  FL_Wrist.SetGoal(angles_FL[2], speed_FL[2]);
  RL_Shoulder.SetGoal(angles_RL[0], speed_RL[0]);
  RL_Elbow.SetGoal(angles_RL[1], speed_RL[1]);
  RL_Wrist.SetGoal(angles_RL[2], speed_RL[2]);
  RR_Shoulder.SetGoal(angles_RR[0], speed_RR[0]);
  RR_Elbow.SetGoal(angles_RR[1], speed_RR[1]);
  RR_Wrist.SetGoal(angles_RR[2], speed_RR[2]);

  Complete_Spot.Update_Spot(0);
}

void perform_gait(double shoulder_list[4][nCyclePoints],double elbow_list[4][nCyclePoints],double wrist_list[4][nCyclePoints],bool &received,char (& incoming_msg)[MAX_BUFFER_LEN]){
  received = get_message(incoming_msg);
  long timeStart, wait_time,timecalc_prev = millis();
  long time_feedback = timeStart/1000;
  long prev_timestamp = wait_time;
  bool starter = true;
  float speedShoulder,speedElbow,speedWrist,speedcalc,positionShoulder,positionElbow,positionWrist,loadShoulder,loadElbow,loadWrist = 0;
  float positionShoulder_prev = positionShoulder;
  float positionElbow_prev = positionElbow;
  float positionWrist_prev = positionWrist;
  memset(incoming_msg, 0, MAX_BUFFER_LEN);
  while(!received){
    for (int i = 0; i <= (nCyclePoints-1); i++){
 
      //r = IK.GetJointAngles(x, y, z, Left, angles_FL);
      angles_FL[0] = shoulder_list[0][i];
      angles_FL[1] = elbow_list[0][i];
      angles_FL[2] = wrist_list[0][i];
      movementTime = Leg_Joint_Speeds(speeds_FL, angles_FL, 0, Speed_Norm);
      FL_Shoulder.SetGoal(angles_FL[0], speeds_FL[0]);
      FL_Elbow.SetGoal(angles_FL[1], speeds_FL[1]);
      FL_Wrist.SetGoal(angles_FL[2], speeds_FL[2]);

      angles_FR[0] = shoulder_list[1][i];
      angles_FR[1] = elbow_list[1][i];
      angles_FR[2] = wrist_list[1][i];
      movementTimeAux = Leg_Joint_Speeds(speeds_FR, angles_FR, 1,Speed_Norm);
      if (movementTimeAux > movementTime){
        movementTime = movementTimeAux;
      }
      
      FR_Shoulder.SetGoal(angles_FR[0], speeds_FR[0]);
      FR_Elbow.SetGoal(angles_FR[1], speeds_FR[1]);
      FR_Wrist.SetGoal(angles_FR[2], speeds_FR[2]);
 
      angles_RL[0] = shoulder_list[2][i];
      angles_RL[1] = elbow_list[2][i];
      angles_RL[2] = wrist_list[2][i];
      movementTimeAux = Leg_Joint_Speeds(speeds_RL, angles_RL, 2,Speed_Norm);
      if (movementTimeAux > movementTime){
        movementTime = movementTimeAux;
      }
      RL_Shoulder.SetGoal(angles_RL[0], speeds_RL[0]);
      RL_Elbow.SetGoal(angles_RL[1], speeds_RL[1]);
      RL_Wrist.SetGoal(angles_RL[2], speeds_RL[2]);
       angles_RR[0] = shoulder_list[3][i];
      angles_RR[1] = elbow_list[3][i];
      angles_RR[2] = wrist_list[3][i];
      movementTimeAux = Leg_Joint_Speeds(speeds_RR, angles_RR, 3, Speed_Norm);
      if (movementTimeAux > movementTime){
        movementTime = movementTimeAux;
      }
      RR_Shoulder.SetGoal(angles_RR[0], speeds_RR[0]);
      RR_Elbow.SetGoal(angles_RR[1], speeds_RR[1]);
      RR_Wrist.SetGoal(angles_RR[2], speeds_RR[2]);
      FL_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
      FL_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
      FL_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);
      
      float time = millis();
      Complete_Spot.getPositionString(LoadString,time);
      Serial.println(LoadString);
 
      wait_time = millis() - prev_timestamp+30;
      if (wait_time < movementTime_prev){ 
        delay(movementTime_prev - wait_time);
      }

      movementTime = movementTime*1000;
      Complete_Spot.Update_Spot(0);

      time = millis();
      Complete_Spot.getPositionString(LoadString,time);
      Serial.println(LoadString);
      //Complete_Spot.getLoadString(LoadString,time);
      //Serial.println(LoadString);

      /*
      
      FL_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
      FL_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
      FL_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);

      time_calc = (millis()-timecalc_prev);
      if (time_calc != 0) {
        //Serial.println("Time Calc: ");Serial.println(time_calc);  
        //Serial.println("Position Shoulder: ");Serial.println(positionShoulder);
        //Serial.println("Position Shoulder Prev: ");Serial.println(positionShoulder_prev);
        speedShoulder = ((((positionShoulder-positionShoulder_prev)/time_calc)*87.912)*3.141)/180;
        speedElbow = ((((positionElbow-positionElbow_prev)/time_calc)*87.912)*3.141)/180;
        speedWrist = ((((positionWrist-positionWrist_prev)/time_calc)*87.912)*3.141)/180;
      } 
      timecalc_prev = millis();
      positionShoulder_prev = positionShoulder;
      positionElbow_prev = positionElbow;
      positionWrist_prev = positionWrist;
      time = millis()-timeStart;
      //sprintf(message, "M0 %.2f; M1 %.2f; M2 %.2f; mode %.2f; e1 %.2f; e2 %.2f; loop %.2f", speedShoulder,loadShoulder,speedElbow,loadElbow,speedWrist,loadWrist,time);
      sprintf(message, " %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", speedShoulder,loadShoulder,speedElbow,loadElbow,speedWrist,loadWrist,time);
      Serial.println(message);*/
      

      //Complete_Spot.getLoadString(LoadString, LoadTime);
      //send_message(LoadString);
      //DEBUG_I("Movement Time: %f", movementTime); 
      prev_timestamp = millis();
      movementTime_prev = movementTime;
      starter = false;

      //while(!all_goals_reached()){
       //  Complete_Spot.Update_Spot(10);
       //}
       //DEBUG_I("ALL REACHED");
       
      if (received = get_message(incoming_msg))
      {
        ini = false;
        return;
      }
    }
  }
  return;
}
 
void Load_Test(SpotServo Servo) {
  double max_angle = 120.0;
  double goal_position = 120.0; // Define your max angle here
  double min_angle = 0.0; // Define your min angle here
  double current_angle = Servo.GetPoseEstimate(); // Get the current angle
  float speed,position,load,speed_calc = 0;
  int chooser = 1;
  int start = millis();
  long time = millis()/1000;
  //servo.Get_Feedback(speed,load,position,speed_calc,time);
  char message[50];
  sprintf(message, " loop %.2f; e1 %.2f; e2 %.2f; %.2f", speed, load,time,position);
  Serial.println(message);
  while(1){
    if (chooser == 0){
      goal_position = max_angle;
      Servo.SetGoal(max_angle,2000); // Set the goal to the max angle
      chooser = 1;
    }
    else if (chooser == 1)
    {
      goal_position = min_angle;
      Servo.SetGoal(min_angle,2000); // Set the goal to the min angle
      chooser = 0;
    }
    else if (chooser == 2)
    {
      goal_position = max_angle;
      Servo.SetGoal(max_angle,2000); // Set the goal to the max angle
      chooser = 4;
    }
    else if (chooser == 3)
    {
      goal_position = min_angle;
      Servo.SetGoal(min_angle,2000); // Set the goal to the min angle
      chooser = 4;
    } 
    else if (chooser == 4){
      goal_position = max_angle;
      Servo.SetGoal(max_angle,1000); // Set the goal to the max angle
      chooser = 5;
    }
    else if (chooser == 5)
    {
      goal_position = min_angle;
      Servo.SetGoal(min_angle,1000); // Set the goal to the min angle
      chooser = 6;
    }
    else if (chooser == 6)
    {
      goal_position = max_angle;
      Servo.SetGoal(max_angle,500); // Set the goal to the max angle
      chooser = 7;
    }
    else if (chooser == 7)
    {
      goal_position = min_angle;
      Servo.SetGoal(min_angle,500); // Set the goal to the min angle
      chooser = 0;
    }
      
    Complete_Spot.Update_Spot(0);
    Serial.print("position:");Serial.println(position);
    Serial.print("goal_position:");Serial.println(goal_position);
    while(abs(position - goal_position) > 2){
        //Servo.Get_Feedback(speed,load,position,speed_calc,time);
        char message[50];
        time = millis()/1000;
        float time_used = millis()-start;
        sprintf(message, " loop %.2f; e1 %.2f; e2 %.2f; %.2f", speed, load,time_used,position);
        Serial.println(message);
        
    }
    Serial.println("Goal Reached");
    delay(1000);
  }
}

 
 
 
 void set_gait(int n_msgs,char incoming_msgs_list[4][MAX_BUFFER_LEN],double (& shoulder_list)[4][nCyclePoints],double (& elbow_list)[4][nCyclePoints],double (& wrist_list)[4][nCyclePoints]){
 
   for (int i = 0; i < n_msgs; i++){
     //DEBUG_I("Received FOR LOOP: %s", incoming_msgs_list[i]);
     char *ptr = incoming_msgs_list[i]; 
     char *str;
     int message_string_index = 0;
     int leg = -1; //0=FL, 1=FR, 2=BL, 3=BR
 
     int nleg = 0;
     // For Servo Calibration Request
     int servo_num = -1;
     double servo_calib_angle = 135.0;
     while ((str = strtok_r(ptr, ",", &ptr)) != NULL) // delimiter is the comma
     {
       // Read Desired Leg
       if(message_string_index == 1)
       {       
         // between 0 and 4
         leg = atoi(str);
         //DEBUG_I("Leg: %d", leg);
         nleg++;
         //DEBUG_I("Leg: %d", leg);
         // Serial1.print(leg);
         if (leg < 0 or leg > 4)
         {
           leg = -1;
         }
       }
       if (leg >= 0 and leg <=3)
       {
         // Read Desired shoulder pos
         if (message_string_index >= 2 and message_string_index <= 1 + nCyclePoints)  //2 + (ncyclep -1)
         {
           if (strcmp(str,"s")==0){
             shoulder_list[leg][message_string_index-2] = shoulder_list[leg][message_string_index-3];
             //DEBUG_I("Shoulder %f",shoulder_list[leg][message_string_index-2]);
           }
           else{
             shoulder_list[leg][message_string_index-2] = atof(str);
             //DEBUG_I("%d Shoulder %f",message_string_index,shoulder_list[leg][message_string_index-2]);
           }
         } 
         else if ((message_string_index >= (nCyclePoints+2)) and (message_string_index <= (2*nCyclePoints+1)))
         {
           if (strcmp(str,"s")==0){
             elbow_list[leg][message_string_index-(nCyclePoints+2)] = elbow_list[leg][message_string_index-(nCyclePoints+3)];
             //DEBUG_I("Elbow %f",elbow_list[leg][message_string_index-(nCyclePoints+2)]);
           }
           else{
             elbow_list[leg][message_string_index-(nCyclePoints+2)] = atof(str);
             //DEBUG_I("%d Elbow %f",message_string_index,elbow_list[leg][message_string_index-(nCyclePoints+2)]);
           }
         } 
         else if (message_string_index >= (2*nCyclePoints+2) and message_string_index <= (3*nCyclePoints+1))
         {
           if (strcmp(str,"s")==0){
             wrist_list[leg][message_string_index-(2*nCyclePoints+2)] = wrist_list[leg][message_string_index-(2*nCyclePoints+3)];
             //DEBUG_I("Wrist %f",wrist_list[leg][message_string_index-(2*nCyclePoints+2)]);
           }
           else{                
             wrist_list[leg][message_string_index-(2*nCyclePoints+2)] = atof(str);
             //DEBUG_I("%d Wrist %f",message_string_index,wrist_list[leg][message_string_index-(2*nCyclePoints+2)]);
           }
         }            
       }
       message_string_index++;
     }
   }
 }
 
void straight_calibration_stance(){
  //set_stance(0,0,0,0,0,0);
  double speed = 500.0;
  set_stance_wspeed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, speed);
}

void prone_calibration_stance(){
  double Left_shoulder_stance = 0.0;
  double Left_elbow_stance =  -90.0;
  double Left_wrist_stance = 120.0;
  double Right_shoulder_stance = 0.0;
  double Right_elbow_stance =  90.0;
  double Right_wrist_stance = -120.0;
  double speed = 500.0;
  //set_stance(Left_shoulder_stance, Left_elbow_stance, Left_wrist_stance, Right_shoulder_stance, Right_elbow_stance, Right_wrist_stance);
  set_stance_wspeed(Left_shoulder_stance, Left_elbow_stance, Left_wrist_stance, Right_shoulder_stance, Right_elbow_stance, Right_wrist_stance, speed);
}
 
 
 
double r;
 
void setup(){
  //DEBUG_BEGIN();
  Serial.begin(115200);
  Serial.println("ESP/LOG:Starting setup!");
  InitRGB();

  RGBcolor(0, 64, 255);

  boardDevInit();
  // setup_wifi();

  // setup_wifi_communicator();

  servoInit();

  pinMode(LED_PIN, OUTPUT);

  //DEBUG_I("Done setting up!");
  RGBoff();

  delay(1000);
  pingAll(true);

  //threadInit();

  ID[0] = 1;
  ID[1] = 2;
  ID[2] = 3;
  Speed[0] = 800;
  Speed[1] = 800;
  Speed[2] = 800;
  ACC[0] = 50;
  ACC[1] = 50;
  ACC[2] = 50;
  Position[0]=2049;
  Position[1]=2049;
  Position[2]=2049;

  FL_Shoulder.Initialize(1, 0, 0, 0, FL, Shoulder);  // 0 | FLS start: 0
  FL_Elbow.Initialize(2, 0, 0, 0, FL, Elbow);        // 0 | FLE start: 0
  FL_Wrist.Initialize(3, 0, 0, 0, FL, Wrist);        // 0 | FLW start: 0

  FR_Shoulder.Initialize(4, 0, 0, 0, FR, Shoulder);  // 0 | FRS start: 0
  FR_Elbow.Initialize(5, 0, 0, 0, FR, Elbow);        // 0 | FRE start: 0
  FR_Wrist.Initialize(6, 0, 0, 0, FR, Wrist);        // 0 | FRW start: 0

  RL_Shoulder.Initialize(7, 0, 0, 0, RL, Shoulder);  // 0 | RLS start: 0
  RL_Elbow.Initialize(8, 0, 0, 0, RL, Elbow);        // 0 | RLE start: 0
  RL_Wrist.Initialize(9, 0, 0, 0, RL, Wrist);        // 0 | RLW start: 0

  RR_Shoulder.Initialize(10, 0, 0, 0, RR, Shoulder);  // 0 | RRS start: 0
  RR_Elbow.Initialize(11, 0, 0, 0, RR, Elbow);        // 0 | RRE start: 0
  RR_Wrist.Initialize(12, 0, 0, 0, RR, Wrist);        // 0 | RRW start: 0

  delay(1000);

  Complete_Spot.Initialize(Servos,12);

  IK.Initialize(0.04, 0.07, 0.11);

  prone_calibration_stance();
  ini = false;

}

void loop(){

  cycle++;
  if (cycle == 4)
    cycle = 0;

  delay(250);
  Eigen::Vector3d orn(0.0, 0.0, 0.0);   // roll, pitch, yaw
  Eigen::Vector3d pos(0.0, 0.0, 0.0);   // body position

  auto joint_angles = model.IK(orn, pos, model.WorldToFoot);

  Serial.println("Joint Angles in Degrees:");
  for (const auto& leg : joint_angles) {
      for (double angle_rad : leg) {
          double angle_deg = angle_rad;
          Serial.print(angle_deg, 3);  // 3 decimal places
          Serial.print(" ");
      }
      Serial.println();
  }

  // Serial.print("ESP/LOOP/CYCLE:");
  // Serial.println(cycle);
  
  if(pos_feedback_toggle){
    if (cycle == 0){ 
      FL_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
      FL_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
      FL_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);
      Serial.print("ESP/FB/FL:");
    } else if (cycle == 1){
      FR_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
      FR_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
      FR_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);
      Serial.print("ESP/FB/FR:");
    } else if (cycle == 2){
      RL_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
      RL_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
      RL_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);
      Serial.print("ESP/FB/RL:");
    } else if (cycle == 3){
      RR_Shoulder.Get_Feedback(speedShoulder,loadShoulder,positionShoulder);
      RR_Elbow.Get_Feedback(speedElbow,loadElbow,positionElbow);
      RR_Wrist.Get_Feedback(speedWrist,loadWrist,positionWrist);
      Serial.print("ESP/FB/RR:");
    }

    Serial.print("Shoulder:");
    Serial.print(positionShoulder);
    Serial.print(",Elbow:");
    Serial.print(positionElbow);
    Serial.print(",Wrist:");
    Serial.println(positionWrist);
  }
 
  if(false)//(ini == true) //MUDAR ISTO
  {
    memset(incoming_msg, 0, MAX_BUFFER_LEN);
    received = get_message(incoming_msg);
    Serial.print("ESP/DEBUG/MAIN_LOOP:Received:");
    Serial.println(incoming_msg);
  }
 
 
  if (false){ //Serial.available()) {
    char incoming_char = Serial.read();
    Serial.print("Received: ");
    Serial.println(incoming_char);

    if (incoming_char == 's') {
      prone_calibration_stance();

    } else if(incoming_char == 'a'){
      straight_calibration_stance();

    }
  }

  if(pi_frame_command.new_command){
    pi_frame_command.new_command = false;

    switch(pi_frame_command.command){
      case SET_STANCE_STRAIGHT:
        straight_calibration_stance();
        break;
      case SET_STANCE_PRONE:
        prone_calibration_stance();
        break;
        
      case SET_SERVO_FRAME:
        for(int i = 0; i < 12; i++){
          if(PRINT){
            Serial.print("Servo: ");
            Serial.print(i);
            Serial.print(" Pos: ");
            Serial.print(pi_frame_command.pos[i]);
            Serial.print(" Speed: ");
            Serial.println(pi_frame_command.speed[i]);
          }
          Servos[i]->SetGoal(pi_frame_command.pos[i], pi_frame_command.speed[i]);
        }
        Complete_Spot.Update_Spot(0);
        break;
      
        case SET_COMPOUND_CONTROL:
          

      case TOGGLE_LOG:
        if(log_toggle){
          Serial.println("ESP/LOG:Log:true");
        }
        log_toggle = !log_toggle;
        break;

      case SET_FEEDBACK:
        pos_feedback_toggle = !pos_feedback_toggle;
        if(log_toggle){
          Serial.print("ESP/POS_FB:Feedback:");
          Serial.println(pos_feedback_toggle);
        }
        break;

      default:
        break;
    }
  }
   
  if(received){
    char incoming_msgs_list[4][MAX_BUFFER_LEN] = {0};
    uint8_t start = 0;
    startTime = micros();
    //send_message(incoming_msg);
    flag[0] = incoming_msg[0];
    ini = true;
    

    if(flag[0] == 'A'){
      sprintf(response, "%s", ACK);
      start++;
    }

    if (flag[0] == 't'){
      memset(incoming_msg, 0, MAX_BUFFER_LEN);
        
      double l_shoulder = 0.0, l_elbow = -50, l_wrist = 90.0;
      double r_shoulder = 0.0, r_elbow = 50, r_wrist = -90;
      double speed = 45 / 0.087912;
      float timee = millis();
      set_stance_wspeed(l_shoulder, l_elbow, l_wrist, r_shoulder, r_elbow, r_wrist, speed);
      timee = millis() - timee;
      Serial.print("Speed: ");
      Serial.print(speed);
      Serial.print("Time: ");
      Serial.println(timee/1000);

    } else if(flag[0] == 's'){
      memset(incoming_msg, 0, MAX_BUFFER_LEN);
      prone_calibration_stance();

    } else if(flag[0] == 'a'){
      memset(incoming_msg, 0, MAX_BUFFER_LEN);
      straight_calibration_stance();

    } else if(flag[0] == 'p'){
      
      double angles_FR_[3] = {0,0,0}, angles_FL_[3] = {0,0,0}, angles_RL_[3] = {0,0,0}, angles_RR_[3] = {0,0,0};
      double spd = 22.5 / 0.087912;
      double speed_FR_[3] = {spd, spd, spd}, speed_FL_[3] = {spd, spd, spd}, speed_RL_[3] = {spd, spd, spd}, speed_RR_[3] = {spd, spd, spd};
      
      dynamic_pose(angles_FR_, angles_FL_, angles_RL_, angles_RR_, speed_FR_, speed_FL_, speed_RL_, speed_RR_);
    } 

    // if (flag[0] == 'C'){     //CALIBRATION
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   Load_Test(FL_Elbow);
    // }
    // else if (flag[0] == 'P'){ //PRONE
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   prone_calibration_stance();
    // }
    // else if (flag[0] == '8'){ //PRONE
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   prone_calibration_stance();
    //   exit(0);
    // }
    
    // else if(flag[0]=='1'){ //FORWARD
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   DEBUG_I("1");
    //   perform_gait(shoulder_list_F,elbow_list_F,wrist_list_F,received,incoming_msg);
    // }
    // else if(flag[0]=='2'){ //BACKWARD
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   DEBUG_I("2");
    //   perform_gait(shoulder_list_B,elbow_list_B,wrist_list_B,received,incoming_msg);
    // }
    // else if(flag[0]=='3'){ //RIGHT
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   DEBUG_I("3");
    //   perform_gait(shoulder_list_R,elbow_list_R,wrist_list_R,received,incoming_msg);
    // }
    // else if(flag[0]=='4'){ //LEFT
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   DEBUG_I("4");
    //   perform_gait(shoulder_list_L,elbow_list_L,wrist_list_L,received,incoming_msg);
    // }
    // else if(flag[0]=='5'){ //RIGHT ROTATION
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   DEBUG_I("5");
    //   perform_gait(shoulder_list_RR,elbow_list_RR,wrist_list_RR,received,incoming_msg);
    // }
    // else if(flag[0]=='6'){ //LEFT ROTATION
    //   memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //   DEBUG_I("6");
    //   perform_gait(shoulder_list_RL,elbow_list_RL,wrist_list_RL,received,incoming_msg);
    // }
    // else if (flag[0]=='7'){
    //   if (custom_gait_ini){
    //     memset(incoming_msg, 0, MAX_BUFFER_LEN);
    //     DEBUG_I("7");
    //     perform_gait(shoulder_list_custom,elbow_list_custom,wrist_list_custom,received,incoming_msg);
    //   }
    //   else{
    //     send_message("GNo Custom Gait Loaded!");
    //   }
    // }
    received = false;
  }
}


void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer[bufferPos] = '\0';  // Null-terminate string
      if (bufferPos > 0) {
        if(inputBuffer[0] == '<' && inputBuffer[1] == (I_TO_A(SET_SERVO_FRAME))) {
          processPiCommandServoFrame(inputBuffer + 1);
        } else if (inputBuffer[0] == '<' && inputBuffer[1] == (I_TO_A(SET_COMPOUND_CONTROL))) {
          processCompoundCommand(inputBuffer + 1);
        } else if (inputBuffer[0] == '<' && inputBuffer[1] == (I_TO_A(SET_MOTION_COMMAND))) {
          processMotionCommand(inputBuffer + 1);
        } else if 
        else if (inputBuffer[0] == '>') {
          processTerminalCommand(inputBuffer + 1);
        } else {
          Serial.println("[ERROR] Unknown command prefix");
        }
      }
      bufferPos = 0;  // Reset buffer
    } else if (bufferPos < COMMAND_BUFFER_SIZE - 1) {
      inputBuffer[bufferPos++] = c;
    } else {
      Serial.println("[ERROR] Command too long");
      bufferPos = 0;
    }
  }
}

// --- Machine Mode Handler ---
void processPiCommand(const char* cmd) {
  Serial.print("ESP/LOG/SerialEvent:Received:");
  Serial.println(cmd);

  // Example: LRS/pos:120.3
  char part[16];
  char type[16];
  float value;

  int matched = sscanf(cmd, "%15[^/]/%15[^:]:%f", part, type, &value);
  if (matched == 3) {
    Serial.print("[PI] Target: "); Serial.println(part);
    Serial.print("[PI] Type: "); Serial.println(type);
    Serial.print("[PI] Value: "); Serial.println(value,8);

    // TODO: Match `part` and `type` and apply SetGoal or similar logic
  } else {
    Serial.println("[ERROR] Malformed Pi command");
  }
}

void processMotionCommand(const char* cmd) {
  Serial.print("[PI] Received:");
  Serial.println(cmd);

  char buf[COMMAND_BUFFER_SIZE];
  strncpy(buf, cmd, COMMAND_BUFFER_SIZE);
  buf[COMMAND_BUFFER_SIZE - 1] = '\0';
  
  char* token = strtok(buf, ":");
}

void processCompoundCommand(const char* cmd) {
  Serial.print("[PI] Received:");
  Serial.println(cmd);
  
  char buf[COMMAND_BUFFER_SIZE];
  strncpy(buf, cmd, COMMAND_BUFFER_SIZE);
  buf[COMMAND_BUFFER_SIZE - 1] = '\0';
  
  char* token = strtok(buf, ":");
  pi_compound_command.command = atoi(token);


  pi_compound_command.new_command = true;
}

void processPiCommandServoFrame(const char* cmd){
  Serial.print("[PI] Received:");
  Serial.println(cmd);
  
  char buf[COMMAND_BUFFER_SIZE];
  strncpy(buf, cmd, COMMAND_BUFFER_SIZE);
  buf[COMMAND_BUFFER_SIZE - 1] = '\0';
  

  char* token = strtok(buf, ":");

  pi_frame_command.command = atoi(token);
  
  int i = 0;
  int idx = 0;
  switch(pi_frame_command.command){
    case SET_SERVO_FRAME:
      token = strtok(NULL, ",");
      while(token != NULL){
        if(i%2 == 0){
          pi_frame_command.pos[idx]=atof(token);
        }else{
          pi_frame_command.speed[idx]=atof(token);
          idx++;
        }

        token = strtok(NULL, ",");
        i++;
      }

      if(i < 24){
        Serial.println("[ERROR] Malformed Pi command, not enought values");
        return;
      }
      break;

    case SET_FEEDBACK:
      token = strtok(NULL, ",");
      pos_feedback_toggle = (atoi(token)>0)?true:false;
      break;
  }
  if(pi_frame_command.command == SET_SERVO_FRAME){
    
  } //else, they are other commands that don't need values
  pi_frame_command.new_command = true;
}
// --- Terminal Mode Handler ---
void processTerminalCommand(const char* cmd) {
  Serial.print("[TERMINAL] Received: ");
  Serial.println(cmd);

  char command[32];
  float arg1, arg2;

  // Check for no-argument command
  if (sscanf(cmd, "%31s", command) == 1) {
    if (strcmp(command, "prone") == 0) {
      Serial.println("[ACTION] Going prone");
      // prone(); // Call your actual function
    }
  }

  // Check for command with two arguments
  if (sscanf(cmd, "%31s %f %f", command, &arg1, &arg2) == 3) {
    if (strcmp(command, "updateangle") == 0) {
      Serial.print("[ACTION] Updating servo "); Serial.print(arg1);
      Serial.print(" to angle "); Serial.println(arg2);
      // updateangle(arg1, arg2); // Call your actual function
    }
  }
}