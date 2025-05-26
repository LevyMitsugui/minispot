
#include "config.h"
#include <Kinematics.hpp>
#include "SpotServo.hpp"
#include <Utilities.hpp>
#include "Spot.hpp"
#include <cstring>
#include "Arduino.h"

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

// ----------------- ADAPT VARIABLES ------------------

#include <ArduinoEigen.h>
#include "Kinematics.hpp"
#include "SpotModel.hpp"
#include "LieAlgebra.hpp"

#define PRINT false
#define DEBUG_set_stance_wspeed false
#define COMMAND_BUFFER_SIZE 256

double shoulder_length = 0.045;
double elbow_length = 0.08;
double wrist_length = 0.103;
double hip_x = 0.185;
double hip_y = 0.077;
double foot_x = 0.185;
double foot_y = 0.17;
double height = 0.145;

bool pos_feedback_toggle = false;
bool log_toggle = false;

int cycle = 0;
float speedShoulder, speedElbow, speedWrist, speedcalc, positionShoulder, positionElbow, positionWrist, loadShoulder, loadElbow, loadWrist = 0;

char inputBuffer[COMMAND_BUFFER_SIZE] = {0};
int bufferPos = 0;

void processPiCommand(const char *cmd);
int parseServoFrame(const char *buf);
int parseOffsetLean(const char *buf);
void process_stream_control(char c);

enum CONTROL_STATES
{
  IDLE,                // 0
  SET_SERVO_FRAME,     // 1
  SET_OFFSET_LEAN,     // 2
  SET_STANCE_STRAIGHT, // 3
  SET_STANCE_PRONE,    // 4
  TOGGLE_LOG,          // 5
  SET_FEEDBACK         // 6
};
enum SERVOS_CONTROL_IDX
{ // Follows indexes from the declaration "SpotServo * Servos[12]" below
  FL_SHOULDER_CONTROL,
  FL_ELBOW_CONTROL,
  FL_WRIST_CONTROL,
  FR_SHOULDER_CONTROL,
  FR_ELBOW_CONTROL,
  FR_WRIST_CONTROL,
  RL_SHOULDER_CONTROL,
  RL_ELBOW_CONTROL,
  RL_WRIST_CONTROL,
  RR_SHOULDER_CONTROL,
  RR_ELBOW_CONTROL,
  RR_WRIST_CONTROL
};

typedef struct OFFSET_LEAN_FRAME
{
  float rpy[3]; // Roll, Pitch, Yaw
  float xyz[3]; // X, Y, Z
  float speed;
} OFFSET_LEAN_FRAME;

typedef struct SERVO_FRAME
{
  float pos[12];
  float speed[12];
} SERVO_FRAME;

typedef struct PI_COMMAND
{
  bool new_command;
  int command;

  char package[COMMAND_BUFFER_SIZE];
  int package_len;

  void *parser_struct;
} PI_COMMAND;

typedef struct FEET_STREAM_CONTROL
{
  float step;
  int selected;
  std::array<Eigen::Vector3d, 4> feet_shifts;
} FEET_STREAM_CONTROL;

SERVO_FRAME servo_frame;
CONTROL_STATES control_state = IDLE;
SERVOS_CONTROL_IDX servo_control_idx = FL_SHOULDER_CONTROL;
OFFSET_LEAN_FRAME offset_lean_frame = {{0}, {0}};
PI_COMMAND pi_command = {false, 0, {0}, 0, NULL};
FEET_STREAM_CONTROL feet_stream_control = {0.01, 0, {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)}};

// helper, TODO remove
const std::array<std::string, 4> leg_order = {"FL", "FR", "BL", "BR"};

bool new_command = false;
bool update_motors = false;
bool stream_control = false;

Spot miniSpot = Spot();

SpotModel model = SpotModel();
int iterator = 0;
Eigen::Vector3d orn(0, 0, 0); // roll, pitch, yaw
Eigen::Vector3d pos(0, 0, 0); // body position
auto joint_angles = model.IK(orn, pos, model.WorldToFoot);
#define SPEED 125
// ----------------------------------------------------

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED 23
#define NUMPIXELS 10

#include "RGB_CTRL.h"

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

// int x=0.0;
// int y=0.01;
// int z=0.09;

int Speed_Norm = 1500;
bool ini = true;

// Definir as Joints

// SpotServo FL_Shoulder, FL_Elbow, FL_Wrist, FR_Shoulder, FR_Elbow, FR_Wrist, RL_Shoulder, RL_Elbow, RL_Wrist, RR_Shoulder, RR_Elbow, RR_Wrist;
// SpotServo * Shoulders[4] = {&FL_Shoulder, &FR_Shoulder, &RL_Shoulder, &RR_Shoulder};
// SpotServo * Elbows[4] = {&FL_Elbow, &FR_Elbow, &RL_Elbow, &RR_Elbow};
// SpotServo * Wrists[4] = {&FL_Wrist, &FR_Wrist, &RL_Wrist, &RR_Wrist};
// SpotServo * FR_Servos[3] = {&FR_Shoulder, &FR_Elbow, &FR_Wrist};
// SpotServo * FL_Servos[3] = {&FL_Shoulder, &FL_Elbow, &FL_Wrist};
// SpotServo * RL_Servos[3] = {&RL_Shoulder, &RL_Elbow, &RL_Wrist};
// SpotServo * RR_Servos[3] = {&RR_Shoulder, &RR_Elbow, &RR_Wrist};
// SpotServo * Front_Servos[1] = {&FR_Wrist};
// SpotServo * Back_Servos[6] = {&RL_Shoulder, &RL_Elbow, &RL_Wrist, &RR_Shoulder, &RR_Elbow, &RR_Wrist};
// SpotServo * Servos[12] = {&FL_Shoulder, &FL_Elbow, &FL_Wrist, &FR_Shoulder, &FR_Elbow, &FR_Wrist, &RL_Shoulder, &RL_Elbow, &RL_Wrist, &RR_Shoulder, &RR_Elbow, &RR_Wrist};
// SpotServo * Test_Servos [9] = {&FR_Shoulder, &FR_Elbow, &FR_Wrist, &RL_Shoulder, &RL_Elbow, &RL_Wrist, &RR_Shoulder, &RR_Elbow, &RR_Wrist};
// Spot Complete_Spot,Front_spot, Test_Spot;
Kinematics IK;
Utilities util;
double Load = 0;

void send_message(const char *msg)
{
  String msg_str = "ESP/MSG:";
  msg_str = msg_str + String(msg);
  Serial.write(msg);
}

/*double Leg_Joint_Speeds(double (& speed) [3],double angles[3],int leg, int speed_const=400){  //TODO ver se eu consigo usar isso aqui
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
}*/

// void set_stance_wspeed(double shoulder_list[4][nCyclePoints],double elbow_list[4][nCyclePoints],double wrist_list[4][nCyclePoints],bool &received,char (& incoming_msg)[MAX_BUFFER_LEN]){
void set_stance_wspeed(Spot miniSpot, const double &l_shoulder_stance, const double &l_elbow_stance, const double &l_wrist_stance,
                       const double &r_shoulder_stance, const double &r_elbow_stance, const double &r_wrist_stance, double &speed)
{

  float speedShoulder, speedElbow, speedWrist, speedcalc, positionShoulder, positionElbow, positionWrist, loadShoulder, loadElbow, loadWrist = 0;

  // set_stance(l_shoulder_stance, l_elbow_stance, l_wrist_stance, r_shoulder_stance, r_elbow_stance, r_wrist_stance);

  miniSpot.Servo_List[FL_SHOULDER].SetGoal(l_shoulder_stance, 500);
  miniSpot.Servo_List[FL_ELBOW].SetGoal(l_elbow_stance, speed);
  miniSpot.Servo_List[FL_WRIST].SetGoal(l_wrist_stance, speed);
  miniSpot.Servo_List[FR_SHOULDER].SetGoal(r_shoulder_stance, 500);
  miniSpot.Servo_List[FR_ELBOW].SetGoal(r_elbow_stance, speed);
  miniSpot.Servo_List[FR_WRIST].SetGoal(r_wrist_stance, speed);
  miniSpot.Servo_List[RL_SHOULDER].SetGoal(l_shoulder_stance, 500);
  miniSpot.Servo_List[RL_ELBOW].SetGoal(l_elbow_stance, speed);
  miniSpot.Servo_List[RL_WRIST].SetGoal(l_wrist_stance, speed);
  miniSpot.Servo_List[RR_SHOULDER].SetGoal(r_shoulder_stance, 500);
  miniSpot.Servo_List[RR_ELBOW].SetGoal(r_elbow_stance, speed);
  miniSpot.Servo_List[RR_WRIST].SetGoal(r_wrist_stance, speed);

  // miniSpot->Servo_List[FL_SHOULDER].SetGoal(l_shoulder_stance, 500);
  // miniSpot->Servo_List[FL_ELBOW].SetGoal(l_elbow_stance, speed);
  // miniSpot->Servo_List[FL_WRIST].SetGoal(l_wrist_stance, speed);
  // miniSpot->Servo_List[FR_SHOULDER].SetGoal(r_shoulder_stance, 500);
  // miniSpot->Servo_List[FR_ELBOW].SetGoal(r_elbow_stance, speed);
  // miniSpot->Servo_List[FR_WRIST].SetGoal(r_wrist_stance, speed);
  // miniSpot->Servo_List[RL_SHOULDER].SetGoal(l_shoulder_stance, 500);
  // miniSpot->Servo_List[RL_ELBOW].SetGoal(l_elbow_stance, speed);
  // miniSpot->Servo_List[RL_WRIST].SetGoal(l_wrist_stance, speed);
  // miniSpot->Servo_List[RR_SHOULDER].SetGoal(r_shoulder_stance, 500);
  // miniSpot->Servo_List[RR_ELBOW].SetGoal(r_elbow_stance, speed);
  // miniSpot->Servo_List[RR_WRIST].SetGoal(r_wrist_stance, speed);

  // miniSpot.Update_Spot(0);
  miniSpot.Update_Spot(0);

  // Loop until goal reached - check BR Wrist (last one)
  // while ((!miniSpot.all_goals_reached()) && DEBUG_set_stance_wspeed)
  // {
  //   miniSpot.Servo_List[FL_SHOULDER].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
  //   miniSpot.Servo_List[FL_ELBOW].Get_Feedback(speedElbow, loadElbow, positionElbow);
  //   miniSpot.Servo_List[FL_WRIST].Get_Feedback(speedWrist, loadWrist, positionWrist);

  //   Serial.print("ESP/FL/Shoulder:");
  //   Serial.println(positionShoulder);
  //   Serial.print("ESP/FL/Elbow:");
  //   Serial.println(positionElbow);
  //   Serial.print("ESP/FL/Wrist:");
  //   Serial.println(positionWrist);
  // }
}

void straight_calibration_stance(Spot miniSpot)
{
  // set_stance(0,0,0,0,0,0);
  double speed = 500.0;
  set_stance_wspeed(miniSpot,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, speed);
}

void prone_calibration_stance(Spot miniSpot)
{
  double Left_shoulder_stance = 0.0;
  double Left_elbow_stance = -90.0;
  double Left_wrist_stance = 120.0;
  double Right_shoulder_stance = 0.0;
  double Right_elbow_stance = 90.0;
  double Right_wrist_stance = -120.0;
  double speed = 500.0;
  // set_stance(Left_shoulder_stance, Left_elbow_stance, Left_wrist_stance, Right_shoulder_stance, Right_elbow_stance, Right_wrist_stance);
  set_stance_wspeed(miniSpot, Left_shoulder_stance, Left_elbow_stance, Left_wrist_stance, Right_shoulder_stance, Right_elbow_stance, Right_wrist_stance, speed);
}

void setup()
{
  // DEBUG_BEGIN();
  Serial.begin(115200);
  delay(5000);
  Serial.println("ESP/LOG:Starting setup!");
  InitRGB();

  // miniSpot = new Spot();

  RGBcolor(0, 64, 255);

  //servoInit();

  pinMode(LED_PIN, OUTPUT);

  RGBoff(); // TODO probably will be removed

  delay(5000);

  // FL_Shoulder.Initialize(1, 0, 0, 0, FL, Shoulder);  // 0 | FLS start: 0
  // FL_Elbow.Initialize(2, 0, 0, 0, FL, Elbow);        // 0 | FLE start: 0
  // FL_Wrist.Initialize(3, 0, 0, 0, FL, Wrist);        // 0 | FLW start: 0

  // FR_Shoulder.Initialize(4, 0, 0, 0, FR, Shoulder);  // 0 | FRS start: 0
  // FR_Elbow.Initialize(5, 0, 0, 0, FR, Elbow);        // 0 | FRE start: 0
  // FR_Wrist.Initialize(6, 0, 0, 0, FR, Wrist);        // 0 | FRW start: 0

  // RL_Shoulder.Initialize(7, 0, 0, 0, RL, Shoulder);  // 0 | RLS start: 0
  // RL_Elbow.Initialize(8, 0, 0, 0, RL, Elbow);        // 0 | RLE start: 0
  // RL_Wrist.Initialize(9, 0, 0, 0, RL, Wrist);        // 0 | RLW start: 0

  // RR_Shoulder.Initialize(10, 0, 0, 0, RR, Shoulder);  // 0 | RRS start: 0
  // RR_Elbow.Initialize(11, 0, 0, 0, RR, Elbow);        // 0 | RRE start: 0
  // RR_Wrist.Initialize(12, 0, 0, 0, RR, Wrist);        // 0 | RRW start: 0

  delay(1000);

  // Complete_Spot.Initialize(Servos,12);

  IK.Initialize(0.04, 0.07, 0.11);

  miniSpot.Init_Servos();
  servoInit();


  prone_calibration_stance(miniSpot);
  Serial.println("ESP/LOG:Completed setup!");
  ini = false;
}

void loop()
{

  cycle++;
  if (cycle == 4)
    cycle = 0;

  // Serial.print("ESP/LOOP/CYCLE:");
  // Serial.println(cycle);

  if (pos_feedback_toggle)
  {
    if (cycle == 0)
    {
      miniSpot.Servo_List[FL_SHOULDER].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
      miniSpot.Servo_List[FL_ELBOW].Get_Feedback(speedElbow, loadElbow, positionElbow);
      miniSpot.Servo_List[FL_WRIST].Get_Feedback(speedWrist, loadWrist, positionWrist);
      Serial.print("ESP/FB/FL:");
    }
    else if (cycle == 1)
    {
      miniSpot.Servo_List[FR_SHOULDER].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
      miniSpot.Servo_List[FR_ELBOW].Get_Feedback(speedElbow, loadElbow, positionElbow);
      miniSpot.Servo_List[FR_WRIST].Get_Feedback(speedWrist, loadWrist, positionWrist);
      Serial.print("ESP/FB/FR:");
    }
    else if (cycle == 2)
    {
      miniSpot.Servo_List[RL_SHOULDER].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
      miniSpot.Servo_List[RL_ELBOW].Get_Feedback(speedElbow, loadElbow, positionElbow);
      miniSpot.Servo_List[RL_WRIST].Get_Feedback(speedWrist, loadWrist, positionWrist);
      Serial.print("ESP/FB/RL:");
    }
    else if (cycle == 3)
    {
      miniSpot.Servo_List[RR_SHOULDER].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
      miniSpot.Servo_List[RR_ELBOW].Get_Feedback(speedElbow, loadElbow, positionElbow);
      miniSpot.Servo_List[RR_WRIST].Get_Feedback(speedWrist, loadWrist, positionWrist);
      Serial.print("ESP/FB/RR:");
    }

    Serial.print("Shoulder:");
    Serial.print(positionShoulder);
    Serial.print(",Elbow:");
    Serial.print(positionElbow);
    Serial.print(",Wrist:");
    Serial.println(positionWrist);
  }

  if (pi_command.new_command)
  {
    pi_command.new_command = false;

    switch (pi_command.command)
    {
    case SET_STANCE_STRAIGHT:
      straight_calibration_stance(miniSpot);
      break;
    case SET_STANCE_PRONE:
      prone_calibration_stance(miniSpot);
      break;

    case SET_SERVO_FRAME:
      if (parseServoFrame(pi_command.package + 2) < 0)
        break;
      for (int i = 0; i < 12; i++)
      {
        if (PRINT)
        {
          Serial.print("Servo: ");
          Serial.print(i);
          Serial.print(" Pos: ");
          Serial.print(servo_frame.pos[i]);
          Serial.print(" Speed: ");
          Serial.println(servo_frame.speed[i]);
        }
        miniSpot.Servo_List[i].SetGoal(servo_frame.pos[i], servo_frame.speed[i]);
      }
      miniSpot.Update_Spot(0);
      break;

    case SET_OFFSET_LEAN:
      if (parseOffsetLean(pi_command.package + 2) < 0)
        // if(parseServoFrame(pi_command.package+2)<0)
        break;
      // Serial.println("Offset Lean:");
      orn.x() = offset_lean_frame.rpy[0];
      orn.y() = offset_lean_frame.rpy[1];
      orn.z() = offset_lean_frame.rpy[2];

      pos.x() = offset_lean_frame.xyz[0];
      pos.y() = offset_lean_frame.xyz[1];
      pos.z() = offset_lean_frame.xyz[2];

      Serial.print("roll: ");
      Serial.print(orn.x(), 3);
      Serial.print(", pitch: ");
      Serial.print(orn.y(), 3);
      Serial.print(", yaw: ");
      Serial.print(orn.z(), 3);
      Serial.print(", x: ");
      Serial.print(pos.x(), 3);
      Serial.print(", y: ");
      Serial.print(pos.y(), 3);
      Serial.print(", z: ");
      Serial.println(pos.z(), 3);

      joint_angles = model.IK(orn, pos, model.WorldToFoot);

      // Serial.println("Joint Angles in Degrees:");
      iterator = 0;
      for (const auto &leg : joint_angles)
      {
        for (double angle_rad : leg)
        {
          if (update_motors)
            miniSpot.Servo_List[iterator].SetGoal(angle_rad * 180 / M_PI, offset_lean_frame.speed);
          iterator += 1;
          Serial.print(angle_rad * 180 / M_PI, 3); // 3 decimal places
          Serial.print(" ");
        }
        Serial.println();
      }
      if (update_motors)
        miniSpot.Update_Spot(0);
      break;

    case TOGGLE_LOG:
      if (log_toggle)
      {
        Serial.println("ESP/LOG:Log:true");
      }
      log_toggle = !log_toggle;
      break;

    case SET_FEEDBACK:
      pos_feedback_toggle = !pos_feedback_toggle;
      if (log_toggle)
      {
        Serial.print("ESP/POS_FB:Feedback:");
        Serial.println(pos_feedback_toggle);
      }
      break;

    default:
      break;
    }
    // Serial.println("out of switch");
  }

  if (stream_control)
  {
    joint_angles = model.IKWithFootOverrides(orn, pos, feet_stream_control.feet_shifts, leg_order);

    iterator = 0;
    for (const auto &leg : joint_angles)
    {
      for (double angle_rad : leg)
      {
        miniSpot.Servo_List[iterator].SetGoal(angle_rad * 180 / M_PI, offset_lean_frame.speed);
        iterator += 1;
        Serial.print(angle_rad * 180 / M_PI, 3); // 3 decimal places
        Serial.print(" ");
      }
      Serial.println();
    }

    miniSpot.Update_Spot(0);
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' && !stream_control)
    {
      inputBuffer[bufferPos] = '\0'; // Null-terminate string
      if (bufferPos > 0)
      {
        if (inputBuffer[0] == '<')
        {
          processPiCommand(inputBuffer + 1);
        }
        else if (inputBuffer[0] == '>')
        { // handle stream mode
          Serial.println("ESP/STREAM:Stream Control Enabled");
          stream_control = true;
        }
        else if (inputBuffer[0] == '/' && inputBuffer[1] == 'k')
        {
          ESP.restart();
        }
        else if (inputBuffer[0] == 'm')
        {
          update_motors = !update_motors;
          String _msg = "Motors: " + String(update_motors);
          send_message(_msg.c_str());
        }
        else
        {
          Serial.println("[ERROR] Unknown command prefix");
        }
      }
      bufferPos = 0; // Reset buffer
    }
    else if (!stream_control && (bufferPos < COMMAND_BUFFER_SIZE - 1))
    {
      inputBuffer[bufferPos++] = c;
    }
    else if (stream_control)
    {
      process_stream_control(c);
    }
    else
    {
      Serial.println("[ERROR] Command too long");
      bufferPos = 0;
    }
  }
}

// --- Machine Mode Handler ---
void processPiCommand(const char *cmd)
{
  // Serial.print("ESP/LOG/SerialEvent:Received:");
  // Serial.println(cmd);

  // char buf[COMMAND_BUFFER_SIZE];
  // strncpy(buf, cmd, COMMAND_BUFFER_SIZE);
  // buf[COMMAND_BUFFER_SIZE - 1] = '\0';
  strncpy(pi_command.package, cmd, COMMAND_BUFFER_SIZE);
  pi_command.package[COMMAND_BUFFER_SIZE - 1] = '\0';

  char *token = strtok(pi_command.package, ":");
  pi_command.command = atoi(token);
  pi_command.new_command = true;
  // strncpy(pi_command.package, buf, COMMAND_BUFFER_SIZE);
}

int parseServoFrame(char *buf)
{
  int i = 0;
  int idx = 0;

  char *token = strtok(buf, ",");
  while (token != NULL)
  {
    if (i % 2 == 0)
    {
      servo_frame.pos[idx] = atof(token);
    }
    else
    {
      servo_frame.speed[idx] = atof(token);
      idx++;
    }
    token = strtok(NULL, ",");
    i++;
  }
  if (i < 24)
  {
    return -1;
    Serial.println("[ERROR] Invalid servo frame");
  }
  return 0;
}

int parseOffsetLean(char *buf)
{
  char *token = strtok(buf, ",");
  for (int i = 0; i < 3; i++)
  {
    // Serial.println(token);
    offset_lean_frame.xyz[i] = atof(token);
    token = strtok(NULL, ",");
    // Serial.println(token);
    offset_lean_frame.rpy[i] = atof(token);
    token = strtok(NULL, ",");

    if (i == 2)
    {
      offset_lean_frame.speed = atof(token);
    }
  }
  return 0;
}

void process_stream_control(char c)
{
  switch (c)
  {
  case 'q':
    stream_control = false;
    Serial.println("ESP/STREAM:Stream Control Disabled");
    break;
  case 'm':
    feet_stream_control.step = feet_stream_control.step + 0.01;
    break;
  case 'n':
    feet_stream_control.step = feet_stream_control.step - 0.01;
    break;
  case '1':
    feet_stream_control.selected = 0;
    break;
  case '2':
    feet_stream_control.selected = 1;
    break;
  case '3':
    feet_stream_control.selected = 2;
    break;
  case '4':
    feet_stream_control.selected = 3;
    break;
  case 'i':
    feet_stream_control.feet_shifts[feet_stream_control.selected].x() = feet_stream_control.feet_shifts[feet_stream_control.selected].x() + feet_stream_control.step;
    break;
  case 'k':
    feet_stream_control.feet_shifts[feet_stream_control.selected].x() = feet_stream_control.feet_shifts[feet_stream_control.selected].x() - feet_stream_control.step;
    break;
  case 'j':
    feet_stream_control.feet_shifts[feet_stream_control.selected].y() = feet_stream_control.feet_shifts[feet_stream_control.selected].y() + feet_stream_control.step;
    break;
  case 'l':
    feet_stream_control.feet_shifts[feet_stream_control.selected].y() = feet_stream_control.feet_shifts[feet_stream_control.selected].y() - feet_stream_control.step;
    break;
  case 'u':
    feet_stream_control.feet_shifts[feet_stream_control.selected].z() = feet_stream_control.feet_shifts[feet_stream_control.selected].z() + feet_stream_control.step;
    break;
  case 'o':
    feet_stream_control.feet_shifts[feet_stream_control.selected].z() = feet_stream_control.feet_shifts[feet_stream_control.selected].z() - feet_stream_control.step;
    break;
  default:
    break;
  }
}