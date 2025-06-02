
#include "config.h"
#include <Kinematics.hpp>
#include "SpotServo.hpp"
#include <Utilities.hpp>
#include "Spot.hpp"
#include <cstring>
#include "Arduino.h"
#include "SerialHandler.hpp"

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

#define ENABLE_DEBUG /* <-- Commenting this line will remove any trace of debug printing */
//#define DEBUG_SERIAL_PORT Serial1
#include <MacroDebugger.h>

// ----------------- ADAPT VARIABLES ------------------

#include <ArduinoEigen.h>
#include "Kinematics.hpp"
#include "SpotModel.hpp"
#include "LieAlgebra.hpp"

#define PRINT false
#define COMMAND_BUFFER_SIZE 256

double shoulder_length = 0.045;
double elbow_length = 0.08;
double wrist_length = 0.103;
double hip_x = 0.185;
double hip_y = 0.077;
double foot_x = 0.185;
double foot_y = 0.17;
double height = 0.145;

bool log_toggle = false;

int cycle = 0;
float speedShoulder, speedElbow, speedWrist, speedcalc, positionShoulder, positionElbow, positionWrist, loadShoulder, loadElbow, loadWrist = 0;

char inputBuffer[COMMAND_BUFFER_SIZE] = {0};
int bufferPos = 0;

//TODO fix GetPoseEstimate()


enum CONTROL_STATES
{
  IDLE,                // 0
  SET_SERVO_FRAME,     // 1
  SET_OFFSET_LEAN,     // 2
  SET_STANCE_STRAIGHT, // 3
  SET_STANCE_PRONE,    // 4
  TOGGLE_LOG,          // 5
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

typedef struct FEET_STREAM_CONTROL
{
  float step;
  int selected;
  Eigen::Vector3d feet_shifts[NUM_LEGS];
} FEET_STREAM_CONTROL;

SERVO_FRAME servo_frame;
CONTROL_STATES control_state = IDLE;
SERVOS_CONTROL_IDX servo_control_idx = FL_SHOULDER_CONTROL;
OFFSET_LEAN_FRAME offset_lean_frame = {{0}, {0}};
PI_COMMAND pi_command = {false, 0, {0}, 0};
FEET_STREAM_CONTROL feet_stream_control = {0.01, 0, {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)}};

// helper, TODO remove
const std::array<std::string, 4> leg_order = {"FL", "FR", "BL", "BR"};

bool new_command = false;
bool stream_control = false;

SerialHandler serialHandler = SerialHandler(false, '>');
Spot miniSpot = Spot();

SpotModel model = SpotModel();
int iterator = 0;
Eigen::Vector3d orn(0, 0, 0); // roll, pitch, yaw
Eigen::Vector3d pos(0, 0, 0); // body position

//auto joint_angles = model.IK(orn, pos, model.WorldToFoot); //TODO finish this

double joint_angles[4][3];
// model.IK(joint_angles, orn, pos, model.WorldToFoot);

#define SPEED 125
double speeds[3] = {0.0, 0.0, 0.0};
double dt_movement = 0.0;

bool SERIAL_FORWARDING = false;
// ----------------------------------------------------

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED 23
#define NUMPIXELS 10

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

int Speed_Norm = 1500;
bool ini = true;

Kinematics IK;
Utilities util;
double Load = 0;

void send_message(const char *msg)
{
  String msg_str = "ESP/MSG:";
  msg_str = msg_str + String(msg);
  Serial.write(msg);
}

void confirm_servos(){ //TODO continue here, have to run this, IDS seems wrong, but we can control the servos normally
  for(int i = 0; i < 12; i++){
    Serial.print(" Leg: ");
    Serial.print(miniSpot.Servo_List[i].Get_servo_ID());

    //miniSpot.Servo_List[i].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
    double pos = miniSpot.Servo_List[i].GetPoseEstimate();
    Serial.print(" Ang: ");
    // Serial.print(positionShoulder);
    Serial.print(pos);
    
    //Serial.println(miniSpot.Servo_List[i].GetPoseEstimate());
  }
  Serial.println();
}

void setup()
{
  DEBUG_BEGIN();
  Serial.begin(115200);
  Serial.println("ESP/LOG:Starting setup!");

  InitRGB();
  RGBcolor(0, 64, 255);
  servoInit();
  
  boardDevInit();// TODO might cause a crash

  pinMode(LED_PIN, OUTPUT);
  RGBoff(); // TODO probably will be removed
  delay(100);

  pingAll(true); //TODO move the routines to init servos to Spot.cpp

  // Complete_Spot.Initialize(Servos,12);

  delay(5000);
  IK.Initialize(0.04, 0.07, 0.11);
  model.IK(joint_angles, orn, pos, model.WorldToFoot);
  

  //miniSpot.Init_Servos();//TODO About the crash: try to see if where the class that deals with waveshare bus  is instantiated
  //servoInit();
  miniSpot.Init();


  miniSpot.prone_calibration_stance();
  Serial.println("ESP/LOG:Completed setup!");
  delay(5000);
  confirm_servos();
  ini = false;
}

void loop()
{

  cycle++;
  if (cycle == 4)
    cycle = 0;

  // Serial.print("ESP/LOOP/CYCLE:");
  // Serial.println(cycle);

  if (pi_command.new_command){    
    pi_command.new_command = false;

    switch (pi_command.command)
    {
    case SET_STANCE_STRAIGHT:
      Serial.println("SET_STANCE_STRAIGHT");
      miniSpot.straight_calibration_stance();
      break;
    case SET_STANCE_PRONE:
      Serial.println("SET_STANCE_PRONE");
      miniSpot.prone_calibration_stance();
      break;

    case SET_SERVO_FRAME:
      if (serialHandler.parseServoFrame(pi_command.package + 2, servo_frame) < 0)
        break;
      for (int i = 0; i < 12; i++)
      {
        miniSpot.Servo_List[i].SetGoal(servo_frame.pos[i], servo_frame.speed[i]);
      }
      miniSpot.Update_Spot(0);
      break;

    case SET_OFFSET_LEAN:
      if (serialHandler.parseOffsetLean(pi_command.package + 2, offset_lean_frame) < 0)
        // if(parseServoFrame(pi_command.package+2)<0)
        break;
      // Serial.println("Offset Lean:");
      orn.x() = offset_lean_frame.rpy[0];
      orn.y() = offset_lean_frame.rpy[1];
      orn.z() = offset_lean_frame.rpy[2];

      pos.x() = offset_lean_frame.xyz[0];
      pos.y() = offset_lean_frame.xyz[1];
      pos.z() = offset_lean_frame.xyz[2];

      // joint_angles = model.IK(orn, pos, model.WorldToFoot); // TODO fix this

      // // Serial.println("Joint Angles in Degrees:");
      // iterator = 0;
      // for (const auto &leg : joint_angles){
      //   for (double angle_rad : leg){
      //     miniSpot.Servo_List[iterator].SetGoal(angle_rad * 180 / M_PI, offset_lean_frame.speed);
      //     iterator += 1;
      //   }
      // }
      
      // miniSpot.Update_Spot(0);
      break;

    case TOGGLE_LOG:
      if (log_toggle){
        Serial.println("ESP/LOG:Log:true");
      }
      log_toggle = !log_toggle;
      break;

    default:
      break;
    }
  }

  if (stream_control){
    stream_control = false;
    //joint_angles = model.IKWithFootOverrides(orn, pos, feet_stream_control.feet_shifts, leg_order);
    model.IKFootOverrides(joint_angles, orn, pos, feet_stream_control.feet_shifts);

    iterator = 0;

    for(int i = 0; i < NUM_LEGS; i++){
      dt_movement = miniSpot.Leg_Joint_Speeds(speeds, joint_angles[i], iterator/3, 200);
      for(int j = 0; j < NUM_JOINTS; j++){
        DEBUG_I("JOINT: %d  speed: %.2f  Angle: %.2f", iterator % 3, speeds[iterator % 3], joint_angles[i][j]);
        miniSpot.Servo_List[iterator].SetGoal(joint_angles[i][j] * 180 / M_PI, speeds[iterator%3]);
        iterator += 1;
      }
    }

    // for (const auto &leg : joint_angles)
    // {
    //   // Serial.print("iterator: ");
    //   // Serial.print(iterator);
    //   dt_movement = miniSpot.Leg_Joint_Speeds(speeds, joint_angles[iterator/3].data(), iterator/3, 400);
    //   for (double angle_rad : leg)
    //   {
    //     DEBUG_I("JOINT: %d  speed: %.2f", iterator % 3, speeds[iterator % 3]);
    //     miniSpot.Servo_List[iterator].SetGoal(angle_rad * 180 / M_PI, speeds[iterator%3]);
    //     iterator += 1;
    //   }
    // }
    miniSpot.Update_Spot(0);
    switch(feet_stream_control.selected){
      case 0:
        miniSpot.Servo_List[0].GetPoseEstimate();
        miniSpot.Servo_List[1].GetPoseEstimate();
        miniSpot.Servo_List[2].GetPoseEstimate();
        break;
      case 1:
        miniSpot.Servo_List[3].GetPoseEstimate();
        miniSpot.Servo_List[4].GetPoseEstimate();
        miniSpot.Servo_List[5].GetPoseEstimate();
        break;
      case 2:
        miniSpot.Servo_List[6].GetPoseEstimate();
        miniSpot.Servo_List[7].GetPoseEstimate();
        miniSpot.Servo_List[8].GetPoseEstimate();
        break;
      case 3:
        miniSpot.Servo_List[9].GetPoseEstimate();
        miniSpot.Servo_List[10].GetPoseEstimate();
        miniSpot.Servo_List[11].GetPoseEstimate();
        break;
    }
  }
}

void serialEvent()
{
  serialHandler.HandleSerialEvent(inputBuffer, bufferPos, process_stream_control, pi_command, offset_lean_frame, servo_frame);
}

bool process_stream_control(char c)
{
  if (!stream_control)
  {
    stream_control = true;
  }
  switch (c)
  {
  case 'q':
    stream_control = false;
    Serial.println("ESP/STREAM:Stream Control Disabled");
    return false;
    break;
  case 'm':
    if (feet_stream_control.step > 0.01) {
      feet_stream_control.step = feet_stream_control.step + 0.01;
    } else if (feet_stream_control.step <= 0.01) {
      feet_stream_control.step = feet_stream_control.step + 0.001;
    }
    break;
  case 'n':
    if (feet_stream_control.step > 0.01) {
      feet_stream_control.step = feet_stream_control.step - 0.01;
    } else if (feet_stream_control.step <= 0.01) {
      feet_stream_control.step = feet_stream_control.step - 0.001;
    }
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
  case 'p':
    confirm_servos();
    break;
  default:
    break;
  }
  return true;
}