
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

const int nCyclePoints = 19;
float converted_xyz[NUM_LEGS][nCyclePoints][3];

double shoulder_list_F[4][nCyclePoints] = {{8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,8},{-8,-9,-9,-9,-9,-9,-9,-9,-9,-8,-8,-8,-8,-8,-8,-8,-9,-9,-9},{8,9,9,9,9,9,9,9,9,8,8,8,8,8,8,8,9,9,9},{-8,-8,-8,-8,-8,-8,-8,-8,-9,-9,-9,-9,-9,-9,-9,-9,-9,-9,-8}};
double elbow_list_F[4][nCyclePoints] = {{-66,-43,-54,-64,-73,-79,-83,-86,-87,-87,-85,-77,-68,-57,-46,-39,-37,-42,-43},{66,87,84,78,69,58,48,39,36,41,52,66,74,79,83,86,88,87,87},{-66,-87,-84,-78,-69,-58,-48,-39,-36,-41,-52,-66,-74,-79,-83,-86,-88,-87,-87},{66,43,54,64,73,79,83,86,87,87,85,77,68,57,46,39,37,42,43}};
double wrist_list_F[4][nCyclePoints] = {{101,95,100,101,100,97,94,94,96,100,105,109,109,107,102,97,94,95,95},{-101,-101,-106,-109,-109,-107,-103,-98,-94,-95,-99,-101,-100,-96,-94,-94,-96,-101,-101},{101,101,106,109,109,107,103,98,94,95,99,101,100,96,94,94,96,101,101},{-101,-95,-100,-101,-100,-97,-94,-94,-96,-100,-105,-109,-109,-107,-102,-97,-94,-95,-95}};

float gait_forward_FL[nCyclePoints][3] = {{-0.050521, 0.126163, 0.069700}, {0.075665, -0.017720, 0.048540}, {-0.230087, 0.064632, 0.060651}, {-0.232387, 0.063966, 0.060553}, {-0.245148, 0.042991, 0.057469}, {-0.050677, -0.041421, 0.045054}, {-0.272968, -0.024846, 0.047492}, {-0.268281, 0.000149, 0.051168}, {-0.200694, -0.059609, 0.062522}, {0.016523, -0.022197, 0.145235}, {-0.200619, -0.064369, 0.052000}, {-0.115665, -0.045111, 0.094576}, {-0.147996, -0.052101, 0.079121}, {-0.154418, -0.008872, 0.174696}, {-0.218363, -0.057532, 0.067115}, {-0.067338, -0.065651, 0.049166}, {0.003908, -0.016068, 0.158786}, {-0.206602, -0.027334, 0.133878}, {0.075665, -0.017720, 0.048540}};
float gait_forward_FR[nCyclePoints][3] = {{-0.050521, -0.126163, 0.069700}, {0.075278, 0.054912, 0.072907}, {-0.032933, 0.014634, 0.161955}, {-0.175234, 0.068953, 0.041865}, {-0.006571, 0.075060, 0.028364}, {-0.270166, 0.062812, 0.055442}, {-0.050986, 0.059335, 0.063129}, {-0.080980, 0.055561, 0.071473}, {0.089108, 0.078661, 0.020403}, {-0.047635, -0.117609, 0.068442}, {-0.026298, -0.068976, 0.061290}, {-0.050521, -0.126163, 0.069700}, {0.064855, -0.034467, 0.056215}, {-0.029005, -0.053908, 0.059074}, {-0.272968, 0.024846, 0.047492}, {-0.268281, -0.000149, 0.051168}, {0.006572, 0.052728, 0.077735}, {0.075278, 0.054912, 0.072907}, {0.075278, 0.054912, 0.072907}};
float gait_forward_RL[nCyclePoints][3] = {{0.134479, 0.126163, 0.069700}, {0.260278, -0.054912, 0.072907}, {0.152067, -0.014634, 0.161955}, {0.009766, -0.068953, 0.041865}, {0.178429, -0.075060, 0.028364}, {-0.085166, -0.062812, 0.055442}, {0.134014, -0.059335, 0.063129}, {0.104020, -0.055561, 0.071473}, {0.274108, -0.078661, 0.020403}, {0.137365, 0.117609, 0.068442}, {0.158702, 0.068976, 0.061290}, {0.134479, 0.126163, 0.069700}, {0.249855, 0.034467, 0.056215}, {0.155995, 0.053908, 0.059074}, {-0.087968, -0.024846, 0.047492}, {-0.083281, 0.000149, 0.051168}, {0.191572, -0.052728, 0.077735}, {0.260278, -0.054912, 0.072907}, {0.260278, -0.054912, 0.072907}};
float gait_forward_RR[nCyclePoints][3] = {{0.134479, -0.126163, 0.069700}, {0.260665, 0.017720, 0.048540}, {-0.045087, -0.064632, 0.060651}, {-0.047387, -0.063966, 0.060553}, {-0.060148, -0.042991, 0.057469}, {0.134323, 0.041421, 0.045054}, {-0.087968, 0.024846, 0.047492}, {-0.083281, -0.000149, 0.051168}, {-0.015694, 0.059609, 0.062522}, {0.201523, 0.022197, 0.145235}, {-0.015619, 0.064369, 0.052000}, {0.069335, 0.045111, 0.094576}, {0.037004, 0.052101, 0.079121}, {0.030582, 0.008872, 0.174696}, {-0.033363, 0.057532, 0.067115}, {0.117662, 0.065651, 0.049166}, {0.188908, 0.016068, 0.158786}, {-0.021602, 0.027334, 0.133878}, {0.260665, 0.017720, 0.048540}};

void ConvertAnglesToBodyXYZ(const double sh[][nCyclePoints], // TODO #5526
                            const double el[][nCyclePoints],
                            const double wr[][nCyclePoints],
                            SpotModel&     spot,
                            float          out_xyz[NUM_LEGS][nCyclePoints][3]);

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
double cycle_time = 50000; // micros
double current_time = 0;
double last_time = 0;
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

  SET_POSE_WALK,       // 6
  SET_POSE_PRONE,      // 7
  TEST_GAIT            // 8
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
  Eigen::Vector3d body_orientation;
  Eigen::Vector3d body_translation;
} FEET_STREAM_CONTROL;

SERVO_FRAME servo_frame;
CONTROL_STATES control_state = IDLE;
SERVOS_CONTROL_IDX servo_control_idx = FL_SHOULDER_CONTROL;
OFFSET_LEAN_FRAME offset_lean_frame = {{0}, {0}};
PI_COMMAND pi_command = {false, 0, {0}, 0};
FEET_STREAM_CONTROL feet_stream_control = {0.01, 0, {Eigen::Vector3d(0.0925, 0.085, -0.145), Eigen::Vector3d(0.0925, -0.085, -0.145), Eigen::Vector3d(-0.0925, 0.085, -0.145), Eigen::Vector3d(-0.0925, -0.085, -0.145)}, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};

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

//double joint_angles[4][3];
// model.IK(joint_angles, orn, pos, model.WorldToFoot);

#define SPEED 125
double speeds[3] = {0.0, 0.0, 0.0};
double dt_movement = 0.0;

bool SERIAL_FORWARDING = false;

int move_foot_cmd = 0; // TODO remove later, only used for testing #2254
bool printLoads = false; // TODO remoce later. #1137

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

//Kinematics IK;
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

  delay(2000);
  //IK.Initialize(0.04, 0.07, 0.11);
  //model.IK(miniSpot.joint_angles, orn, pos);//, model.WorldToFoot);
  

  //miniSpot.Init_Servos();//TODO About the crash: try to see if where the class that deals with waveshare bus  is instantiated
  //servoInit();
  miniSpot.Init();


  //miniSpot.prone_calibration_stance();
  Serial.println("ESP/LOG:Completed setup!");
  delay(2000);
  confirm_servos();
  ini = false;

  ConvertAnglesToBodyXYZ(shoulder_list_F, elbow_list_F, wrist_list_F, miniSpot.model, converted_xyz);
}

void loop()
{

  cycle++;
  if (cycle == 4)
    cycle = 0;

  // Serial.print("ESP/LOOP/CYCLE:");
  // Serial.println(cycle);

  current_time = micros();
  if(current_time - last_time > cycle_time){
    last_time = current_time;
    
    if (pi_command.new_command){    
      pi_command.new_command = false;
      DEBUG_I("Command: %d", pi_command.command);
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

      case SET_POSE_WALK:
        miniSpot.pose(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
        break;
      
      case SET_POSE_PRONE:
        miniSpot.pose(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0.02));
        break;
      
      case TEST_GAIT:
        DEBUG_I("TEST_GAIT");
        miniSpot.perform_gait_singular(1, nCyclePoints, gait_forward_FR, 900000.0);
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

    if (move_foot_cmd == 1){ // TODO remove this later #2245
      miniSpot.move_foot(feet_stream_control.selected, Eigen::Vector3d(0.0925, 0.085, -0.115));
      move_foot_cmd = 0;
      stream_control = false;
    } else if (move_foot_cmd == 2) { // TODO remove this later #2245
      miniSpot.move_foot(feet_stream_control.selected, Eigen::Vector3d(0.0925, 0.085, -0.145));
      move_foot_cmd = 0;
      stream_control = false;
    } else if (stream_control){
      stream_control = false;

      //miniSpot.move_feet(feet_stream_control.feet_shifts);

      switch(feet_stream_control.selected){
        case 0:
          miniSpot.move_foot(0, feet_stream_control.feet_shifts[0]);
          miniSpot.Servo_List[0].GetPoseEstimate();
          miniSpot.Servo_List[1].GetPoseEstimate();
          miniSpot.Servo_List[2].GetPoseEstimate();
          break;
        case 1:
          miniSpot.move_foot(1, feet_stream_control.feet_shifts[1]);
          miniSpot.Servo_List[3].GetPoseEstimate();
          miniSpot.Servo_List[4].GetPoseEstimate();
          miniSpot.Servo_List[5].GetPoseEstimate();
          break;
        case 2:
          miniSpot.move_foot(2, feet_stream_control.feet_shifts[2]);
          miniSpot.Servo_List[6].GetPoseEstimate();
          miniSpot.Servo_List[7].GetPoseEstimate();
          miniSpot.Servo_List[8].GetPoseEstimate();
          break;
        case 3:
          miniSpot.move_foot(3, feet_stream_control.feet_shifts[3]);
          miniSpot.Servo_List[9].GetPoseEstimate();
          miniSpot.Servo_List[10].GetPoseEstimate();
          miniSpot.Servo_List[11].GetPoseEstimate();
          break;
        case 4:
          miniSpot.rotate(feet_stream_control.body_orientation.x(), feet_stream_control.body_orientation.y(), feet_stream_control.body_orientation.y());
          break;
      }
      if (feet_stream_control.selected <4) printFootPos(feet_stream_control.selected);
      miniSpot.getLoads();
    }

    if(printLoads){ // TODO remove this later #1137
      switch(feet_stream_control.selected){
        case 0:
          DEBUG_I("S_LOAD: %f, E_LOAD: %f, W_LOAD: %f", 
          miniSpot.Servo_List[0].getLoad(),
          miniSpot.Servo_List[1].getLoad(),
          miniSpot.Servo_List[2].getLoad());
          break;
        case 1:
          DEBUG_I("S_LOAD: %f, E_LOAD: %f, W_LOAD: %f", 
          miniSpot.Servo_List[3].getLoad(),
          miniSpot.Servo_List[4].getLoad(),
          miniSpot.Servo_List[5].getLoad());
          break;
        case 2:
          DEBUG_I("S_LOAD: %f, E_LOAD: %f, W_LOAD: %f", 
          miniSpot.Servo_List[6].getLoad(),
          miniSpot.Servo_List[7].getLoad(),
          miniSpot.Servo_List[8].getLoad());
          break;
        case 3:
          DEBUG_I("S_LOAD: %f, E_LOAD: %f, W_LOAD: %f", 
          miniSpot.Servo_List[9].getLoad(),
          miniSpot.Servo_List[10].getLoad(),
          miniSpot.Servo_List[11].getLoad());
          break;

      }
    }
  }
}

void printFootPos(int leg){
  Eigen::Vector3d footPosition;
  miniSpot.getFootPosition(leg, footPosition);
  DEBUG_I("Foot Position: %f, %f, %f", footPosition[0], footPosition[1], footPosition[2]);
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
  case '5':
    feet_stream_control.selected = 4;
    break;
  case 'i':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].x() = feet_stream_control.feet_shifts[feet_stream_control.selected].x() + feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.x() += feet_stream_control.step;
    break;
  case 'k':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].x() = feet_stream_control.feet_shifts[feet_stream_control.selected].x() - feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.x() -= feet_stream_control.step;
    break;
  case 'j':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].y() = feet_stream_control.feet_shifts[feet_stream_control.selected].y() + feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.y() += feet_stream_control.step;
    break;
  case 'l':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].y() = feet_stream_control.feet_shifts[feet_stream_control.selected].y() - feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.y() -= feet_stream_control.step;
    break;
  case 'u':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].z() = feet_stream_control.feet_shifts[feet_stream_control.selected].z() + feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.z() += feet_stream_control.step;
    break;
  case 'o':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].z() = feet_stream_control.feet_shifts[feet_stream_control.selected].z() - feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.z() -= feet_stream_control.step;
    break;
  case 'p':
    confirm_servos();
    break;
  case 'h':
    move_foot_cmd = 1; // TODO remove later [tag #2254] (use ctrl+f to find parent)
    break;
  case 'y':
    move_foot_cmd = 2; // TODO remove later [tag #2254] (use ctrl+f to find parent)
    break;
  default:
    break;
  }
  return true;
}

void ConvertAnglesToBodyXYZ(const double sh[][nCyclePoints], //TODO Temporary function, delete it after conversions are made #5526
                            const double el[][nCyclePoints],
                            const double wr[][nCyclePoints],
                            SpotModel&     spot,
                            float          out_xyz[NUM_LEGS][nCyclePoints][3])
{
  for (int leg = 0; leg < NUM_LEGS; ++leg)
  {

    LegQuadrant side = (leg == 0 || leg == 2) ? Right : Left; // FR, RR

    for (int f = 0; f < nCyclePoints; ++f)
    {
      double a[3] = {sh[leg][f], el[leg][f], wr[leg][f]};

      // A: FK in hip frame
      Eigen::Vector3d pHip;
      spot.FK_singular(a, side, pHip);

      // B: to body frame (zero body RPY & pos assumed here)
      Eigen::Vector3d pBody;
      spot.HipToBodyV(leg, pHip, pBody); // <- your transform

      // store as float
      out_xyz[leg][f][0] = static_cast<float>(pBody.x());
      out_xyz[leg][f][1] = static_cast<float>(pBody.y());
      out_xyz[leg][f][2] = static_cast<float>(pBody.z());
      DEBUG_I("Leg: %d, Frame: %d, X: %f, Y: %f, Z: %f", leg, f, out_xyz[leg][f][0], out_xyz[leg][f][1], out_xyz[leg][f][2]);
    }
  }
}