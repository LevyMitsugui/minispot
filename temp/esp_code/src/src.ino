
#include "config.h"
#include <Kinematics.hpp>
#include "SpotServo.hpp"
#include <Utilities.hpp>
#include <cstring>
#include "Arduino.h"
#include "SerialHandler.hpp"
#include "gait_routines.hpp"

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
#include "Spot.hpp"
#include "Kinematics.hpp"
#include "SpotModel.hpp"
#include "LieAlgebra.hpp"

#define PRINT false
#define COMMAND_BUFFER_SIZE 256

#define STD_SPEED_RAD 0.8 // rad/s

const int nCyclePoints = 19;
float converted_xyz[NUM_LEGS][nCyclePoints][3];

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
double cycle_time = 22; // millis
double current_time = 0;
double last_time = 0;
float speedShoulder, speedElbow, speedWrist, speedcalc, positionShoulder, positionElbow, positionWrist, loadShoulder, loadElbow, loadWrist = 0;

char inputBuffer[COMMAND_BUFFER_SIZE] = {0};
int bufferPos = 0;

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
  TEST_GAIT,           // 8
  GAIT_FORWARD,        // 9
  GAIT_BACKWARD,       // 10
  GAIT_RIGHT,          // 11
  GAIT_LEFT,           // 12
  GAIT_ROTATE_RIGHT,   // 13
  GAIT_ROTATE_LEFT,    // 14 
  NO_BLOCKING_TEST,    // 15
  STANCE_TEST,         // 16
  PERFORM_STEP,        // 17
  PERFORM_DYNAMIC_GAIT,// 18
  PRINT_PATH_POINTS,   // 19
  PRINT_PATH_POINTS1,   // 20
  PRINT_PATH_POINTS2,   // 21
  PRINT_PATH_POINTS3,   // 22
  PERFORM_DYNAMIC_GAIT2,// 23
  CONTROL_HOLONOMIC     // 24
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

enum HOLONOMIC_STATES
{
  OFF,
  ACKNOWLEDGE,
  WALK,
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
FEET_STREAM_CONTROL feet_stream_control = {0.01, 0, {Eigen::Vector3d(0.0925, 0.085, -0.135), Eigen::Vector3d(0.0925, -0.085, -0.135), Eigen::Vector3d(-0.0925, 0.085, -0.135), Eigen::Vector3d(-0.0925, -0.085, -0.135)}, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};

// helper, TODO remove
const std::array<std::string, 4> leg_order = {"FL", "FR", "BL", "BR"};

bool new_command = false;
bool stream_control = false;

SerialHandler serialHandler = SerialHandler(false, '>');
Spot miniSpot = Spot();

//SpotModel model = SpotModel();
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

bool printLoads = false; // TODO remove later. #1137

bool gait_no_blocking_test = false;
bool test_time = false;

bool test_gait = false;
bool stance_test = false;
// double t_swing = 0.0;
// Eigen::Vector3d swingPos;
// Eigen::Vector3d bezierPoints[4] = 
//     {Eigen::Vector3d(0.0,0.0,0.0),
//     Eigen::Vector3d(0.0,0.0,0.06),
//     Eigen::Vector3d(0.08,0.0,0.06),
//     Eigen::Vector3d(0.05,0.0,0.0)};

// Eigen::Vector3d startingPoint;
// double bezCurrTime = millis();
// double bezPrevTime = bezCurrTime;
// double bezPeriodTime = 0;
// Eigen::Vector3d points_bf [4];

#define D_STANCE 1.5 / 100.0
#define T_GAIT 0.7

double V  = 0.0;
double Vn = 0.0;
double W  = 0.0;
double Vt  = 0.0;

double Rx  = 0.0;
double Ry  = 0.0;
double Rth = 0.0;

double Vx = 0.0;
double Vy = 0.0;
const double Vz = 0.0;

Eigen::Vector3d velocities[4] = {Eigen::Vector3d(D_STANCE*2/T_GAIT,0.0,0.0),
                                 Eigen::Vector3d(D_STANCE*2/T_GAIT,0.0,0.0),
                                 Eigen::Vector3d(D_STANCE*2/T_GAIT,0.0,0.0),
                                 Eigen::Vector3d(D_STANCE*2/T_GAIT,0.0,0.0)};

// Eigen::Vector3d velocities[4] = {Eigen::Vector3d(0.0,D_STANCE*2/T_GAIT,0.0),
//                                  Eigen::Vector3d(0.0,D_STANCE*2/T_GAIT,0.0),
//                                  Eigen::Vector3d(0.0,D_STANCE*2/T_GAIT,0.0),
//                                  Eigen::Vector3d(0.0,D_STANCE*2/T_GAIT,0.0)};

Eigen::Vector3d velocities0[4] = {Eigen::Vector3d(0.0,0.0,0.0),
                                  Eigen::Vector3d(0.0,0.0,0.0),
                                  Eigen::Vector3d(0.0,0.0,0.0),
                                  Eigen::Vector3d(0.0,0.0,0.0)};

fsm_t holonomic_sm;
void holonomicControlStateMachine(fsm_t &stateMachine, double currTime);

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

void confirm_servos(){ 
  for(int i = 0; i < 12; i++){
    Serial.print(" Leg: ");
    Serial.print(miniSpot.Servo_List[i].Get_servo_ID());

    //miniSpot.Servo_List[i].Get_Feedback(speedShoulder, loadShoulder, positionShoulder);
    double pos = miniSpot.Servo_List[i].GetPoseEstimate();// TODO rebase this from degrees to radians
    Serial.print(" Ang: ");
    // Serial.prin(positionShoulder);
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
  
  boardDevInit();

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
}

void loop()
{

  cycle++;
  if (cycle == 4){
    cycle = 0;
  }
  // Serial.print("ESP/LOOP/CYCLE:");
  // Serial.println(cycle);

  if (test_gait){
    test_gait = miniSpot.testGaitGen();
  }

  if (stance_test){
    stance_test = miniSpot.performStancePhase();
  }

  current_time = millis();
  if(current_time - last_time > cycle_time){
    last_time = current_time;
    // Serial.print("ESP/LOOP/CYCLE:");
    // Serial.println(cycle);

    if (gait_no_blocking_test){
      miniSpot.perform_gait_no_blocking(gait_backward, false);
    }

    if (test_time){
      if ((millis() - miniSpot.timeHelper[0])> miniSpot.timeHelper[1]){
        DEBUG_I("TIME IS DONE: %f", miniSpot.timeHelper[0]);
        test_time = false;
      }
    }

    switch (control_state){
      case PERFORM_STEP:
        miniSpot.performStep(0, current_time);
        miniSpot.performStep(3, current_time);
      break;

      case PERFORM_DYNAMIC_GAIT:
        miniSpot.performDynamicGait(current_time, velocities);
      break;

      case PERFORM_DYNAMIC_GAIT2:
        miniSpot.computeFeetVelocities(velocities, V, Vn, W);
        miniSpot.performDynamicGait2(current_time, velocities);
      break;

      case CONTROL_HOLONOMIC:
        holonomicControlStateMachine(holonomic_sm, current_time);
      break;

      default:
      break;
    }

    if (control_state == PERFORM_STEP && miniSpot.isStepDone(0, 3)){
      control_state = IDLE;
    }

    miniSpot.Update_Spot(0);

    if (pi_command.new_command){ 

      pi_command.new_command = false;
      DEBUG_I("Command: %d", pi_command.command);
      
      switch (pi_command.command)
      {
      case IDLE:
        control_state = IDLE;
        break;
      
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
        miniSpot.set_lean(0, 0, 0);
        miniSpot.set_rpy(0, 0, 0);
        miniSpot.pose();
        break;
      
      case SET_POSE_PRONE:
        miniSpot.set_lean(0, 0, 0.02);
        miniSpot.set_rpy(0, 0, 0);
        miniSpot.pose();
        break;
      
      case TEST_GAIT:
        DEBUG_I("TEST_GAIT_GEN");
        //miniSpot.perform_gait_singular(3, nCyclePoints, gait_forward_RR, 125000.0);
        test_gait = true;
        break;
      
      case GAIT_FORWARD:
        DEBUG_I("GAIT_FORWARD");
        miniSpot.perform_gait( nCyclePoints, gait_forward, 190000.0, 5);
        break;

      case GAIT_BACKWARD:
        DEBUG_I("GAIT_BACKWARD");
        //miniSpot.perform_gait( nCyclePoints, gait_backward, 120000.0, 5);
        miniSpot.perform_gait( nCyclePoints, gait_backward, 150000.0, 5);
        break;

      case GAIT_RIGHT:
        DEBUG_I("GAIT_RIGHT");
        miniSpot.perform_gait( nCyclePoints, gait_right, 120000.0, 5);
        break;
      
      case GAIT_LEFT:
        DEBUG_I("GAIT_LEFT");
        miniSpot.perform_gait( nCyclePoints, gait_left, 120000.0, 5);
        break;

      case GAIT_ROTATE_RIGHT:
        DEBUG_I("GAIT_ROTATE_RIGHT");
        miniSpot.perform_gait( nCyclePoints, gait_rotate_right, 190000.0, 5);
        break;
      
      case GAIT_ROTATE_LEFT:
        DEBUG_I("GAIT_ROTATE_LEFT");
        miniSpot.perform_gait( nCyclePoints, gait_rotate_left, 190000.0, 5);
        break;
      
      case NO_BLOCKING_TEST:
        DEBUG_I("NO_BLOCKING_TEST");
        gait_no_blocking_test = !gait_no_blocking_test;
        miniSpot.perform_gait_no_blocking(gait_backward, true);
        break;
      
      case STANCE_TEST:
        DEBUG_I("STANCE_TEST");
        stance_test = true;
        break;

      case PERFORM_STEP:
        DEBUG_I("PERFORM_STEP");
        //miniSpot.setPeriods(0.4, 0.2, 0.2);
        miniSpot.setStanceVelocity(0, Eigen::Vector3d(-0.15, 0.0, 0));
        miniSpot.setStanceVelocity(3, Eigen::Vector3d(-0.15, 0.0, 0));
        control_state = PERFORM_STEP;
        break;

      case PERFORM_DYNAMIC_GAIT:
        DEBUG_I("PERFORM_DYNAMIC_GAIT");
        // miniSpot.setPeriods(0.4, 0.2, 0.2);
        // miniSpot.setPeriods(0.8, 0.4, 0.4);
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
        if (control_state == PERFORM_DYNAMIC_GAIT){
          miniSpot.setGaitState(0, current_time);
          miniSpot.performDynamicGait(current_time, velocities0);
        }
        control_state = (control_state == PERFORM_DYNAMIC_GAIT) ? IDLE : PERFORM_DYNAMIC_GAIT;
        break;

      case PRINT_PATH_POINTS:
        DEBUG_I("PRINT_PATH_POINTS");
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
        miniSpot.setStanceVelocity(RL, Eigen::Vector3d(-0.15, 0.0, 0.0));
        miniSpot.printPathPoints(RL);
        break;
      
      case PRINT_PATH_POINTS1:
        DEBUG_I("PRINT_PATH_POINTS2");
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
        miniSpot.setStanceVelocity(RL, Eigen::Vector3d(0.15, 0.15, 0.0));
        miniSpot.printPathPoints(RL);
        break;

      case PRINT_PATH_POINTS2:
        DEBUG_I("PRINT_PATH_POINTS2");
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
        miniSpot.setStanceVelocity(RL, Eigen::Vector3d(-0.30, 0.0, 0.0));
        miniSpot.printPathPoints(RL);
        break;
      
      case PRINT_PATH_POINTS3:
        DEBUG_I("PRINT_PATH_POINTS3");
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
        miniSpot.setStanceVelocity(RR, velocities[RR]);
        miniSpot.printPathPoints(RR);
        break;

      case PERFORM_DYNAMIC_GAIT2:
        DEBUG_I("PERFORM_DYNAMIC_GAIT2");
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
        if (control_state == PERFORM_DYNAMIC_GAIT2){
          miniSpot.setGaitState(0, current_time);
          miniSpot.performDynamicGait2(current_time, velocities0);
        }
        control_state = (control_state == PERFORM_DYNAMIC_GAIT2) ? IDLE : PERFORM_DYNAMIC_GAIT2;
        break;

      case CONTROL_HOLONOMIC:
        if (holonomic_sm.state == OFF){
          holonomic_sm.new_state = ACKNOWLEDGE;
        } else if (holonomic_sm.state == WALK){
          holonomic_sm.new_state == OFF;
        }
      break;

      case 25:
        parseSpeeds(pi_command, V, Vn, W);
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

      //miniSpot.move_feet(feet_stream_control.feet_shifts);

      switch(feet_stream_control.selected){
        case 0:
          miniSpot.move_foot(0, feet_stream_control.feet_shifts[0], STD_SPEED_RAD);
          miniSpot.Servo_List[0].GetPoseEstimate();
          miniSpot.Servo_List[1].GetPoseEstimate();
          miniSpot.Servo_List[2].GetPoseEstimate();
          break;
        case 1:
          miniSpot.move_foot(1, feet_stream_control.feet_shifts[1], STD_SPEED_RAD);
          miniSpot.Servo_List[3].GetPoseEstimate();
          miniSpot.Servo_List[4].GetPoseEstimate();
          miniSpot.Servo_List[5].GetPoseEstimate();
          break;
        case 2:
          miniSpot.move_foot(2, feet_stream_control.feet_shifts[2], STD_SPEED_RAD);
          miniSpot.Servo_List[6].GetPoseEstimate();
          miniSpot.Servo_List[7].GetPoseEstimate();
          miniSpot.Servo_List[8].GetPoseEstimate();
          break;
        case 3:
          miniSpot.move_foot(3, feet_stream_control.feet_shifts[3], STD_SPEED_RAD);
          miniSpot.Servo_List[9].GetPoseEstimate();
          miniSpot.Servo_List[10].GetPoseEstimate();
          miniSpot.Servo_List[11].GetPoseEstimate();
          break;
        case 4:
          //miniSpot.rotate(feet_stream_control.body_orientation.x(), feet_stream_control.body_orientation.y(), feet_stream_control.body_orientation.z());
          miniSpot.set_rpy(feet_stream_control.body_orientation.x(), feet_stream_control.body_orientation.y(), feet_stream_control.body_orientation.z());
          miniSpot.pose();
          break;
        
        case 5:
          //miniSpot.translate(feet_stream_control.body_translation.x(), feet_stream_control.body_translation.y(), feet_stream_control.body_translation.z());
          miniSpot.set_lean(feet_stream_control.body_translation.x(), feet_stream_control.body_translation.y(), feet_stream_control.body_translation.z());
          miniSpot.pose();
          break;
      }
      //if (feet_stream_control.selected <4) printFootPos(feet_stream_control.selected);
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
  Eigen::Vector3d footPosition = miniSpot.getFootPosition(leg);
  DEBUG_I("Foot Position: %f, %f, %f", footPosition[0], footPosition[1], footPosition[2]);
  footPosition = miniSpot.getEstimatedFootPosition(leg);
  DEBUG_I("Foot Position: %f, %f, %f - REAL", footPosition[0], footPosition[1], footPosition[2]);
}

void serialEvent()
{
  //serialHandler.HandleSerialEvent(inputBuffer, bufferPos, process_stream_control, pi_command, offset_lean_frame, servo_frame);
  serialHandler.HandleSerialEvent(inputBuffer, bufferPos, control_speeds, pi_command, offset_lean_frame, servo_frame);
}

void parseSpeeds(PI_COMMAND pi_command, double &V, double &Vn, double&W){
  char *token = strtok(pi_command.package + 3, ",");

  V = atof(token);
  token = strtok(NULL, ",");
  Vn = atof(token);
  token = strtok(NULL, ",");
  W = atof(token);
  DEBUG_I("V: %f, Vn: %f, W: %f", V, Vn, W);
}

void holonomicControlStateMachine(fsm_t &stateMachine, double currTime){

  stateMachine.tis = currTime - stateMachine.tes;

  switch(stateMachine.state){
    case OFF:
    break;
    case ACKNOWLEDGE:
      send_message("True");
    case WALK:
      miniSpot.computeFeetVelocities(velocities, V, Vn, W);
      miniSpot.performDynamicGait2(current_time, velocities);
    break;
  }

  if (stateMachine.state == ACKNOWLEDGE && stateMachine.tis >= 500){
    stateMachine.new_state = WALK;
  }

  if (stateMachine.state != stateMachine.new_state){
    stateMachine.state = stateMachine.new_state;
    stateMachine.tes = currTime;
    stateMachine.tis = 0.0;
    stateMachine.stateChanged = true;
  } else {
    stateMachine.stateChanged = false;
  }
}

bool control_speeds(char c) {
  if (!stream_control)
  {
    stream_control = true;
  }
  switch (c){
    case 'q':
      stream_control = false;
      Serial.println("ESP/STREAM:Stream Control Disabled");
      return false;
    break;

    case '1':
      if (control_state == PERFORM_DYNAMIC_GAIT2){
        control_state = IDLE;
        miniSpot.setGaitState(IDLE_GAIT, current_time);
      } else {
        control_state = PERFORM_DYNAMIC_GAIT2;
        miniSpot.setPeriods(T_GAIT, T_GAIT/2, T_GAIT/2);
      }
    break;

    case '2':
    case 's':
      V  = 0.0;
      Vn = 0.0;
      W  = 0.0;
    break;

    case 'i':
      V += feet_stream_control.step;
    break;

    case 'k':
      V -= feet_stream_control.step;
    break;

    case 'j':
      W += feet_stream_control.step*2;
    break;

    case 'l':
      W -= feet_stream_control.step*2;
    break;

    case 'u':
      Vn += feet_stream_control.step;
    break;

    case 'o':
      Vn -= feet_stream_control.step;
    break;

    case 'm':
      if (fabs(feet_stream_control.step) >= 0.01) {
        feet_stream_control.step = feet_stream_control.step + 0.01;
      } else if (fabs(feet_stream_control.step) < 0.01) {
        feet_stream_control.step = feet_stream_control.step + 0.001;
      }
      DEBUG_I("Step value: %f", feet_stream_control.step);
    break;

    case 'n':
      if (fabs(feet_stream_control.step) > 0.01) {
        feet_stream_control.step = feet_stream_control.step - 0.01;
      } else if (fabs(feet_stream_control.step) <= 0.01) {
        feet_stream_control.step = feet_stream_control.step - 0.001;
      }
      DEBUG_I("Step value: %f", feet_stream_control.step);
    break;

    default:
    break;
  }
  DEBUG_I("V: %f, Vn: %f, W: %f", V, Vn, W);
  return true;
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
    if (fabs(feet_stream_control.step) >= 0.01) {
      feet_stream_control.step = feet_stream_control.step + 0.01;
    } else if (fabs(feet_stream_control.step) < 0.01) {
      feet_stream_control.step = feet_stream_control.step + 0.001;
    }
    DEBUG_I("Step value: %f", feet_stream_control.step);
    break;
  case 'n':
    if (fabs(feet_stream_control.step) > 0.01) {
      feet_stream_control.step = feet_stream_control.step - 0.01;
    } else if (fabs(feet_stream_control.step) <= 0.01) {
      feet_stream_control.step = feet_stream_control.step - 0.001;
    }
    DEBUG_I("Step value: %f", feet_stream_control.step);
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
  case '6':
    feet_stream_control.selected = 5;
  case 'i':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].x() = feet_stream_control.feet_shifts[feet_stream_control.selected].x() + feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.y() += feet_stream_control.step;
    break;
  case 'k':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].x() = feet_stream_control.feet_shifts[feet_stream_control.selected].x() - feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.y() -= feet_stream_control.step;
    break;
  case 'j':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].y() = feet_stream_control.feet_shifts[feet_stream_control.selected].y() + feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.x() -= feet_stream_control.step;
    break;
  case 'l':
    if (feet_stream_control.selected < 4)
      feet_stream_control.feet_shifts[feet_stream_control.selected].y() = feet_stream_control.feet_shifts[feet_stream_control.selected].y() - feet_stream_control.step;
    else if (feet_stream_control.selected == 4)
      feet_stream_control.body_orientation.x() += feet_stream_control.step;
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
  case 'r':
    feet_stream_control.body_translation.x() = feet_stream_control.body_translation.x() + feet_stream_control.step;
    break;
  case 'f':
    feet_stream_control.body_translation.x() = feet_stream_control.body_translation.x() - feet_stream_control.step;
    break;
  case 'd':
    feet_stream_control.body_translation.y() = feet_stream_control.body_translation.y() + feet_stream_control.step;
    break;
  case 'g':
    feet_stream_control.body_translation.y() = feet_stream_control.body_translation.y() - feet_stream_control.step;
    break;
  case 'e':
    feet_stream_control.body_translation.z() = feet_stream_control.body_translation.z() + feet_stream_control.step;
    break;
  case 't':
    feet_stream_control.body_translation.z() = feet_stream_control.body_translation.z() - feet_stream_control.step;
    break;
  case 'p':
    confirm_servos();
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
      double a[3] = {sh[leg][f]*(M_PI/180), el[leg][f]*(M_PI/180), wr[leg][f]*(M_PI/180)};

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