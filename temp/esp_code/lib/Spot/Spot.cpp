#include "SpotServo.hpp"
#include <SCServo.h>
#include "STSCTRL.h"
#include "Spot.hpp"
#include <algorithm>
using namespace std;

#define ENABLE_DEBUG
#include <MacroDebugger.h>


#define STD_SPEED 200

Spot::Spot()
{
    Init_Servos();
    for (int i = 0; i < N_SERVOS; i++)
    {
        ID[i] = Servo_List[i].Get_servo_ID();
    }
    model = SpotModel();
    torsoOrientationRPY = Eigen::Vector3d(0, 0, 0); // TODO ideally read the position of the servos and
    torsoPosition = Eigen::Vector3d(0, 0, 0);       // do the direct kinematics and star from there.
}

void Spot::Init()
{
    for (int i = 0; i < N_SERVOS; i++)
    {
        this->Servo_List[i].Init();
    }

    pose(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
}

void Spot::Init_Servos()
{
    this->Servo_List[0] = SpotServo(1, 0, 0, 0, FL, Shoulder, true);
    this->Servo_List[1] = SpotServo(2, 0, 0, 0, FL, Elbow, true);
    this->Servo_List[2] = SpotServo(3, 0, 0, 0, FL, Wrist, true);
    this->Servo_List[3] = SpotServo(4, 0, 0, 0, FR, Shoulder, true);
    this->Servo_List[4] = SpotServo(5, 0, 0, 0, FR, Elbow, true);
    this->Servo_List[5] = SpotServo(6, 0, 0, 0, FR, Wrist, true);
    this->Servo_List[6] = SpotServo(7, 0, 0, 0, RL, Shoulder, true);
    this->Servo_List[7] = SpotServo(8, 0, 0, 0, RL, Elbow, true);
    this->Servo_List[8] = SpotServo(9, 0, 0, 0, RL, Wrist, true);
    this->Servo_List[9] = SpotServo(10, 0, 0, 0, RR, Shoulder, true);
    this->Servo_List[10] = SpotServo(11, 0, 0, 0, RR, Elbow, true);
    this->Servo_List[11] = SpotServo(12, 0, 0, 0, RR, Wrist, true);
}

void Spot::Update_Spot(int ACC)
{
    s16 Goal_Pos_list[N_SERVOS];
    u16 Speed_list[N_SERVOS];
    u8 ACC_list[N_SERVOS];

    for (int i = 0; i < N_SERVOS; i++)
    {
        Goal_Pos_list[i] = Servo_List[i].Get_Goal_Pos_w_Offset();
        Speed_list[i] = Servo_List[i].Get_Speed();
        ACC_list[i] = ACC;
    }
    st.SyncWritePosEx(ID, N_SERVOS, Goal_Pos_list, Speed_list, ACC_list);
}

bool Spot::all_goals_reached()
{
    for (int i = 0; i < N_SERVOS; i++)
    {
        if (!Servo_List[i].GoalReached())
        {
            return false;
        }
    }
    return true;

}

void Spot::getFootPosition(int leg, Eigen::Vector3d &footPosition){
    footPosition = model.T_bf[leg].block<3,1>(0,3);
}

void Spot::move_feet(Eigen::Vector3d vectors[NUM_LEGS]){
    model.IKFeetOverrides(joint_angles, torsoOrientationRPY, torsoPosition, vectors);

    int iterator = 0;
    double speeds[NUM_JOINTS];

    for(int i = 0; i < NUM_LEGS; i++){
      double dt_movement = Leg_Joint_Speeds(speeds, joint_angles[i], iterator/3, STD_SPEED);
      for(int j = 0; j < NUM_JOINTS; j++){
        Servo_List[iterator].SetGoal(joint_angles[i][j] * 180 / M_PI, speeds[iterator%3]);
        iterator += 1;
      }
    }
    Update_Spot(50);
}

void Spot::move_foot(int leg, Eigen::Vector3d vector){
    model.IK_singular(joint_angles[leg], vector, leg);

    double speeds[NUM_JOINTS];
    double dt = Leg_Joint_Speeds(speeds, joint_angles[leg], leg, STD_SPEED);

    for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
        Servo_List[leg * 3 + jointIdx].SetGoal(joint_angles[leg][jointIdx] * 180 / M_PI, speeds[jointIdx]);
    }
    Update_Spot(50);
}

void Spot::rotate(double row, double pitch, double yaw){
    DEBUG_I("row: %f, pitch: %f, yaw: %f", row, pitch, yaw);
    torsoOrientationRPY = Eigen::Vector3d(row, pitch, yaw);
    model.rotateBody(joint_angles, torsoOrientationRPY);

    int iterator = 0;
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
            //Servo_List[iterator].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, STD_SPEED);
            iterator += 1;
        }
    }
    //Update_Spot(50);
}

void Spot::pose(Eigen::Vector3d orientation, Eigen::Vector3d position){
    model.IK(joint_angles, orientation, position);
    int iterator = 0;
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
            Servo_List[iterator].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, STD_SPEED);
            iterator += 1;
        }
    }
    Update_Spot(50);
}

bool Spot::touch_ground(int leg){
    if (Servo_List[leg * 3].getLoad() >= 0.0) return false;

    while(Servo_List[leg * 3].getLoad() >= 0.0){
        
        Update_Spot(50);
    }
    return true;
}

double Spot::Leg_Joint_Speeds(double (&speed)[3], double angles[3], int leg, int speed_const)
{
    double angles_[3] = 
        {angles[0] * 180/PI,
         angles[1] * 180/PI,
         angles[2] * 180/PI};
    
    // DEBUG_I("LEG: %d, angles[0]: %f, angles[1]: %f, angles[2]: %f", leg, angles[0], angles[1], angles[2]);

    double s_estimate = Servo_List[leg * 3].GetPoseEstimate();
    double e_estimate = Servo_List[leg * 3 + 1].GetPoseEstimate();
    double w_estimate = Servo_List[leg * 3 + 2].GetPoseEstimate();

    
    double shoulder_dist = abs(angles_[0] - s_estimate);
    double elbow_dist    = abs(angles_[1] - e_estimate);
    double wrist_dist    = abs(angles_[2] - w_estimate);

    // DEBUG_I("s_dist: %f, e_dist: %f, w_dist: %f", shoulder_dist, elbow_dist, wrist_dist);

    double scaling_factor = max(shoulder_dist, elbow_dist, wrist_dist);

    double dt_movement = scaling_factor / (speed_const * 0.087912);

    double shoulder_scaled = shoulder_dist / scaling_factor;
    double elbow_scaled = elbow_dist / scaling_factor;
    double wrist_scaled = wrist_dist / scaling_factor;

    double s_speed = 0.0;
    double e_speed = 0.0;
    double w_speed = 0.0;

    s_speed = (shoulder_dist < POS_ERROR_THRESHOLD) ? 0 : speed_const / shoulder_scaled;
    e_speed = (elbow_dist < POS_ERROR_THRESHOLD)    ? 0 : speed_const / elbow_scaled;
    w_speed = (wrist_dist < POS_ERROR_THRESHOLD)    ? 0 : speed_const / wrist_scaled;

    speed[0] = s_speed;
    speed[1] = e_speed;
    speed[2] = w_speed;

    // DEBUG_I("s_speed: %f  e_speed: %f  w_speed: %f", s_speed, e_speed, w_speed);

    return dt_movement;
}

void Spot::getPositionString(char (&PosString)[256], long time)
{
    memset(PosString, 0, 256);
    double Pos = 0;
    int offset = 0;
    offset += sprintf(PosString, "Q, %ld", time);
    for (int i = 0; i < N_SERVOS; i++)
    {
        Pos = Servo_List[i].GetPoseEstimate();
        offset += sprintf(PosString + offset, ", %.1f", Pos);
        if (offset >= 256)
        {
            break; // Prevent buffer overflow
        }
    }
}

void Spot::getLoadString(char (&LoadString)[256], long time) // MAX_BUFFER_LEN = 256
{
    memset(LoadString, 0, 256);
    double Load = 0;
    double speed = 0;
    int offset = 0;
    offset += sprintf(LoadString, "Q, %ld", time);
    for (int i = 0; i < N_SERVOS; i++)
    {
        Load = Servo_List[i].getLoad();
        speed = Servo_List[i].Get_Speed();
        offset += sprintf(LoadString + offset, ", %.0f", Load);
        offset += sprintf(LoadString + offset, ", %.0f", speed);
        if (offset >= 256)
        {
            break; // Prevent buffer overflow
        }
    }
}

double Spot::getLoads()
{
    double Loads = 0;
    double Load = 0;
    for (int i = 0; i < N_SERVOS; i++)
    {
        Load = Servo_List[i].getLoad();
        Loads = Loads + abs(Load) / 1000;
    }
    return Loads;
}

void Spot::set_stance_wspeed(const double &l_shoulder_stance, const double &l_elbow_stance, const double &l_wrist_stance,
                             const double &r_shoulder_stance, const double &r_elbow_stance, const double &r_wrist_stance, double &speed)
{
    float speedShoulder, speedElbow, speedWrist, speedcalc, positionShoulder, positionElbow, positionWrist, loadShoulder, loadElbow, loadWrist = 0;

    Servo_List[FL_SHOULDER].SetGoal(l_shoulder_stance, 500);
    Servo_List[FL_ELBOW].SetGoal(l_elbow_stance, speed);
    Servo_List[FL_WRIST].SetGoal(l_wrist_stance, speed);
    Servo_List[FR_SHOULDER].SetGoal(r_shoulder_stance, 500);
    Servo_List[FR_ELBOW].SetGoal(r_elbow_stance, speed);
    Servo_List[FR_WRIST].SetGoal(r_wrist_stance, speed);
    Servo_List[RL_SHOULDER].SetGoal(l_shoulder_stance, 500);
    Servo_List[RL_ELBOW].SetGoal(l_elbow_stance, speed);
    Servo_List[RL_WRIST].SetGoal(l_wrist_stance, speed);
    Servo_List[RR_SHOULDER].SetGoal(r_shoulder_stance, 500);
    Servo_List[RR_ELBOW].SetGoal(r_elbow_stance, speed);
    Servo_List[RR_WRIST].SetGoal(r_wrist_stance, speed);

    Update_Spot(0);
}

void Spot::straight_calibration_stance()
{
    // set_stance(0,0,0,0,0,0);
    double speed = 500.0;
    set_stance_wspeed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, speed);
}

void Spot::prone_calibration_stance()
{
    double Left_shoulder_stance = 0.0;
    double Left_elbow_stance = -90.0;
    double Left_wrist_stance = 120.0;
    double Right_shoulder_stance = 0.0;
    double Right_elbow_stance = 90.0;
    double Right_wrist_stance = -120.0;
    double speed = 500.0;
    // set_stance(Left_shoulder_stance, Left_elbow_stance, Left_wrist_stance, Right_shoulder_stance, Right_elbow_stance, Right_wrist_stance);
    set_stance_wspeed(Left_shoulder_stance, Left_elbow_stance, Left_wrist_stance, Right_shoulder_stance, Right_elbow_stance, Right_wrist_stance, speed);
}

double Spot::max(double a0, double a1, double a2)
{
    if (a0 >= a1 && a0 >= a2)
    {
        return a0;
    }
    if (a1 >= a0 && a1 >= a2)
    {
        return a1;
    }
    if (a2 >= a1 && a2 >= a0)
    {
        return a2;
    }
    return 0;
}

// TODO add inverse kinematics