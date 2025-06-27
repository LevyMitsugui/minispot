#include "SpotServo.hpp"
#include <SCServo.h>
#include "STSCTRL.h"
#include "Spot.hpp"
#include <algorithm>
using namespace std;

#define ENABLE_DEBUG
#include <MacroDebugger.h>


#define STD_SPEED 450 // steps/s
#define STD_SPEED_RAD 0.8 // rad/s

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

    // frame_forward = 0; // TODO remove this
    // frame_backward = 0;
    // frame_right = 0;
    // frame_left = 0;
    // frame_rotate_right = 0;
    // frame_rotate_left = 0;
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

void Spot::move_feet(Eigen::Vector3d vectors[NUM_LEGS]){ // TODO Still using old speed calc
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

double Spot::move_foot(int leg, Eigen::Vector3d vector, double max_speed){
    model.IK_singular(joint_angles[leg], vector, leg);

    double speeds[NUM_JOINTS];
    double dt = Leg_Joint_Speeds_2(speeds, joint_angles[leg], leg, max_speed);

    for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
        Servo_List[leg * 3 + jointIdx].SetGoal(joint_angles[leg][jointIdx] * 180 / M_PI, speeds[jointIdx]); // TODO see how the code use these speeds, even though it says it is deg/s i strongly doubt it.
    }
    Update_Spot(50);
    return dt;
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
            Servo_List[iterator].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, STD_SPEED_RAD);
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

void Spot::perform_gait_singular(int leg, int nFrames, float (*posFrames)[3], double timeInterval_us){
    double currentTime_us = micros();
    double prevTime_us = currentTime_us;
    double dt = 0.0;
    int frameNumber = 0;

    while (frameNumber < nFrames){
        currentTime_us = micros();
        dt = currentTime_us - prevTime_us;
        //DEBUG_I("current: %f, Previous : %f, dt: %f", currentTime_us, prevTime_us, dt);
        if(dt > timeInterval_us){
            prevTime_us = currentTime_us;

            Eigen::Vector3d vecPos(posFrames[frameNumber][0], posFrames[frameNumber][1], posFrames[frameNumber][2]);
            move_foot(leg, vecPos, STD_SPEED_RAD);
            frameNumber += 1;
        }
    }
}

// void Spot::perform_gait(int nFrames, float (*posFrames)[19][3], double timeInterval_us, int cycles){
//     double currentTime_us = micros();
//     double prevTime_us = currentTime_us;
//     double dt = 0.0;
//     int frameNumber;

//     Eigen::Vector3d vecPos1(posFrames[0][frameNumber][0], posFrames[0][frameNumber][1], posFrames[0][frameNumber][2]);
//     Eigen::Vector3d vecPos2(posFrames[1][frameNumber][0], posFrames[1][frameNumber][1], posFrames[1][frameNumber][2]);
//     Eigen::Vector3d vecPos3(posFrames[2][frameNumber][0], posFrames[2][frameNumber][1], posFrames[2][frameNumber][2]);
//     Eigen::Vector3d vecPos4(posFrames[3][frameNumber][0], posFrames[3][frameNumber][1], posFrames[3][frameNumber][2]);
//     move_foot(FL, vecPos1);
//     move_foot(FR, vecPos2);
//     move_foot(RL, vecPos3);
//     move_foot(RR, vecPos4);

//     for (int i = 0; i < cycles; i++){
//         frameNumber = 1;
//         while (frameNumber < nFrames){
//             currentTime_us = micros();
//             dt = currentTime_us - prevTime_us;
//             //DEBUG_I("current: %f, Previous : %f, dt: %f", currentTime_us, prevTime_us, dt);
//             if(dt > timeInterval_us){
//                 prevTime_us = currentTime_us;

//                 vecPos1 = Eigen::Vector3d(posFrames[0][frameNumber][0], posFrames[0][frameNumber][1], posFrames[0][frameNumber][2]);
//                 vecPos2 = Eigen::Vector3d(posFrames[1][frameNumber][0], posFrames[1][frameNumber][1], posFrames[1][frameNumber][2]);
//                 vecPos3 = Eigen::Vector3d(posFrames[2][frameNumber][0], posFrames[2][frameNumber][1], posFrames[2][frameNumber][2]);
//                 vecPos4 = Eigen::Vector3d(posFrames[3][frameNumber][0], posFrames[3][frameNumber][1], posFrames[3][frameNumber][2]);
//                 move_foot(FL, vecPos1);
//                 move_foot(FR, vecPos2);
//                 move_foot(RL, vecPos3);
//                 move_foot(RR, vecPos4);
                
//                 frameNumber += 1;
//             }
//         }   
//     }
// }

void Spot::perform_gait(int nFrames, float (*posFrames)[19][3], double timeInterval_us, int cycles){
    double currentTime_us = micros();
    double prevTime_us = currentTime_us;
    double dt = 0.0;
    int frameNumber = 0;
    int cycle = 0;

    Eigen::Vector3d vecPos1(posFrames[0][0][0], posFrames[0][0][1], posFrames[0][0][2]);
    Eigen::Vector3d vecPos2(posFrames[1][0][0], posFrames[1][0][1], posFrames[1][0][2]);
    Eigen::Vector3d vecPos3(posFrames[2][0][0], posFrames[2][0][1], posFrames[2][0][2]);
    Eigen::Vector3d vecPos4(posFrames[3][0][0], posFrames[3][0][1], posFrames[3][0][2]);
    move_foot(FL, vecPos1, STD_SPEED_RAD);
    move_foot(FR, vecPos2, STD_SPEED_RAD);
    move_foot(RL, vecPos3, STD_SPEED_RAD);
    move_foot(RR, vecPos4, STD_SPEED_RAD);
        
    while (cycle < cycles){
        currentTime_us = micros();
        dt = currentTime_us - prevTime_us;
        
        if(dt > timeInterval_us){
            DEBUG_I("Frame: %d", frameNumber); 
            prevTime_us = currentTime_us;

            vecPos1 = Eigen::Vector3d(posFrames[0][frameNumber][0], posFrames[0][frameNumber][1], posFrames[0][frameNumber][2]);
            vecPos2 = Eigen::Vector3d(posFrames[1][frameNumber][0], posFrames[1][frameNumber][1], posFrames[1][frameNumber][2]);
            vecPos3 = Eigen::Vector3d(posFrames[2][frameNumber][0], posFrames[2][frameNumber][1], posFrames[2][frameNumber][2]);
            vecPos4 = Eigen::Vector3d(posFrames[3][frameNumber][0], posFrames[3][frameNumber][1], posFrames[3][frameNumber][2]);
            move_foot(FL, vecPos1, STD_SPEED_RAD);
            move_foot(FR, vecPos2, STD_SPEED_RAD);
            move_foot(RL, vecPos3, STD_SPEED_RAD);
            move_foot(RR, vecPos4, STD_SPEED_RAD);
            
            frameNumber += 1;
        }

        if (frameNumber > 17) {
            frameNumber = 1;
            cycle +=1;
        }
        taskYIELD();
    }   
}

void Spot::perform_gait_no_blocking(float (*posFrames)[19][3], bool is_first_frame){
    if (is_first_frame){
        frame = 0;
        Eigen::Vector3d vecPos1(posFrames[0][0][0], posFrames[0][0][1], posFrames[0][0][2]);
        Eigen::Vector3d vecPos2(posFrames[1][0][0], posFrames[1][0][1], posFrames[1][0][2]);
        Eigen::Vector3d vecPos3(posFrames[2][0][0], posFrames[2][0][1], posFrames[2][0][2]);
        Eigen::Vector3d vecPos4(posFrames[3][0][0], posFrames[3][0][1], posFrames[3][0][2]);
        move_foot(FL, vecPos1, STD_SPEED_RAD);
        move_foot(FR, vecPos2, STD_SPEED_RAD);
        move_foot(RL, vecPos3, STD_SPEED_RAD);
        move_foot(RR, vecPos4, STD_SPEED_RAD);
    }

    // TODO continue Here, now move_foot returns the time it will take to finish its movement.
}

double Spot::Leg_Joint_Speeds(double (&speed)[3], double angles[3], int leg, int speed_const) //TODo fix this
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

double Spot::Leg_Joint_Speeds_2(double (&speed)[3], double angles[3], int leg, double max_speed){ // max speed in rad/s (TODO make a function description coment)
    double angles_[3] = 
        {angles[0],
         angles[1],
         angles[2]};
    
    DEBUG_I("LEG: %d, angles[0]: %f, angles[1]: %f, angles[2]: %f", leg, angles[0], angles[1], angles[2]);

    double s_estimate = Servo_List[leg * 3].GetPoseEstimateRad();
    double e_estimate = Servo_List[leg * 3 + 1].GetPoseEstimateRad();
    double w_estimate = Servo_List[leg * 3 + 2].GetPoseEstimateRad();

    DEBUG_I("s_estimate: %f, e_estimate: %f, w_estimate: %f", s_estimate, e_estimate, w_estimate);
    
    double shoulder_dist = abs(angles[0] - s_estimate);
    double elbow_dist    = abs(angles[1] - e_estimate);
    double wrist_dist    = abs(angles[2] - w_estimate);

    DEBUG_I("s_dist: %f, e_dist: %f, w_dist: %f", shoulder_dist, elbow_dist, wrist_dist);

    double max_angle = max(shoulder_dist, elbow_dist, wrist_dist);

    DEBUG_I("max_angle: %f", max_angle);

    double time = max_angle / max_speed; // Computes the time taken by the servo with the longgest distance to reach the end position goal

    DEBUG_I("Time: %f = %f / %f", time, max_angle, max_speed);

    speed[0] = shoulder_dist / time;
    speed[1] = elbow_dist / time;
    speed[2] = wrist_dist / time;

    DEBUG_I("s_speed: %f  e_speed: %f  w_speed: %f", speed[0], speed[1], speed[2]);

    return time;
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