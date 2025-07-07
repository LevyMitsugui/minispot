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
#define SPEED_LAST_EDITION 30 //0.02 //1.149822906 // 2.299645812 // rad/s

#define VELOCITY_THRESH 0.001
#define GAIT_STOP_TIME_MILLIS 100
#define FRNT_LEGS_X_OFFSET    Eigen::Vector3d(0.05,0.0,0.0)
#define BEZ_CTRL_PTS_Z_OFFSET Eigen::Vector3d(0.0,0.0,0.02)

#define BEZIER_CURVE_PATTERN_1 { \
    Eigen::Vector3d(0.0, 0.0, 0.0), \
    Eigen::Vector3d(0.0, 0.0, 0.1), \
    Eigen::Vector3d(0.08, 0.0, 0.1), \
    Eigen::Vector3d(0.05, 0.0, 0.0) \
}

#define BEZIER_CURVE_PATTERN_2 {    \
    Eigen::Vector3d(0.0, 0.0, 0.0), \
    Eigen::Vector3d(0.8, 0.0, 0.1), \
    Eigen::Vector3d(1.5, 0.0, 1.0), \
    Eigen::Vector3d(1.0, 0.0, 0.0)  \
}


Spot::Spot()
{
    Init_Servos();
    for (int i = 0; i < N_SERVOS; i++)
    {
        ID[i] = Servo_List[i].Get_servo_ID();
    }
    model = SpotModel();
    torsoRPY = Eigen::Vector3d(0, 0, 0); // TODO ideally read the position of the servos and
    torsoLean = Eigen::Vector3d(0, 0, 0);       // do the direct kinematics and star from there.

    torsoRPYTarget = Eigen::Vector3d(0, 0, 0);
    torsoLeanTarget = Eigen::Vector3d(0, 0, 0);
}

void Spot::Init()
{
    for (int i = 0; i < N_SERVOS; i++)
    {
        this->Servo_List[i].Init();
    }
    for (int i = 0; i < NUM_LEGS; i++){
        this-> gaitState_testing[i] = 0;
        this-> legStartingPos[i] = Eigen::Vector3d::Zero(); // TODO Use forward kinematics
        this-> stepState[i] = IDLE_STEP;
    }
    Eigen::Vector3d bP_[BEZIER_CONTROL_POINTS] = BEZIER_CURVE_PATTERN_1;
    // for(int i = 0; i < BEZIER_CONTROL_POINTS; i++) { 
    //     this->bezControlPoints[i] = bP_[i]; 
    // }
    pose();
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

Eigen::Vector3d Spot::getFootPosition(int Leg){
    return model.T_wf[Leg].block<3,1>(0,3);
}

Eigen::Vector3d Spot::getEstimatedFootPosition(int leg){
    double angles[NUM_JOINTS];
    Eigen::Vector3d pos_hip, pos_body;

    for(int i = 0; i < NUM_JOINTS; i++){
        angles[i] = Servo_List[leg * 3 + i].GetPoseEstimateRad();
    }

    model.FK_singular(angles, leg, pos_hip);
    model.HipToBodyV(leg, pos_hip, pos_body);

    return pos_body;
}

double Spot::move_feet(Eigen::Vector3d vectors[NUM_LEGS], double max_speed){ // TODO Still using old IK and old speed calc
    double speeds[NUM_JOINTS];
    double dt = 0.0, dt_ = 0.0;

    for(int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
        model.IK_singular(joint_angles[legIdx], vectors[legIdx], legIdx);
        dt_ = Leg_Joint_Speeds_2(speeds, joint_angles[legIdx], legIdx, max_speed);
        dt = (dt > dt_) ? dt : dt_;
        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
            Servo_List[legIdx * 3 + jointIdx].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, speeds[jointIdx]); // TODO see how the code use these speeds, even though it says it is deg/s i strongly doubt it.
        }
    }
    Update_Spot(0);
    return dt;
}

double Spot::move_foot(int leg, Eigen::Vector3d vector, double max_speed){
    model.IK_singular(joint_angles[leg], vector, leg);

    double speeds[NUM_JOINTS];
    double dt = Leg_Joint_Speeds_2(speeds, joint_angles[leg], leg, max_speed);

    for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
        Servo_List[leg * 3 + jointIdx].SetGoal(joint_angles[leg][jointIdx] * 180 / M_PI, speeds[jointIdx]); // TODO see how the code use these speeds, even though it says it is deg/s i strongly doubt it.
    }
    Update_Spot(0);
    return dt;
}

double Spot::move_foot(int leg, Eigen::Vector3d vector, double max_speed, bool noUpdate){
    model.IK_singular(joint_angles[leg], vector, leg);

    double speeds[NUM_JOINTS];
    double dt = Leg_Joint_Speeds_2(speeds, joint_angles[leg], leg, max_speed);

    for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
        Servo_List[leg * 3 + jointIdx].SetGoal(joint_angles[leg][jointIdx] * 180 / M_PI, speeds[jointIdx]); // TODO see how the code use these speeds, even though it says it is deg/s i strongly doubt it.
    }
    if(!noUpdate) Update_Spot(0);
    return dt;
}

void Spot::translate(double x, double y, double z){
    DEBUG_I("x: %f, y: %f, z: %f", x, y, z);
    torsoLean = Eigen::Vector3d(x, y, z);
    model.ComputeJointAnglesFromTranslation(joint_angles, torsoLean);

    double speeds[NUM_JOINTS];
    double dt = 0;

    int iterator = 0;
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
            Leg_Joint_Speeds_2(speeds, joint_angles[legIdx], legIdx, 0.2);
            Servo_List[iterator].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, speeds[jointIdx]);
            iterator += 1;
        }
    }
}

void Spot::rotate(double row, double pitch, double yaw){
    DEBUG_I("row: %f, pitch: %f, yaw: %f", row, pitch, yaw);
    torsoRPY = Eigen::Vector3d(row, pitch, yaw);
    model.ComputeJointAnglesFromRotation(joint_angles, torsoRPY);

    double speeds[NUM_JOINTS];

    int iterator = 0;
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
            Leg_Joint_Speeds_2(speeds, joint_angles[legIdx], legIdx, 0.2);
            Servo_List[iterator].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, speeds[jointIdx]);
            iterator += 1;
        }
    }
}

void Spot::set_lean(double x, double y, double z){
    torsoLeanTarget = Eigen::Vector3d(x, y, z);
}

void Spot::set_rpy(double row, double pitch, double yaw){
    torsoRPYTarget = Eigen::Vector3d(row, pitch, yaw);
}

double Spot::pose(){
    double speeds[NUM_JOINTS];
    double dt = 0;
    double dt_= 0;

    model.ComputePoseJointAngles(joint_angles, torsoLeanTarget, torsoRPYTarget);

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx){
            dt_ = Leg_Joint_Speeds_2(speeds, joint_angles[legIdx], legIdx, STD_SPEED_RAD);
            dt = (dt > dt_) ? dt : dt_;
            Servo_List[legIdx * 3 + jointIdx].SetGoal(joint_angles[legIdx][jointIdx] * 180 / M_PI, speeds[jointIdx]);
        }
    }
    Update_Spot(0);
    return dt;
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
    double currentTime = millis();
    double dt = timeHelper[1]; // timeHelper[1] will act as the time to finish the last frame (in milliseconds)
    double dt_ = 0.0;
    DEBUG_I("Current Frame: %d", frame);
    DEBUG_I("To next Frame: %f / %f", (currentTime - timeHelper[0]), dt);

    if (is_first_frame){
        timeHelper[0] = currentTime; // timeHelper[0] will act as the previous time variable
        frame = 1;
        Eigen::Vector3d vecPos1(posFrames[0][0][0], posFrames[0][0][1], posFrames[0][0][2]);
        Eigen::Vector3d vecPos2(posFrames[1][0][0], posFrames[1][0][1], posFrames[1][0][2]);
        Eigen::Vector3d vecPos3(posFrames[2][0][0], posFrames[2][0][1], posFrames[2][0][2]);
        Eigen::Vector3d vecPos4(posFrames[3][0][0], posFrames[3][0][1], posFrames[3][0][2]);
        dt = move_foot(FL, vecPos1, SPEED_LAST_EDITION); // dt in seconds!!!
        dt_ = move_foot(FR, vecPos2, SPEED_LAST_EDITION);
        dt = (dt > dt_) ? dt : dt_;
        dt_ = move_foot(RL, vecPos3, SPEED_LAST_EDITION);
        dt = (dt > dt_) ? dt : dt_;
        dt_ = move_foot(RR, vecPos4, SPEED_LAST_EDITION);
        dt = (dt > dt_) ? dt : dt_;
        DEBUG_I("dt : %f seconds", dt);
        timeHelper[1] = dt*1000; // convert it to milliseconds
        DEBUG_I("timeHelper[1]: %f milliseconds", timeHelper[1]);

    } else if (frame < 19 && (currentTime - timeHelper[0]) > dt){
        timeHelper[0] = currentTime;
        Eigen::Vector3d vecPos1(posFrames[0][frame][0], posFrames[0][frame][1], posFrames[0][frame][2]);
        Eigen::Vector3d vecPos2(posFrames[1][frame][0], posFrames[1][frame][1], posFrames[1][frame][2]);
        Eigen::Vector3d vecPos3(posFrames[2][frame][0], posFrames[2][frame][1], posFrames[2][frame][2]);
        Eigen::Vector3d vecPos4(posFrames[3][frame][0], posFrames[3][frame][1], posFrames[3][frame][2]);
        dt = move_foot(FL, vecPos1, SPEED_LAST_EDITION); // dt in seconds!!!
        dt_ = move_foot(FR, vecPos2, SPEED_LAST_EDITION);
        dt = (dt > dt_) ? dt : dt_;
        dt_ = move_foot(RL, vecPos3, SPEED_LAST_EDITION);
        dt = (dt > dt_) ? dt : dt_;
        dt_ = move_foot(RR, vecPos4, SPEED_LAST_EDITION);
        dt = (dt > dt_) ? dt : dt_;
        DEBUG_I("dt : %f seconds", dt);
        timeHelper[1] = dt*1000; // convert it to milliseconds
        DEBUG_I("timeHelper[1]: %f milliseconds", timeHelper[1]);
        frame = (frame >= 17) ? 1 : frame + 1;
        DEBUG_I("Next Frame: %d", frame);
    }
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
    
    // DEBUG_I("LEG: %d, angles[0]: %f, angles[1]: %f, angles[2]: %f", leg, angles[0], angles[1], angles[2]);

    if (max_speed > MAX_SPEED_RAD) max_speed = MAX_SPEED_RAD;

    double s_estimate = Servo_List[leg * 3].GetPoseEstimateRad();
    double e_estimate = Servo_List[leg * 3 + 1].GetPoseEstimateRad();
    double w_estimate = Servo_List[leg * 3 + 2].GetPoseEstimateRad();

    // DEBUG_I("s_estimate: %f, e_estimate: %f, w_estimate: %f", s_estimate, e_estimate, w_estimate);
    
    double shoulder_dist = abs(angles[0] - s_estimate);
    double elbow_dist    = abs(angles[1] - e_estimate);
    double wrist_dist    = abs(angles[2] - w_estimate);

    // DEBUG_I("s_dist: %f, e_dist: %f, w_dist: %f", shoulder_dist, elbow_dist, wrist_dist);

    double max_angle = max(shoulder_dist, elbow_dist, wrist_dist);

    // DEBUG_I("max_angle: %f", max_angle);

    double time = max_angle / max_speed; // Computes the time taken by the servo with the longgest distance to reach the end position goal

    // DEBUG_I("Time: %f = %f / %f", time, max_angle, max_speed);

    speed[0] = shoulder_dist / time;
    speed[1] = elbow_dist / time;
    speed[2] = wrist_dist / time;

    // DEBUG_I("s_speed: %f  e_speed: %f  w_speed: %f", speed[0], speed[1], speed[2]);

    return time; // time in seconds
}

void Spot::bezierControlPoints(
    Eigen::Vector3d (&points)[BEZIER_CONTROL_POINTS],
    Eigen::Vector3d pattern[BEZIER_CONTROL_POINTS],
    Eigen::Vector3d startingPos,
    Eigen::Vector3d stancePoints)
{
    //points[0] = s
}

Eigen::Vector3d Spot::bezier(double t,
                           Eigen::Vector3d p0,
                           Eigen::Vector3d p1,
                           Eigen::Vector3d p2,
                           Eigen::Vector3d p3)
{
    return pow(1-t, 3)*p0 +
           3*pow(1-t, 2)*t*p1 +
           3*(1-t)*pow(t, 2)*p2 +
           pow(t, 3)*p3;
}

void Spot::getStancePoints(int leg, Eigen::Vector3d &stancePoints, Eigen::Vector3d &stanceVelocities){
    Eigen::Vector3d midPoint = model.startingFeetPos[leg];
    if(leg == FR || leg == FL) midPoint += FRNT_LEGS_X_OFFSET;
    // midPoint += FRNT_LEGS_X_OFFSET;
    //DEBUG_I("midPoint: %f, %f, %f", midPoint[0], midPoint[1], midPoint[2]);
    stancePoints = (midPoint + (-stanceVelocities * gaitTst)/2);
    //DEBUG_I("stancePoints[0]: %f, %f, %f", stancePoints[0], stancePoints[1], stancePoints[2]);
    //stancePoints[1] = (midPoint + (stanceVelocities[0] * gaitTst)/2);
}

bool Spot::performDynamicGait(double &currTimeMillis, Eigen::Vector3d Velocities[NUM_LEGS]){
    double t_swing = 0.0;
    Eigen::Vector3d stanceTargetPoint;
    Eigen::Vector3d swingTargetPoint;

    // Update finite state machine timer
    updateStateTime(gaitState, currTimeMillis);
    //     state,time(ms),x(m),y(m),z(m)
    DEBUG_I(",%d,%f,%f,%f,%f", gaitState.state, currTimeMillis,
        getFootPosition(FL)[0],
        getFootPosition(FL)[1],
        getFootPosition(FL)[2]);

    // define state actions (actual outputs, like motor movement, is done later)
    switch (gaitState.state){
 /*0*/  case IDLE_GAIT:
            if (gaitState.stateChanged){
                for (int i = 0; i < NUM_LEGS; i++){
                    move_foot(i, model.startingFeetPos[i], 4.8, true);
                }
            }
            break;
 /*1*/  case STARTER_SW1_ST2:
            // Plan only stance phase for pair 2 (fr + rl)
            // Plan whole step for pair 1 (fl + rr)
            if (gaitState.stateChanged){
               for (int i = 0; i < NUM_LEGS; i++){
                   legStartingPos[i] = getFootPosition(i);
               } 
               
                getStancePoints(0, stancePoints[0], stanceVelocities[0]);
                bezControlPoints[0][0] = legStartingPos[0];
                bezControlPoints[0][1] = legStartingPos[0] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[0][2] =   stancePoints[0] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[0][3] =   stancePoints[0];

                getStancePoints(3, stancePoints[3], stanceVelocities[3]);
                bezControlPoints[3][0] = legStartingPos[3];
                bezControlPoints[3][1] = legStartingPos[3] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[3][2] =   stancePoints[3] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[3][3] =   stancePoints[3];
            }
            
            // Perform pair 2 stance
            stanceTargetPoint = legStartingPos[1] + (stanceVelocities[1] * gaitState.tis/1000);
            move_foot(1, stanceTargetPoint, 4.8, true);
            stanceTargetPoint = legStartingPos[2] + (stanceVelocities[2] * gaitState.tis/1000);
            move_foot(2, stanceTargetPoint, 4.8, true);

            // Perform pair 1 swing
            t_swing = gaitState.tis/(gaitTsw*1000);
            if (t_swing > 1.0) t_swing = 1.0;

            for(int i = 0; i<4; i += 3){
                swingTargetPoint = bezier(t_swing,
                                  bezControlPoints[i][0],
                                  bezControlPoints[i][1],
                                  bezControlPoints[i][2],
                                  bezControlPoints[i][3]);
                move_foot(i, swingTargetPoint, 4.8, true);
            }

            break;
 /*2*/  case ST1_SW2:
            // Plan whole step for pair 2 (fr + rl)
            if (gaitState.stateChanged){
                updateVelocities(2, Velocities);
                legStartingPos[1] = getFootPosition(1);
                legStartingPos[2] = getFootPosition(2);

                getStancePoints(1, stancePoints[1], stanceVelocities[1]);
                bezControlPoints[1][0] = legStartingPos[1];
                bezControlPoints[1][1] = legStartingPos[1] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[1][2] =   stancePoints[1] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[1][3] =   stancePoints[1];

                getStancePoints(2, stancePoints[2], stanceVelocities[2]);
                bezControlPoints[2][0] = legStartingPos[2];
                bezControlPoints[2][1] = legStartingPos[2] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[2][2] =   stancePoints[2] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[2][3] =   stancePoints[2];
            }
            // Perform pair 1 stance
            stanceTargetPoint = stancePoints[0] + (stanceVelocities[0] * gaitState.tis/1000);
            move_foot(0, stanceTargetPoint, 4.8, true);
            stanceTargetPoint = stancePoints[3] + (stanceVelocities[3] * gaitState.tis/1000);
            move_foot(3, stanceTargetPoint, 4.8, true);

            // Perform pair 2 swing
            t_swing = gaitState.tis/(gaitTsw*1000);
            if (t_swing > 1.0) t_swing = 1.0;
            
            for(int i = 1; i<3; i += 1){
                swingTargetPoint = bezier(t_swing,
                                  bezControlPoints[i][0],
                                  bezControlPoints[i][1],
                                  bezControlPoints[i][2],
                                  bezControlPoints[i][3]);
                move_foot(i, swingTargetPoint, 4.8, true);
            }
            break;
 /*3*/  case SW1_ST2:
            // Plan whole step for pair 1 (fl + rr)
            if(gaitState.stateChanged){
                updateVelocities(1, Velocities);
                legStartingPos[0] = getFootPosition(0);
                legStartingPos[3] = getFootPosition(3);

                getStancePoints(0, stancePoints[0], stanceVelocities[0]);
                bezControlPoints[0][0] = legStartingPos[0];
                bezControlPoints[0][1] = legStartingPos[0] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[0][2] =   stancePoints[0] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[0][3] =   stancePoints[0];
                
                getStancePoints(3, stancePoints[3], stanceVelocities[3]);
                bezControlPoints[3][0] = legStartingPos[3];
                bezControlPoints[3][1] = legStartingPos[3] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[3][2] =   stancePoints[3] + BEZ_CTRL_PTS_Z_OFFSET;
                bezControlPoints[3][3] =   stancePoints[3];
            }
            // Perform pair 1 swing
            t_swing = gaitState.tis/(gaitTsw*1000);
            if (t_swing > 1.0) t_swing = 1.0;
            
            for (int i = 0; i < 4; i += 3){
                swingTargetPoint = bezier(t_swing,
                                bezControlPoints[i][0],
                                bezControlPoints[i][1],
                                bezControlPoints[i][2],
                                bezControlPoints[i][3]);
                move_foot(i, swingTargetPoint, 4.8, true);
            }

            // Perform pair 2 stance
            stanceTargetPoint = stancePoints[1] + stanceVelocities[1] * gaitState.tis/1000;
            move_foot(1, stanceTargetPoint, 4.8, true);
            stanceTargetPoint = stancePoints[2] + stanceVelocities[2] * gaitState.tis/1000;
            move_foot(2, stanceTargetPoint, 4.8, true);
            break;
    }

    // Compute next state
    if(gaitState.state == IDLE_GAIT){
        if (gaitState.tis > GAIT_STOP_TIME_MILLIS){
            for(int i = 0; i < NUM_LEGS; i++){
                if (Velocities[i].norm() > VELOCITY_THRESH){
                    gaitState.new_state = STARTER_SW1_ST2;
                    updateVelocities(0, Velocities);
                }
            }
        }
    } else if (gaitState.state == STARTER_SW1_ST2){
        if(gaitState.tis > gaitTst*1000){
            gaitState.new_state = ST1_SW2;
        } else {
            for(int i = 0; i < NUM_LEGS; i++){
                if (Velocities[i].norm() <= VELOCITY_THRESH){
                    gaitState.new_state = IDLE_GAIT;
                }
            }
        }
    } else if (gaitState.state == ST1_SW2){
        if(gaitState.tis > gaitTst*1000){
            gaitState.new_state = SW1_ST2;
        } else {
            for(int i = 0; i < NUM_LEGS; i++){
                if (Velocities[i].norm() <= VELOCITY_THRESH){
                    gaitState.new_state = IDLE_GAIT;
                }
            }
        }
    } else if (gaitState.state == SW1_ST2){
        if (gaitState.tis > gaitTst*1000){
            gaitState.new_state = ST1_SW2;
        } else {
            for(int i = 0; i < NUM_LEGS; i++){
                if (Velocities[i].norm() <= VELOCITY_THRESH){
                    gaitState.new_state = IDLE_GAIT;
                }
            }
        }
    }

    // Update: state ➞ new state
    setState(gaitState, currTimeMillis);

    // Update servos
    //  ↳done in main loop !!

    return true;  
}

bool Spot::setState(fsm_t &stateMachine, double &currTimeMillis){
    if (stateMachine.new_state != stateMachine.state){
        stateMachine.state = stateMachine.new_state;
        stateMachine.tes = currTimeMillis;
        stateMachine.tis = 0.0;
        stateMachine.stateChanged = true;
        return true;
    }

    stateMachine.stateChanged = false;
    return false;
}

void Spot::setGaitState(int state, double &currTimeMillis){
    gaitState.new_state = state;
    setState(gaitState, currTimeMillis);
}

void Spot::updateStateTime(fsm_t &stateMachine, double &currTimeMillis){ // Has to be called every loop, once
    stateMachine.tis = currTimeMillis - stateMachine.tes;
}

void Spot::updateVelocities(int pair, Eigen::Vector3d Velocities[NUM_LEGS]){
    if (pair == 1){
        stanceVelocities[0] = Velocities[0];
        stanceVelocities[3] = Velocities[3];
    } else if (pair == 2){
        stanceVelocities[1] = Velocities[1];
        stanceVelocities[2] = Velocities[2];
    } else {
        for (int i = 0; i < NUM_LEGS; i++){
            stanceVelocities[i] = Velocities[i];
        }
    }
}

bool Spot::performStep(int Leg, double &currTimeMillis){ // TODO if this works, make it perform the step for 2 legs at the same time
    // Define points first
    if (stepState[Leg] == IDLE_STEP){
        getStancePoints(Leg, stancePoints[Leg], stanceVelocities[Leg]);
        legStartingPos[Leg] = getFootPosition(Leg);
        bezControlPoints[Leg][0] = legStartingPos[Leg];
        bezControlPoints[Leg][1] = legStartingPos[Leg] + Eigen::Vector3d(0.0,0.0,0.6);
        bezControlPoints[Leg][2] = stancePoints[Leg]+ Eigen::Vector3d(0.0,0.0,0.6);
        bezControlPoints[Leg][3] = stancePoints[Leg];

        for(int i=0; i<4; i++) { DEBUG_I("bezControlPoints[%d]: %f, %f, %f", i, bezControlPoints[Leg][i][0], bezControlPoints[Leg][i][1], bezControlPoints[Leg][i][2]); }

        gaitStartTime[Leg] = currTimeMillis;//millis();
        stepState[Leg] = SWING;
    }

    double t_swing = 0.0;
    Eigen::Vector3d bezPoint;

    // Then perform Bezier (swing phase) From current position to the first point
    if (stepState[Leg] == SWING){
        t_swing = (currTimeMillis - gaitStartTime[Leg])/(gaitTsw*1000);
        DEBUG_I("t_swing: %f", t_swing);
        if (t_swing > 1.0){
            t_swing = 1.0;
            gaitStartTime[Leg] = currTimeMillis; // millis();
            stepState[Leg] = STANCE;
        } else if (t_swing < 0.0){
            return true;
        }

        bezPoint = bezier(t_swing, 
                          bezControlPoints[Leg][0], 
                          bezControlPoints[Leg][1], 
                          bezControlPoints[Leg][2], 
                          bezControlPoints[Leg][3]);
        
        DEBUG_I("%f%,%f,%f,%f,%f",t_swing*100, currTimeMillis, bezPoint[0], bezPoint[1], bezPoint[2]);
        move_foot(Leg, bezPoint, 120.0, true);
    }

    Eigen::Vector3d stanceTargetPoint;
    double dt = 0.0;

    // Then perform Stance phase
    if (stepState[Leg] == STANCE){
        dt = currTimeMillis - gaitStartTime[Leg];

        if (dt > gaitTst*1000){
            stepState[Leg] = END_STEP;
            return false;
        } else if (dt < 0.0){
            return true;
        }

        stanceTargetPoint = stancePoints[Leg] + (stanceVelocities[Leg] * dt/1000);
        DEBUG_I("stanceTargetPoint: %f, %f, %f", stanceTargetPoint[0], stanceTargetPoint[1], stanceTargetPoint[2]);
        move_foot(Leg, stanceTargetPoint, 10, true);
    }

    return true;
}

void Spot::setPeriods(double totalPeriod, double stancePeriod, double swingPeriod){
    if(totalPeriod <= 0.0 || stancePeriod <= 0.0 || swingPeriod <= 0.0 || (
        stancePeriod + swingPeriod > totalPeriod
    )){
        DEBUG_E("Invalid Gait Periods, total period must be greater than stance + swing, total: %f, stance %f, swing: %f", totalPeriod, stancePeriod, swingPeriod);
        return;
    }
    gaitT    = totalPeriod;     // seconds. Period of the entire gait pattern
    gaitTst  = stancePeriod;    // seconds. Period of the stance phase
    gaitTsw  = swingPeriod;     // seconds. Period of the swing phase
    DEBUG_I("Applied Gait Periods: total: %f, stance %f, swing: %f", gaitT, gaitTst, gaitTsw);
}

void Spot::setStanceVelocity(int Leg, Eigen::Vector3d stanceVelocities){
    if (stepState[Leg] != STANCE || stepState[Leg] != SWING){
       this->stanceVelocities[Leg] = stanceVelocities; 
    } else {
        DEBUG_I("Cannot set stance velocity while in stance or swing phase");
    }

    DEBUG_I("Applied to leg %d: %f, %f, %f", Leg,
             this->stanceVelocities[Leg][0], 
             this->stanceVelocities[Leg][1], 
             this->stanceVelocities[Leg][2]);
}

int Spot::getStepState(int Leg){
    return stepState[Leg];
}

bool Spot::isStepDone(int Leg){
    bool ret = false;
    if(stepState[Leg] == END_STEP || stepState[Leg] == IDLE_STEP){
        ret = true;
        stepState[Leg] = IDLE_STEP;
    }
    
    return ret;
}

bool Spot::isStepDone(int Leg, int Leg2){
    bool ret = false;
    if(stepState[Leg] == END_STEP && stepState[Leg2] == END_STEP){
        ret = true;
        stepState[Leg] = IDLE_STEP;
        stepState[Leg2] = IDLE_STEP;
    }
    
    return ret;
}

// - - - Testing Space - - -
bool Spot::performStancePhase(){
    Eigen::Vector3d stanceTargetPoint;
    double dt = 0.0;

    stanceVelocities[0] = Eigen::Vector3d(-0.5, 0.5, 0.0);

    if(gaitState_testing[0] == 2){
        gaitState_testing[0] = 3;
        gaitCurrTime[0]  = millis();
        gaitStartTime[0] = gaitCurrTime[0];
        gaitPrevTime[0] = gaitCurrTime[0];

        gaitT    = 0.400;    // seconds. Period of the entire gait pattern
        gaitTst  = 0.200;     // seconds. Period of the stance phase
        gaitTsw  = gaitT - gaitTst;  // seconds. Period of the swing phase

        gaitProcessPeriod = 1; // ms
        //DEBUG_I("Gait State: %d", gaitState_testing[0]);

        getStancePoints(0, stancePoints[0], stanceVelocities[0]);
    }

    gaitCurrTime[0] = millis();
    if(gaitCurrTime[0] - gaitPrevTime[0] > gaitProcessPeriod){
        gaitPrevTime[0] = gaitCurrTime[0],
        dt = gaitCurrTime[0] - gaitStartTime[0];

        if (dt > gaitTst*1000){
            gaitState_testing[0] = 0;
            return false;
        }

        stanceTargetPoint = stancePoints[0] + (stanceVelocities[0] * dt/1000);
        DEBUG_I("stanceTargetPoint: %f, %f, %f", stanceTargetPoint[0], stanceTargetPoint[1], stanceTargetPoint[2]);
        move_foot(0, stanceTargetPoint, 10);
    };
    return true;
}

bool Spot::testGaitGen(){
    double t_swing = 0.0;

    Eigen::Vector3d swingPosTarget;
    Eigen::Vector3d bezierPoints[4] = BEZIER_CURVE_PATTERN_1;
    Eigen::Vector3d bezPoint;

    if (gaitState_testing[0] == 0){
        gaitCurrTime[0]  = millis();
        gaitStartTime[0] = gaitCurrTime[0];
        gaitPrevTime[0]  = gaitCurrTime[0];
        gaitState_testing[0]  = 1;
        legStartingPos[0] = getFootPosition(0);

        gaitT    = 0.400;    // seconds. Period of the entire gait pattern
        gaitTst  = 0.200;     // seconds. Period of the stance phase
        gaitTsw  = gaitT - gaitTst;  // seconds. Period of the swing phase

        gaitProcessPeriod = 0.01; // ms
        //DEBUG_I("Gait State: %d", gaitState_testing[0]);
    }
    

    gaitCurrTime[0] = millis();
    //DEBUG_I("Gait Process Time: %f/%f", gaitCurrTime-gaitPrevTime, gaitProcessPeriod);
    if(gaitCurrTime[0] - gaitPrevTime[0] > gaitProcessPeriod){
        
        gaitPrevTime[0] = gaitCurrTime[0];
        t_swing = (gaitCurrTime - gaitStartTime)/(gaitTsw*1000);

        if (t_swing > 1.0){
            t_swing = 1.0;
            gaitState_testing[0] = 2;
        }

        bezPoint = bezier(t_swing, bezierPoints[0], bezierPoints[1], bezierPoints[2], bezierPoints[3])+
                   legStartingPos[0];
        DEBUG_I("%f%,%f,%f,%f,%f",t_swing*100, gaitCurrTime, bezPoint[0], bezPoint[1], bezPoint[2]);
        move_foot(0, bezPoint, 120.0);
    }
    return (gaitState_testing[0] == 2);
}
// - - - - - - - - - - - - -

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

    Servo_List[FL_SHOULDER].SetGoal(l_shoulder_stance, speed);
    Servo_List[FL_ELBOW].SetGoal(l_elbow_stance, speed);
    Servo_List[FL_WRIST].SetGoal(l_wrist_stance, speed);
    Servo_List[FR_SHOULDER].SetGoal(r_shoulder_stance, speed);
    Servo_List[FR_ELBOW].SetGoal(r_elbow_stance, speed);
    Servo_List[FR_WRIST].SetGoal(r_wrist_stance, speed);
    Servo_List[RL_SHOULDER].SetGoal(l_shoulder_stance, speed);
    Servo_List[RL_ELBOW].SetGoal(l_elbow_stance, speed);
    Servo_List[RL_WRIST].SetGoal(l_wrist_stance, speed);
    Servo_List[RR_SHOULDER].SetGoal(r_shoulder_stance, speed);
    Servo_List[RR_ELBOW].SetGoal(r_elbow_stance, speed);
    Servo_List[RR_WRIST].SetGoal(r_wrist_stance, speed);

    Update_Spot(0);
}

void Spot::straight_calibration_stance()
{
    // set_stance(0,0,0,0,0,0);
    double speed = STD_SPEED_RAD;
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
    double speed = STD_SPEED_RAD;
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