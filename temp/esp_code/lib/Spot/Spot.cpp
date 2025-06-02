#include "SpotServo.hpp"
#include <SCServo.h>
#include "STSCTRL.h"
#include "Spot.hpp"
#include <algorithm>
using namespace std;

Spot::Spot()
{
    Init_Servos();
    for (int i = 0; i < N_SERVOS; i++)
    {
        ID[i] = Servo_List[i].Get_servo_ID();
    }
}

void Spot::Init()
{
    for (int i = 0; i < N_SERVOS; i++)
    {
        this->Servo_List[i].Init();
    }
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
// void Spot::Initialize(SpotServo *Servo_List_[], int nServos_)
// {
//     nServos = nServos_;
//     for (int i = 0; i < nServos; i++)
//     {
//         // Find the correct position to insert the new servo
//         int j = i;
//         while (j > 0 && Servo_List_[i]->Get_servo_ID() < Servo_List[j - 1]->Get_servo_ID())
//         {
//             Servo_List[j] = Servo_List[j - 1];
//             j--;
//         }
//         // Insert the new servo at the correct position
//         Servo_List[j] = Servo_List_[i];
//         ID[j] = Servo_List[j]->Get_servo_ID();
//     }
// }

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

double Spot::Leg_Joint_Speeds(double (&speed)[3], double angles[3], int leg, int speed_const)
{ // TODO ver se eu consigo usar isso aqui
    // Calcula as velocidades para cada joint para que acabem todas ao mesmo tempo, atingindo um movimento mais fluido
    Serial.print("LEG: ");
    Serial.println(leg);
    
    double angles_[3] = 
        {angles[0] * 180/PI,
         angles[1] * 180/PI,
         angles[2] * 180/PI};
    
    Serial.print("angles[0]: ");
    Serial.print(angles_[0]);
    Serial.print("  angles[1]: ");
    Serial.print(angles_[1]);
    Serial.print("  angles[2]: ");
    Serial.println(angles_[2]);

    double s_estimate = Servo_List[leg * 3].GetPoseEstimate();
    double e_estimate = Servo_List[leg * 3 + 1].GetPoseEstimate();
    double w_estimate = Servo_List[leg * 3 + 2].GetPoseEstimate();

    Serial.print("s_estimate: ");
    Serial.print(s_estimate);
    Serial.print("  e_estimate: ");
    Serial.print(e_estimate);
    Serial.print("  w_estimate: ");
    Serial.println(w_estimate);
    
    // double shoulder_dist = abs(angles[0] - Servo_List[leg * 3].GetPoseEstimate());
    // double elbow_dist    = abs(angles[1] - Servo_List[leg * 3 + 1].GetPoseEstimate());
    // double wrist_dist    = abs(angles[2] - Servo_List[leg * 3 + 2].GetPoseEstimate());

    
    double shoulder_dist = abs(angles_[0] - s_estimate);
    double elbow_dist    = abs(angles_[1] - e_estimate);
    double wrist_dist    = abs(angles_[2] - w_estimate);

    Serial.print("shoulder_dist: ");
    Serial.print(shoulder_dist);
    Serial.print("  elbow_dist: ");
    Serial.print(elbow_dist);
    Serial.print("  wrist_dist: ");
    Serial.println(wrist_dist);

    double scaling_factor = max(shoulder_dist, elbow_dist, wrist_dist);

    double dt_movement = scaling_factor / (speed_const * 0.087912);

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

    Serial.print("s_speed: ");
    Serial.print(s_speed);
    Serial.print("  e_speed: ");
    Serial.print(e_speed);
    Serial.print("  w_speed: ");
    Serial.println(w_speed);

    return dt_movement;
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