#include "SpotServo.hpp"
#include <SCServo.h>
#include "STSCTRL.h"
#include "Spot.hpp"
#include <algorithm>
using namespace std;


Spot::Spot(){
    for (int i = 0; i < N_SERVOS; i++){
        ID[i] = Servo_List[i].Get_servo_ID();
    }
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
    double Load= 0;
    for (int i = 0; i < N_SERVOS; i++)
    {
        Load = Servo_List[i].getLoad();
        Loads = Loads + abs(Load)/1000;        
    }
    return Loads;
}
void Spot::getPositionString(char(& PosString)[256],long time)
{
    memset(PosString, 0, 256);
    double Pos= 0;
    int offset = 0;
    offset += sprintf(PosString, "Q, %ld", time);
    for (int i = 0; i < N_SERVOS; i++)
    {
        Pos = Servo_List[i].GetPoseEstimate();
        offset += sprintf(PosString + offset, ", %.1f", Pos);
        if (offset >= 256) {
            break; // Prevent buffer overflow
        }
    }
}

void Spot::getLoadString(char (& LoadString)[256], long time) //MAX_BUFFER_LEN = 256
{
    memset(LoadString, 0, 256);
    double Load= 0;
    double speed= 0;
    int offset = 0;
    offset += sprintf(LoadString, "Q, %ld", time);
    for (int i = 0; i < N_SERVOS; i++)
    {
        Load = Servo_List[i].getLoad();
        speed = Servo_List[i].Get_Speed();
        offset += sprintf(LoadString + offset, ", %.0f", Load);
        offset += sprintf(LoadString + offset, ", %.0f", speed);
        if (offset >= 256) {
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

// double //TODO add speed control
//TODO add inverse kinematics 