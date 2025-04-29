#include "SpotServo.hpp"
#include <SCServo.h>
#include "STSCTRL.h"
#include "Spot.hpp"
#include <algorithm>
using namespace std;


void Spot::Initialize(SpotServo *Servo_List_[], int nServos_)
{
    nServos = nServos_;
    for (int i = 0; i < nServos; i++)
    {
        // Find the correct position to insert the new servo
        int j = i;
        while (j > 0 && Servo_List_[i]->Get_servo_ID() < Servo_List[j - 1]->Get_servo_ID())
        {
            Servo_List[j] = Servo_List[j - 1];
            j--;
        }
        // Insert the new servo at the correct position
        Servo_List[j] = Servo_List_[i];
        ID[j] = Servo_List[j]->Get_servo_ID();
    }
}

void Spot::Update_Spot(int ACC)
{
    s16 Goal_Pos_list[nServos];
    u16 Speed_list[nServos];
    u8 ACC_list[nServos];

    for (int i = 0; i < nServos; i++)
    {
        Goal_Pos_list[i] = Servo_List[i]->Get_Goal_Pos_w_Offset();
        Speed_list[i] = Servo_List[i]->Get_Speed();
        ACC_list[i] = ACC;
    }
    st.SyncWritePosEx(ID, nServos, Goal_Pos_list, Speed_list, ACC_list);
}

double Spot::getLoads()
{
    double Loads = 0;
    double Load= 0;
    for (int i = 0; i < nServos; i++)
    {
        Load = Servo_List[i]->getLoad();
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
    for (int i = 0; i < nServos; i++)
    {
        Pos = Servo_List[i]->GetPoseEstimate();
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
    for (int i = 0; i < nServos; i++)
    {
        Load = Servo_List[i]->getLoad();
        speed = Servo_List[i]->Get_Speed();
        offset += sprintf(LoadString + offset, ", %.0f", Load);
        offset += sprintf(LoadString + offset, ", %.0f", speed);
        if (offset >= 256) {
            break; // Prevent buffer overflow
        }
    }
}
