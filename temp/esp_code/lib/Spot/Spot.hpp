#ifndef SPOT_INCLUDE_GUARD_HPP
#define SPOT_INCLUDE_GUARD_HPP

#include <Arduino.h>
#include "STSCTRL.h"
#include "SpotServo.hpp"
#include "config.h"

#define N_SERVOS 12

enum ServoIndex {
    FL_SHOULDER, FL_ELBOW, FL_WRIST,
    FR_SHOULDER, FR_ELBOW, FR_WRIST,
    RL_SHOULDER, RL_ELBOW, RL_WRIST,
    RR_SHOULDER, RR_ELBOW, RR_WRIST
};

class Spot
{
public:
    Spot();
    void Init();
    void Init_Servos();
    // void Initialize(SpotServo *Servo_List[], int nServos);
    void Update_Spot(int ACC);
    double getLoads();
    void getLoadString(char (& LoadString)[256],long time);
    void getPositionString(char(& PosString)[256],long time);
    bool all_goals_reached();

    void set_stance_wspeed(const double &l_shoulder_stance, const double &l_elbow_stance, const double &l_wrist_stance,
                       const double &r_shoulder_stance, const double &r_elbow_stance, const double &r_wrist_stance, double &speed);
    void straight_calibration_stance();
    void prone_calibration_stance();


    SpotServo Servo_List[N_SERVOS];
    
private:
    u8 ID[N_SERVOS];
};


#endif