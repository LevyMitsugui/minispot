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
    // void Init_Servos();
    // void Initialize(SpotServo *Servo_List[], int nServos);
    void Update_Spot(int ACC);
    double getLoads();
    void getLoadString(char (& LoadString)[256],long time);
    void getPositionString(char(& PosString)[256],long time);
    bool all_goals_reached();

    SpotServo Servo_List[N_SERVOS] = {
        SpotServo(1, 0, 0, 0, FL, Shoulder),
        SpotServo(2, 0, 0, 0, FL, Elbow),
        SpotServo(3, 0, 0, 0, FL, Wrist),
        SpotServo(4, 0, 0, 0, FR, Shoulder),
        SpotServo(5, 0, 0, 0, FR, Elbow),
        SpotServo(6, 0, 0, 0, FR, Wrist),
        SpotServo(7, 0, 0, 0, RL, Shoulder),
        SpotServo(8, 0, 0, 0, RL, Elbow),
        SpotServo(9, 0, 0, 0, RL, Wrist),
        SpotServo(10, 0, 0, 0, RR, Shoulder),
        SpotServo(11, 0, 0, 0, RR, Elbow),
        SpotServo(12, 0, 0, 0, RR, Wrist)
    };
    
private:
    u8 ID[N_SERVOS];
};


#endif