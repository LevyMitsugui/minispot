#ifndef SPOT_INCLUDE_GUARD_HPP
#define SPOT_INCLUDE_GUARD_HPP

#include <Arduino.h>
#include "STSCTRL.h"
#include "SpotServo.hpp"
#include "config.h"


class Spot
{
public:
    void Initialize(SpotServo *Servo_List[], int nServos);
    void Update_Spot(int ACC);
    double getLoads();
    void getLoadString(char (& LoadString)[256],long time);
    void getPositionString(char(& PosString)[256],long time);

private:
    SpotServo *Servo_List[12];
    int nServos=0;
    u8 ID[12];
};


#endif