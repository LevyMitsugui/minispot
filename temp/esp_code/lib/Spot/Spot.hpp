#ifndef SPOT_INCLUDE_GUARD_HPP
#define SPOT_INCLUDE_GUARD_HPP

#include <Arduino.h>
#include "STSCTRL.h"
#include "SpotServo.hpp"
#include "config.h"
#include <ArduinoEigen.h>
#include "SpotModel.hpp"

#define N_SERVOS 12
#define POS_ERROR_THRESHOLD 0.1 // degrees

enum ServoIndex {
    FL_SHOULDER, FL_ELBOW, FL_WRIST, // leg 0
    FR_SHOULDER, FR_ELBOW, FR_WRIST, // leg 1
    RL_SHOULDER, RL_ELBOW, RL_WRIST, // leg 2
    RR_SHOULDER, RR_ELBOW, RR_WRIST  // leg 3
};

class Spot
{
public:
    Spot();
    void Init();
    void Init_Servos();
    void Update_Spot(int ACC);
    double getLoads();
    void getLoadString(char (& LoadString)[256],long time);
    void getPositionString(char(& PosString)[256],long time);
    
    bool all_goals_reached();
    void getFootPosition(int leg, Eigen::Vector3d &footPosition);

    void perform_gait_singular(int leg, int nFrames, float (*posFrames)[3], double timeInterval_us);
    void perform_gait(int nFrames, float (*posFrames)[19][3], double timeInterval_us, int cycles);

    double Leg_Joint_Speeds(double (& speed) [3],double angles[3],int leg, int speed_const);
    double Leg_Joint_Speeds_2(double (&speed)[3], double angles[3], int leg, double max_speed);

    void move_feet(Eigen::Vector3d vectors[NUM_LEGS]);
    void move_foot(int leg, Eigen::Vector3d vector);
    void rotate(double row, double pitch, double yaw);
    void pose(Eigen::Vector3d orientation, Eigen::Vector3d position);
    bool touch_ground(int leg);

    void set_stance_wspeed(const double &l_shoulder_stance, const double &l_elbow_stance, const double &l_wrist_stance,
                       const double &r_shoulder_stance, const double &r_elbow_stance, const double &r_wrist_stance, double &speed);
    void straight_calibration_stance();
    void prone_calibration_stance();


    SpotServo Servo_List[N_SERVOS];
    double Mem_Agnles[N_SERVOS];

    
    double joint_angles[NUM_LEGS][NUM_JOINTS];
    Eigen::Vector3d torsoOrientationRPY;
    Eigen::Vector3d torsoPosition;
    Eigen::Matrix4d TorsoToFoot[NUM_LEGS]; // TODO maybe this should be exclusive for model class


    SpotModel model;

private:
    double max(double a0, double a1, double a2);

    u8 ID[N_SERVOS];
    //SpotModel model;
};


#endif