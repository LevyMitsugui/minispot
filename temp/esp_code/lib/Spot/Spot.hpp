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

#define BEZIER_CONTROL_POINTS 4 // cubic bezier curve

enum ServoIndex {
    FL_SHOULDER, FL_ELBOW, FL_WRIST, // leg 0
    FR_SHOULDER, FR_ELBOW, FR_WRIST, // leg 1
    RL_SHOULDER, RL_ELBOW, RL_WRIST, // leg 2
    RR_SHOULDER, RR_ELBOW, RR_WRIST  // leg 3
};

enum StepState {
    IDLE_STEP,
    SWING,
    STANCE,
    END_STEP
};

class Spot
{
public:
    Spot();
    void Init();
    void Init_Servos();

    void Update_Spot(int ACC);
    bool all_goals_reached();
    
    double getLoads();
    void getLoadString(char (& LoadString)[256],long time);
    void getPositionString(char(& PosString)[256],long time);
    Eigen::Vector3d getFootPosition(int Leg);
    Eigen::Vector3d getRealFootPosition(int leg);

    void perform_gait_singular(int leg, int nFrames, float (*posFrames)[3], double timeInterval_us);
    void perform_gait(int nFrames, float (*posFrames)[19][3], double timeInterval_us, int cycles);
    void perform_gait_no_blocking(float (*posFrames)[19][3], bool is_first_frame);

    double Leg_Joint_Speeds(double (& speed) [3],double angles[3],int leg, int speed_const);
    double Leg_Joint_Speeds_2(double (&speed)[3], double angles[3], int leg, double max_speed);

    double move_feet(Eigen::Vector3d vectors[NUM_LEGS], double max_speed);
    double move_foot(int leg, Eigen::Vector3d vector, double max_speed);
    double move_foot(int leg, Eigen::Vector3d vector, double max_speed, bool noUpdate);

    void translate(double x, double y, double z);
    void rotate(double row, double pitch, double yaw);

    void set_lean(double x, double y, double z);
    void set_rpy(double row, double pitch, double yaw);
    double pose();
    bool touch_ground(int leg);

    void set_stance_wspeed(const double &l_shoulder_stance, const double &l_elbow_stance, const double &l_wrist_stance,
                       const double &r_shoulder_stance, const double &r_elbow_stance, const double &r_wrist_stance, double &speed);
    void straight_calibration_stance();
    void prone_calibration_stance();

    Eigen::Vector3d bezier(double t,
                           Eigen::Vector3d p0,
                           Eigen::Vector3d p1,
                           Eigen::Vector3d p2,
                           Eigen::Vector3d p3   );

    void getStancePoints(int leg, Eigen::Vector3d &stancePoints, Eigen::Vector3d &stanceVelocities);

    bool performStep(int Leg, double &currTimeMillis);
    int getStepState(int Leg);
    bool stepDone(int Leg);
    void setPeriods(double totalPeriod, double stancePeriod, double swingPeriod);
    void setStanceVelocity(int Leg, Eigen::Vector3d stanceVelocities);

    // - - - Testing Space - - -
    bool performStancePhase();
    bool testGaitGen();
    // - - - - - - - - - - - - -

    SpotServo Servo_List[N_SERVOS];
    double joint_angles[NUM_LEGS][NUM_JOINTS];

    Eigen::Vector3d torsoRPY;
    Eigen::Vector3d torsoRPYTarget;
    Eigen::Vector3d torsoLean;
    Eigen::Vector3d torsoLeanTarget;
    Eigen::Matrix4d TorsoToFoot[NUM_LEGS]; // TODO maybe this should be exclusive for model class

    SpotModel model;
    double timeHelper[4];  // To be used inside functions that require time memory between cycles
    

private:
    double max(double a0, double a1, double a2);

    int frame;
    // double timeHelper[4];  // To be used inside functions that require time memory between cycles
    
    // int frame_forward, frame_backward, // TODO remove this
    //     frame_right, frame_left,
    //     frame_rotate_right, frame_rotate_left;

    u8 ID[N_SERVOS];
    //SpotModel model;

    double gaitT;               // seconds
    double gaitTst;             // secondsd
    double gaitTsw;             // seconds
    double gaitProcessPeriod;      // ms
    double gaitStartTime[2];       // ms
    double gaitPrevTime[2];        // ms
    double gaitCurrTime[2];        // ms

    StepState stepState[NUM_LEGS];

    Eigen::Vector3d bezPoint[NUM_LEGS];
    Eigen::Vector3d bezControlPoints[BEZIER_CONTROL_POINTS];

    Eigen::Vector3d legStartingPos[NUM_LEGS];
    Eigen::Vector3d stancePoints[NUM_LEGS];
    Eigen::Vector3d stanceVelocities[NUM_LEGS];

    u8 gaitState[NUM_LEGS];

};


#endif