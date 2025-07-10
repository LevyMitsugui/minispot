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
#define GAIT_PATH_FRAMES 16 // (HAS TO BE EVEN) -> 9 for stance and 9 for swing

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

enum GaitState {        // Pair 1 = FL + RR, Pair 2 = FR + RL
    IDLE_GAIT,          // Idle
    STARTER_SW1_ST2,    // swing pair 1, stance pair 2 (starter because pair 2 does not have to plan swing phase)
    ST1_SW2,            // stance pair 1, swing pair 2
    SW1_ST2             // swing pair 1, stance pair 2
};

class fsm_t{
    public:
	int state, new_state;
    bool stateChanged;

	// tes - time entering state
	// tis - time in state
	double tes, tis;
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
    Eigen::Vector3d getEstimatedFootPosition(int leg);

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

    // Old Dynamic Gait
    void bezierControlPoints(
        Eigen::Vector3d (&points)[BEZIER_CONTROL_POINTS],
        Eigen::Vector3d pattern[BEZIER_CONTROL_POINTS],
        Eigen::Vector3d startingPos,
        Eigen::Vector3d stancePoints);

    Eigen::Vector3d bezier(double t,
                           Eigen::Vector3d p0,
                           Eigen::Vector3d p1,
                           Eigen::Vector3d p2,
                           Eigen::Vector3d p3   );

    void getStancePoints(int leg, Eigen::Vector3d &stancePoints, Eigen::Vector3d &stanceVelocities);

    bool performDynamicGait(double &currTimeMillis, Eigen::Vector3d Velocities[NUM_LEGS]);
    bool performDynamicGait2(double &currTimeMillis, Eigen::Vector3d Velocities[NUM_LEGS]);
    bool performStep(int Leg, double &currTimeMillis);
    int getStepState(int Leg);
    bool isStepDone(int Leg);
    bool isStepDone(int Leg, int Leg2);
    void setPeriods(double totalPeriod, double stancePeriod, double swingPeriod);
    void setStanceVelocity(int Leg, Eigen::Vector3d stanceVelocities);
    void setGaitState(int state, double &currTimeMillis);

    // New Dynamic Gait
    /**
     * @brief generatePath generates the path for the gait. It uses two cubic bezier curves
     * to generate the path points. One bezier for the swing and another for the stance.
     * (Note: cubic bezier curves are generated using FOUR control points)
     * 
     * @param @p outputPathPoints the output path points
     * @param @p swingCtrlPoints the bezier control points for the swing
     * @param @p stanceCtrlPoints the bezier control points for the stance
     */
    void generatePath(
        Eigen::Vector3d (&outputPathPoints)[GAIT_PATH_FRAMES], 
        Eigen::Vector3d swingCtrlPoints[BEZIER_CONTROL_POINTS], 
        Eigen::Vector3d stanceCtrlPoints[BEZIER_CONTROL_POINTS]);

    Eigen::Vector3d rotateZToAlign(Eigen::Vector3d& V, Eigen::Vector3d& vReference);

    /**
     * @brief getNewPathPoint returns the path point for this @p frame for this @p leg based on .
     * admits the velocities are already updated and stored in stanceVelocities.
     * It automatically move the points to the reference midpoint of a leg 
     * (defined as the starting position of the leg), can be applied any offset with 
     * the @p midPointOffset parameter
     * 
     * @param @p leg the leg index
     * @param @p frame the frame index
     * @param @p stepHeight the step height
     * @param @p midPointOffset the offset to apply to the path point
     */
   Eigen::Vector3d getNewPathPoint(int leg, int frame, double stepHeight, Eigen::Vector3d midPointOffset);

    // - - - - - - - - - - - - -

    // - - - Testing Space - - -
    void printPathPoints(int leg);
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
    
    Eigen::Vector3d feetVelocities[NUM_LEGS]; // TODO maybe will not be used
    Eigen::Vector3d gaitPathPoints[GAIT_PATH_FRAMES];
    int frameIter[NUM_LEGS];
    double periodUpdate;
    double prevTime;

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
    double gaitStartTime[NUM_LEGS];       // ms
    double gaitPrevTime[NUM_LEGS];        // ms
    double gaitCurrTime[NUM_LEGS];        // ms

    StepState stepState[NUM_LEGS];
    fsm_t gaitState;

    Eigen::Vector3d bezPoint[NUM_LEGS];
    Eigen::Vector3d bezControlPoints[NUM_LEGS][BEZIER_CONTROL_POINTS];

    Eigen::Vector3d legStartingPos[NUM_LEGS];
    Eigen::Vector3d stancePoints[NUM_LEGS];
    Eigen::Vector3d stanceVelocities[NUM_LEGS];

    u8 gaitState_testing[NUM_LEGS];

    bool setState(fsm_t &stateMachine, double &currTimeMillis);
    void updateStateTime(fsm_t &stateMachine, double &currTimeMillis);
    void updateVelocities(int pair, Eigen::Vector3d Velocities[NUM_LEGS]);
    double reframeAngle(double angle);

};


#endif