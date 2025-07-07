#ifndef SPOTSERVO_INCLUDE_GUARD_HPP
#define SPOTSERVO_INCLUDE_GUARD_HPP
/// \file
/// \brief Servo Speed Control Library. Adapted from https://github.com/adham-elarabawy/OpenQuadruped
#include <Arduino.h>
#include "STSCTRL.h"
// #include <cmath>

#define ENCODER_MID 2047 // Encoder middle value, 0 degrees, 0 radians (encoder resolution is 2^12 = 4096)
#define DEG_RATIO 0.087912 // One encoder step is 0.087912 degrees (4096/2/180 = 2048/180 = 0.087912)
#define RAD_RATIO 0.0015340 // One encoder step is 0.0015340 radians (4096/2/180*3.14159 = 2048/180*3.14159 = 0.0015340)

//#define RAD_TO_STEPS_SPEED 652.27436 // from the manual, 50 steps/s = 0.732 rpm = 0.0766549 rad/s. ratio = 50*60/(2*pi*0.732) steps/rad
#define RAD_TO_STEPS_SPEED 735.29412 // I did some measurements and experiments to get this value, 1500 steps/s more equivalent to 2.04 rad/s

#define MAX_SPEED 3073 // steps/s
#define MAX_SPEED_RAD MAX_SPEED/RAD_TO_STEPS_SPEED

enum LegType {FL, FR, RL, RR};
enum JointType {Shoulder, Elbow, Wrist};

/// \brief SpotServo class responsible servo control

double PosToDeg(int pos);
class SpotServo
{

public:
    // using default constructor
    SpotServo();

    /// \brief Initialize parameters
    /// \param servo_ID: servo id
    /// \param home_angle_: default joint angle
    /// \param offset_: motor position offset (due to mechanical fit issues)
    /// \param leg_type_: Front Left, Front Right, Back Left, or Back Right leg (see enum)
    /// \param joint_type_: Shoulder, Elbow or Wrist (see enum)
    SpotServo(const int & servo_ID_, const double & stand_angle_, const double & home_angle_, const double & offset_, const LegType & leg_type_, const JointType & joint_type_, bool init);

    void Init();
    
    /// \brief Initialize parameters
    /// \param servo_ID: servo id
    /// \param home_angle_: default joint angle
    /// \param offset_: motor position offset (due to mechanical fit issues)
    /// \param leg_type_: Front Left, Front Right, Back Left, or Back Right leg (see enum)
    /// \param joint_type_: Shoulder, Elbow or Wrist (see enum)
    void Initialize(const int & servo_ID, const double & stand_angle_, const double & home_angle_, const double & offset_, const LegType & leg_type_, const JointType & joint_type_);

    /// \brief Commands a motor to move to a certain goal at a certain speed
    /// \param goal_pose_: the desired motor position in degrees
    /// \param desired_speed_: the desired motor speed (deg/sec) while reaching this goal
    void SetGoal(const double & goal_pose_, const double & desired_speed_);

    /// \brief returns joint_type
    /// \returns: joint_type
    JointType return_joint_type();

    /// \brief returns leg_type
    /// \returns: leg_type
    LegType return_legtype();

    /// \brief returns this servo's home angle
    /// \returns: home_angle
    double return_home();

    /// \brief Return the interpolated position (time-step is small enough that this is mostly accurate)
    /// \returns: motor_pose_est
    double GetPoseEstimate();

    double GetPoseEstimateRad();

    /// \brief Return the servo ID
    /// \returns: servo_ID
    int Get_servo_ID();

    double Get_Speed();

    /// \brief Perform one motor update, potentially sending a new motor command, and updating the time-step.
    void update_clk();

    /// \brief Update the position estimate using feedback
    void update_position_using_Feedback();

    /// \brief Detach the motor
    void detach();

    /// \brief Returns true if the motor has reached its goal
    bool GoalReached();

    /// \brief Returns the current goal
    double Get_Goal();

    /// \brief Returns the current goal Position with offset
    double Get_Goal_Pos_w_Offset();

    double getLoad();

    void Get_Feedback(float (&speed),float (&load),float (&position));

    bool init_type = false;

private:
    // Intrinsic Parameters
    // time elapsed since last servo update
    double last_actuated = 0.0;
    // error threshold for servo position
    double error_threshold = 1;
    // maximum servo angle (minimum is 0)
    double control_range = 360.0; //vai de -180 a 180
    // loop period (milisec)
    double wait_time = 1.0;
    // motor position offset (due to mechanical fit issues)
    int offset = 0;
    double offset_rad = 0;

    int speed =600;
    int acc = 50;
    // Leg Type (see enum)
    LegType leg_type;
    // Joint Type (see enum)
    JointType joint_type;
    // default angle
    double home_angle = 0.0;
    double stand_angle = 0.0;

    int servo_ID = 0;

    // Changeable Parameters
    double goal_pose = 0.0; // deg
    double update_pose = 0.0; // deg this pose is used to not overload the servos with commands, it represents the pose the servo is moving towards.
    double current_pose = 0.0; // deg
    double desired_speed = 0.0; //[edit 2025: motors work with steps/s. this variable is in steps/s]

    double current_pose_rad = 0.0; //rad

    // Servo's PWM range (usec)
    int min_pos = 0;
    int max_pos = 4095;

    double min_ang = 0.0; //-180
    double max_ang = 0.0; //180

    // Interpolation to convert from deg to usec
    double conv_slope = 0.0;
    double conv_intcpt = 0.0;
};
    

#endif