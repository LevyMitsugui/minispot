#include "SpotServo.hpp"
#include <SCServo.h>
#include "STSCTRL.h"
#define ENABLE_DEBUG /* <-- Commenting this line will remove any trace of debug printing */
#include <MacroDebugger.h>
using namespace std;


// Spot Full Constructor
void SpotServo::Initialize(const int & servo_ID_, const double & stand_angle_, const double & home_angle_, const double & offset_, const LegType & leg_type_, const JointType & joint_type_)
{
	if (joint_type_ == Shoulder)
	{
		min_pos = 1690;
		max_pos = 2404;
		min_ang = -31.4;
		max_ang = 31.4;
	}
	else if (joint_type_ == Elbow)
	{
		if (leg_type_ == FL || leg_type_ == RL)
		{
			min_pos = 634;
			max_pos = 2679;
			min_ang = -124.3;
			max_ang = 55.6;
		}
		else if (leg_type_ == FR || leg_type_ == RR)
		{
			min_pos = 1415;
			max_pos = 3460;
			max_ang = 124.3;
			min_ang = -55.6;
		}
	}
	else if (joint_type_ == Wrist)
	{
		if (leg_type_ == FL || leg_type_ == RL)
		{
			min_pos = 1981;
			max_pos = 3443;
			min_ang = -5.8;
			max_ang = 121;
		}
		else if (leg_type_ == FR || leg_type_ == RR) 
		{
			min_pos = 660;
			max_pos = 2113;
			min_ang = -121;
			max_ang = 5.8;
		}
	}
	servo_ID = servo_ID_;
	offset = offset_;
	home_angle = home_angle_;
	current_pose = home_angle + offset;
	leg_type = leg_type_;
	joint_type = joint_type;
	stand_angle = stand_angle_;
	goal_pose = stand_angle + offset;
	update_pose = stand_angle + offset;

	int pos = 2047 + ((stand_angle + offset))/ 0.087912;
	st.WritePosEx(servo_ID, pos, speed, acc); // 2047 is the center position of the servo
	last_actuated = millis();
}

void SpotServo::SetGoal(const double & goal_pose_, const double & desired_speed_)
{
	goal_pose = goal_pose_;
	goal_pose += offset;
	
	// cpp would be std::clamp() with include cmath
	if (goal_pose < min_ang)
	{
		goal_pose = min_ang;
	} else if (goal_pose > max_ang)
	{
		goal_pose = max_ang;
	}

	desired_speed = desired_speed_;
}

JointType SpotServo::return_joint_type()
{
	return joint_type;
}

LegType SpotServo::return_legtype()
{
	return leg_type;
}

int SpotServo::Get_servo_ID()
{
	return servo_ID;
}
double SpotServo::Get_Goal()
{
	return goal_pose;
}
double SpotServo::Get_Goal_Pos_w_Offset()
{
	return 2047 + ((goal_pose + offset) / 0.087912);
}

double SpotServo::Get_Speed()
{
	return desired_speed;
}

double SpotServo::return_home()
{
	return home_angle;
}

double SpotServo::GetPoseEstimate()
{
	update_position_using_Feedback();
	return current_pose; //da a posicao que o servo acha que esta
}

bool SpotServo::GoalReached()
{
	update_position_using_Feedback();
	return (abs(current_pose - goal_pose) <= error_threshold);
}

void SpotServo::update_clk()
{
	//Atualiza a posicao do servo se estiver dentro do threshold e se o tempo de espera foi atingido

	// Only perform update if loop rate is met
	if(millis() - last_actuated > wait_time)
	{
		// Only update position if not within threshold
		if(abs(update_pose - goal_pose) > error_threshold)
		{
			// returns 1.0 * sign of goal_pose - current_pose

			int pos = 2047 + ((goal_pose + offset) / 0.087912);

			if (pos<min_pos)
			{
				pos = min_pos;
			}
			else if (pos>max_pos)
			{
				pos = max_pos;
			}
			update_pose = goal_pose; //nao sei se devo fazer isto porque estou a dar logo update a current pose
			last_actuated = millis();
		}
	}

}

void SpotServo::Get_Feedback(float (&speed),float (&load),float (&position))
{
	getFeedBack(servo_ID);
	speed = 0; //((speedRead[servo_ID]*0.087912)*3.141)/180;
	load = (loadRead[servo_ID]*30)/1000;
	position = posRead[servo_ID];
}

double PosToDeg(int pos) {
  return ((pos-2047) * 0.087912);
}


void SpotServo::update_position_using_Feedback()
{
	getPositionFeedback(servo_ID);
	current_pose = PosToDeg(posRead[servo_ID]) - offset;
	//DEBUG_I("posRead: %d\n", posRead[servo_ID]);
	//DEBUG_I("Current Pose: %f\n", current_pose);
}

double SpotServo::getLoad()
{
	getLoadFeedback(servo_ID);
	return loadRead[servo_ID];

}
void SpotServo::detach()
{
	//servo.detach();
}