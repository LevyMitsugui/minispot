// SpotModel.cpp
#include "SpotModel.hpp"
#include <vector>

#define ENABLE_DEBUG
#include <MacroDebugger.h>

#define VALIDATE_MODEL

#define DEBUG_PRINT_MATRIX4D(label, mat)                 \
    do {                                                 \
        DEBUG_I("%s", label);                            \
        for (int _i = 0; _i < 4; ++_i) {                  \
            DEBUG_I("[% .3f % .3f % .3f % .3f]",          \
                mat(_i, 0), mat(_i, 1), mat(_i, 2), mat(_i, 3)); \
        }                                                \
    } while (0)

SpotModel::SpotModel(double shoulder_length,
                     double elbow_length,
                     double wrist_length,
                     double hip_x,
                     double hip_y,
                     double foot_x,
                     double foot_y,
                     double height)
    : shoulder_length(shoulder_length),
      elbow_length(elbow_length),
      wrist_length(wrist_length),
      hip_x(hip_x),
      hip_y(hip_y),
      foot_x(foot_x),
      foot_y(foot_y),
      height(height) {

    Legs[FL_m].Initialize(shoulder_length, elbow_length, wrist_length);
    Legs[FR_m].Initialize(shoulder_length, elbow_length, wrist_length);
    Legs[RL_m].Initialize(shoulder_length, elbow_length, wrist_length);
    Legs[RR_m].Initialize(shoulder_length, elbow_length, wrist_length);

    Eigen::Matrix3d Rwb = Eigen::Matrix3d::Identity();

    // Homogenous transforms from body to hip
    T_bh[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0,  hip_y / 2.0, 0));
    T_bh[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0, -hip_y / 2.0, 0));
    T_bh[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0,  hip_y / 2.0, 0));
    T_bh[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0, -hip_y / 2.0, 0));

    // Homogenous transforms from body to foot
    T_bf[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, ( foot_y / 2.0), -height));
    T_bf[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, (-foot_y / 2.0), -height));
    T_bf[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, ( foot_y / 2.0), -height));
    T_bf[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, (-foot_y / 2.0), -height));

    // TorsoToFoot[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, ( foot_y / 2.0) + shoulder_length, -height));
    // TorsoToFoot[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, (-foot_y / 2.0) - shoulder_length, -height));
    // TorsoToFoot[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, ( foot_y / 2.0) + shoulder_length, -height));
    // TorsoToFoot[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, (-foot_y / 2.0) - shoulder_length, -height));
}

/**
 * @brief HipToFeet Calculate the vector from the hip to the foot for each leg.
 *
 * @param hipToFootVectors The output array of vectors from the hip to the foot.
 * @param bodyOrientationRPY The orientation of the robot body in terms of roll, pitch, and yaw.
 * @param bodyPosition The position of the robot body.
 * @param T_bf The transformation matrices from the body frame to each foot frame.
 */
void SpotModel::HipToFeet(
    Eigen::Vector3d hipToFootVectors[NUM_LEGS],
    const Eigen::Vector3d& bodyOrientationRPY,
    const Eigen::Vector3d& bodyPosition,
    const Eigen::Matrix4d T_bf[NUM_LEGS])
{
    Eigen::Matrix4d T_wb = LieAlgebra::RpToTrans(
        LieAlgebra::RPY(bodyOrientationRPY[0], 
                        bodyOrientationRPY[1], 
                        bodyOrientationRPY[2]).block<3, 3>(0, 0),
        bodyPosition);

    for (int i = 0; i < NUM_LEGS; ++i) {
        Eigen::Vector3d footPosInBody = T_bf[i].block<3, 1>(0, 3); // foot position in torso frame
        
        // Transform from world to hip  = transform from virtual world to torso * Trasform from torso to a hip
        Eigen::Matrix4d T_wh = LieAlgebra::TransInv(T_wb) * T_bh[i];
        
        
        Eigen::Vector3d hipPosInWorld = T_wh.block<3, 1>(0, 3);

        Eigen::Matrix4d T_hf = LieAlgebra::TransInv(T_wh) * T_bf[i];
        Eigen::Vector3d footPosInHip = T_hf.block<3, 1>(0, 3);

        hipToFootVectors[i] = footPosInHip;
    }
}

void SpotModel::HipToBodyV(int leg, Eigen::Vector3d p_hf, Eigen::Vector3d& p_bf)
{
    Eigen::Vector4d v_v = T_bh[leg] * Eigen::Vector4d(p_hf.x(), p_hf.y(), p_hf.z(), 1);
    p_bf = Eigen::Vector3d(v_v.x(), v_v.y(), v_v.z());
}

/**
 * @brief IK Calculate the inverse kinematics for each leg given the body orientation and position.
 *
 * @param jointAngles The output array of joint angles for each leg.
 * @param bodyOrientationRPY The orientation of the robot body in terms of roll, pitch, and yaw, in radians.
 * @param bodyPosition The position of the robot body.
 */
void SpotModel::IK( // TODO it does not look like it is updating the values in memory correctly.
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyOrientationRPY,
    const Eigen::Vector3d& bodyPosition)
{
    Eigen::Vector3d hipToFootVectors[NUM_LEGS];
    HipToFeet(hipToFootVectors, bodyOrientationRPY, bodyPosition, T_bf);

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        double jointAngleSet[NUM_JOINTS];
        Eigen::Vector3d hipToFoot = hipToFootVectors[legIdx];
        LegQuadrant side = (legIdx == FR_m || legIdx == RR_m) ? Right : Left;

        Legs[legIdx].GetJointAngles(hipToFoot[0], hipToFoot[1], hipToFoot[2], side, jointAngleSet);

        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
            jointAngles[legIdx][jointIdx] = jointAngleSet[jointIdx];
            
        }
    }

    #ifdef VALIDATE_MODEL
        Eigen::Vector3d footPositions;
        for(int legIdx = 0; legIdx < NUM_LEGS; ++legIdx){
            FK_singular(jointAngles[legIdx], legIdx, footPositions);
            DEBUG_I("X: %f : %f  Y: %f : %f   Z: %f : %f", 
            hipToFootVectors[legIdx][0], footPositions[0], 
            hipToFootVectors[legIdx][1], footPositions[1], 
            hipToFootVectors[legIdx][2], footPositions[2]);
        }
    #endif
}

void SpotModel::translateBody(
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyPosition)
{

}

/**
 * @brief IK_singular Calculate the inverse kinematics for a single
 *        leg given a new foot position relative to the robot's torso.
 *
 * @param legJointAngles The output array of joint angles for the leg.
 * @param newFootPos The new position of the foot relative to the robot torso.
 * @param leg The leg for which to calculate the inverse kinematics.
 */
void SpotModel::IK_singular(
    double legJointAngles[NUM_JOINTS],
    Eigen::Vector3d newFootPos,         // new foot pos relative to robot torso. 
    int leg)
{
    T_bf[leg].block<3,1>(0, 3) = newFootPos; // update foot pos in memory of the class

    // NOTE: here we could have used just 3d vectors and the computation would be a simple subtraction, but keep track of rotations could be done in the future.
    Eigen::Matrix4d T_hf = LieAlgebra::TransInv(T_bh[leg]) * T_bf[leg]; // from foot pos in torso frame to transform of hip to foot
    Eigen::Vector3d footPosInHipFrame = T_hf.block<3,1>(0,3); // extract the pos of the foot in the hip frame

    if      (footPosInHipFrame.x() < -0.08 ) footPosInHipFrame.x() = -0.08;
    else if (footPosInHipFrame.x() >  0.065 ) footPosInHipFrame.x() =  0.065;
    if(leg == FL_m || leg == RL_m){
        if      (footPosInHipFrame.y() < -0.02) footPosInHipFrame.y() = -0.02;
        else if (footPosInHipFrame.y() >  0.15 ) footPosInHipFrame.y() =  0.15;}
    else{
        if      (footPosInHipFrame.y() < -0.15) footPosInHipFrame.y() = -0.15;
        else if (footPosInHipFrame.y() >  0.02) footPosInHipFrame.y() =  0.02;}
    if      (footPosInHipFrame.z() < -0.18 ) footPosInHipFrame.z() = -0.18;
    else if (footPosInHipFrame.z() > -0.02 ) footPosInHipFrame.z() = -0.02;

    LegQuadrant side = (leg == FR_m || leg == RR_m) ? Right : Left;
    
    double angles[NUM_JOINTS];
    Legs[leg].GetJointAngles(footPosInHipFrame[0], footPosInHipFrame[1], footPosInHipFrame[2], side, angles);

    for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
        legJointAngles[jointIdx] = angles[jointIdx];
    }
}

void SpotModel::FK_singular(
        double legJointAngles[NUM_JOINTS],
        int leg, 
        Eigen::Vector3d& footPosition)
{
    const double angles[NUM_JOINTS] = {
        legJointAngles[0],
        legJointAngles[1],
        legJointAngles[2]
        };
    double xyz[NUM_JOINTS];
    LegQuadrant side = (leg == FR_m || leg == RR_m) ? Right : Left;
    Legs[leg].ForwardKinematics(angles, side, xyz);

    footPosition[0] = xyz[0];
    footPosition[1] = xyz[1];
    footPosition[2] = xyz[2];
}

// make a hip to foot that atualizes the transforms

void SpotModel::ComputeJointAnglesFromTranslation(
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyPosition)
{
    DEBUG_I("bodyPosition: %f %f %f", bodyPosition[0], bodyPosition[1], bodyPosition[2]);
    
    Eigen::Matrix4d T_wb = LieAlgebra::RpToTrans(Eigen::Matrix3d::Identity(), bodyPosition);

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        Eigen::Matrix4d T_wh = T_wb * T_bh[legIdx];

        Eigen::Matrix4d T_hf = LieAlgebra::TransInv(T_wh) * T_bf[legIdx];

        Eigen::Vector3d footPosInHip = T_hf.block<3, 1>(0, 3);


        if      (footPosInHip.x() < -0.08 )  footPosInHip.x() = -0.08;
        else if (footPosInHip.x() >  0.065 ) footPosInHip.x() =  0.065;
        if(legIdx == FL_m || legIdx == RL_m){
            if      (footPosInHip.y() < -0.02)  footPosInHip.y() = -0.02;
            else if (footPosInHip.y() >  0.15 ) footPosInHip.y() =  0.15;}
        else{
            if      (footPosInHip.y() < -0.15) footPosInHip.y() = -0.15;
            else if (footPosInHip.y() >  0.02) footPosInHip.y() =  0.02;}
        if      (footPosInHip.z() < -0.18 ) footPosInHip.z() = -0.18;
        else if (footPosInHip.z() > -0.02 ) footPosInHip.z() = -0.02;


        LegQuadrant side = (legIdx == FR_m || legIdx == RR_m) ? Right : Left;

        double jointAngleSet[NUM_JOINTS];
        Legs[legIdx].GetJointAngles(footPosInHip[0], footPosInHip[1], footPosInHip[2], side, jointAngleSet);

        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
            jointAngles[legIdx][jointIdx] = jointAngleSet[jointIdx];
        }
    }
}

void SpotModel::ComputeJointAnglesFromRotation(
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyOrientationRPY)
{
    // Create world-to-body transform with rotation only, no translation
    Eigen::Matrix3d R = LieAlgebra::RPY(
        bodyOrientationRPY[0],
        bodyOrientationRPY[1],
        bodyOrientationRPY[2]
    ).block<3, 3>(0, 0);

    Eigen::Matrix4d T_wb = LieAlgebra::RpToTrans(R, Eigen::Vector3d::Zero());

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        // Transform from world to hip: T_wh = T_wb⁻¹ * T_bh
        //Eigen::Matrix4d T_wh = LieAlgebra::TransInv(T_wb) * T_bh[legIdx];
        Eigen::Matrix4d T_wh = T_wb * T_bh[legIdx];

        // Transform from hip to foot: T_hf = T_wh⁻¹ * T_bf
        Eigen::Matrix4d T_hf = LieAlgebra::TransInv(T_wh) * T_bf[legIdx];

        // Extract foot position in hip frame
        Eigen::Vector3d footPosInHip = T_hf.block<3, 1>(0, 3);

        if      (footPosInHip.x() < -0.08 )  footPosInHip.x() = -0.08;
        else if (footPosInHip.x() >  0.065 ) footPosInHip.x() =  0.065;
        if(legIdx == FL_m || legIdx == RL_m){
            if      (footPosInHip.y() < -0.02)  footPosInHip.y() = -0.02;
            else if (footPosInHip.y() >  0.15 ) footPosInHip.y() =  0.15;}
        else{
            if      (footPosInHip.y() < -0.15) footPosInHip.y() = -0.15;
            else if (footPosInHip.y() >  0.02) footPosInHip.y() =  0.02;}
        if      (footPosInHip.z() < -0.18 ) footPosInHip.z() = -0.18;
        else if (footPosInHip.z() > -0.02 ) footPosInHip.z() = -0.02;

        // Compute side of the robot for IK
        LegQuadrant side = (legIdx == FR_m || legIdx == RR_m) ? Right : Left;

        // Compute joint angles
        double jointAngleSet[NUM_JOINTS];
        Legs[legIdx].GetJointAngles(footPosInHip[0], footPosInHip[1], footPosInHip[2], side, jointAngleSet);

        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
            jointAngles[legIdx][jointIdx] = jointAngleSet[jointIdx];
        }
    }
}



void SpotModel::rotateBody( // TODO continue here
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyOrientationRPY)
{   
    Eigen::Matrix4d T_hf_print[NUM_LEGS];
    for(int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        T_hf_print[legIdx] = LieAlgebra::TransInv(T_bh[legIdx]) * T_bf[legIdx];
    }
    DEBUG_PRINT_MATRIX4D("hip to foot 0", T_hf_print[FL_m]);
    DEBUG_PRINT_MATRIX4D("hip to foot 1", T_hf_print[FR_m]);
    DEBUG_PRINT_MATRIX4D("hip to foot 2", T_hf_print[RL_m]);
    DEBUG_PRINT_MATRIX4D("hip to foot 3", T_hf_print[RR_m]);

    Eigen::Vector3d hipToFootVectors[NUM_LEGS];
    Eigen::Matrix4d T_wb = LieAlgebra::RpToTrans( // this considers world flat and cetered to the torso. ("world" is a virtual reference frame, used to calculate transforms to the feet)
        LieAlgebra::RPY(bodyOrientationRPY[0], bodyOrientationRPY[1], bodyOrientationRPY[2]).block<3, 3>(0, 0),
        Eigen::Vector3d::Zero());
    
    for(int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        // world to hip first!
        Eigen::Matrix4d T_wh = T_wb * T_bh[legIdx];

        //              T_hf =      T_hw                 * T_wf -> here is the maigc, instead of actualy using T_wf we will use T_bf, so it will apply the movement to each foot
        Eigen::Matrix4d T_hf = LieAlgebra::TransInv(T_wh) * T_bf[legIdx];
        
        T_hf_print[legIdx] = T_hf; // TODO remove, for debugging
        
        // with T_hf in hands we can get the h -> f vectors and process the IK
        Eigen::Vector3d footPosInHip = T_hf.block<3,1>(0,3);
        IK_singular(jointAngles[legIdx], footPosInHip, legIdx);

        // Update new T_bf -> body to feet transform
        //Eigen::Vector4d V_bf_new = LieAlgebra::TransInv(T_wh) * (Eigen::Vector4d(footPosInHip.x(),footPosInHip.y(),footPosInHip.z(),1.0));
        //T_bf[legIdx].block<4,1>(0,3) = V_bf_new;
        T_bf[legIdx].block<4,1>(0,3) = T_bh[legIdx] * Eigen::Vector4d(footPosInHip.x(),footPosInHip.y(),footPosInHip.z(),1.0);
    }
    
    for(int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        T_hf_print[legIdx] = LieAlgebra::TransInv(T_bh[legIdx]) * T_bf[legIdx];
    }
    DEBUG_PRINT_MATRIX4D("hip to foot 0 new", T_hf_print[FL_m]);
    DEBUG_PRINT_MATRIX4D("hip to foot 1 new", T_hf_print[FR_m]);
    DEBUG_PRINT_MATRIX4D("hip to foot 2 new", T_hf_print[RL_m]);
    DEBUG_PRINT_MATRIX4D("hip to foot 3 new", T_hf_print[RR_m]);
}

void SpotModel::IKFeetOverrides( // TODO Should not be used, REMOVE
    double jointAngles[NUM_LEGS][NUM_JOINTS], 
    const Eigen::Vector3d& bodyOrientationRPY, 
    const Eigen::Vector3d& bodyPosition, 
    const Eigen::Vector3d feetShifts[NUM_LEGS])
{
    // Eigen::Matrix4d baseToFootTransforms[NUM_LEGS];

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        Eigen::Vector3d originalFootPos = T_bf[legIdx].block<3, 1>(0, 3);
        Eigen::Vector3d shiftedFootPos = originalFootPos + feetShifts[legIdx];

        // baseToFootTransforms[legIdx] = LieAlgebra::RpToTrans(
        //     Eigen::Matrix3d::Identity(), shiftedFootPos);

        T_bf[legIdx] = LieAlgebra::RpToTrans(
            Eigen::Matrix3d::Identity(), shiftedFootPos);
    }

    // IK(jointAngles, bodyOrientationRPY, bodyPosition, baseToFootTransforms);
    IK(jointAngles, bodyOrientationRPY, bodyPosition);
}