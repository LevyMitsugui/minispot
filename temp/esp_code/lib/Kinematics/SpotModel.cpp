// SpotModel.cpp
#include "SpotModel.hpp"
#include <vector>

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

    TorsoToHip[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0,  hip_y / 2.0, 0));
    TorsoToHip[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0, -hip_y / 2.0, 0));
    TorsoToHip[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0,  hip_y / 2.0, 0));
    TorsoToHip[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0, -hip_y / 2.0, 0));

    TorsoToFoot[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, ( foot_y / 2.0), -height));
    TorsoToFoot[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, (-foot_y / 2.0), -height));
    TorsoToFoot[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, ( foot_y / 2.0), -height));
    TorsoToFoot[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, (-foot_y / 2.0), -height));

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
 * @param baseToFootTransforms The transformation matrices from the base frame to each foot frame.
 */
void SpotModel::HipToFeet(
    Eigen::Vector3d hipToFootVectors[NUM_LEGS],
    const Eigen::Vector3d& bodyOrientationRPY,
    const Eigen::Vector3d& bodyPosition,
    const Eigen::Matrix4d baseToFootTransforms[NUM_LEGS])
{
    Eigen::Matrix4d T_woRL_mdToBase = LieAlgebra::RpToTrans(
    LieAlgebra::RPY(bodyOrientationRPY[0], bodyOrientationRPY[1], bodyOrientationRPY[2]).block<3, 3>(0, 0),
    bodyPosition);

    for (int i = 0; i < NUM_LEGS; ++i) {
        Eigen::Vector3d footPosInBase = baseToFootTransforms[i].block<3, 1>(0, 3);
        Eigen::Matrix4d T_baseToHip = LieAlgebra::TransInv(T_woRL_mdToBase) * TorsoToHip[i];
        Eigen::Vector3d hipPosInBase = T_baseToHip.block<3, 1>(0, 3);
        
        Eigen::Vector3d approxHipToFeet = footPosInBase - hipPosInBase;

        Eigen::Matrix4d T_hipToFoot = LieAlgebra::TransInv(T_baseToHip) * baseToFootTransforms[i];
        Eigen::Vector3d footPosInHip = T_hipToFoot.block<3, 1>(0, 3);

        hipToFootVectors[i] = footPosInHip;
    }
}

void SpotModel::IK(
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyOrientationRPY,
    const Eigen::Vector3d& bodyPosition)
{
    Eigen::Vector3d hipToFootVectors[NUM_LEGS];
    HipToFeet(hipToFootVectors, bodyOrientationRPY, bodyPosition, TorsoToFoot);
    
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        double jointAngleSet[NUM_JOINTS];
        Eigen::Vector3d hipToFoot = hipToFootVectors[legIdx];
        LegQuadrant side = (legIdx == FR_m || legIdx == RR_m) ? Right : Left;

        Legs[legIdx].GetJointAngles(hipToFoot[0], hipToFoot[1], hipToFoot[2], side, jointAngleSet);

        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
            jointAngles[legIdx][jointIdx] = jointAngleSet[jointIdx];
        }
    }
}


// void SpotModel::IK_singular( //TODO continue here
//     double legJointAngles[NUM_JOINTS],
//     const Eigen::Matrix4d torsoToFootTransform,)
// {

// }

void SpotModel::IK(
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyOrientationRPY,
    const Eigen::Vector3d& bodyPosition,
    const Eigen::Matrix4d TorsoToFoot[NUM_LEGS])
{
    Eigen::Vector3d hipToFootVectors[NUM_LEGS];
    HipToFeet(hipToFootVectors, bodyOrientationRPY, bodyPosition, TorsoToFoot);
    
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        double jointAngleSet[NUM_JOINTS];
        Eigen::Vector3d hipToFoot = hipToFootVectors[legIdx];
        LegQuadrant side = (legIdx == FR_m || legIdx == RR_m) ? Right : Left;

        Legs[legIdx].GetJointAngles(hipToFoot[0], hipToFoot[1], hipToFoot[2], side, jointAngleSet);

        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
            jointAngles[legIdx][jointIdx] = jointAngleSet[jointIdx];
        }
    }
}

void SpotModel::IKFeetOverrides(
    double jointAngles[NUM_LEGS][NUM_JOINTS], 
    const Eigen::Vector3d& bodyOrientationRPY, 
    const Eigen::Vector3d& bodyPosition, 
    const Eigen::Vector3d feetShifts[NUM_LEGS])
{
    Eigen::Matrix4d baseToFootTransforms[NUM_LEGS];

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        Eigen::Vector3d originalFootPos = TorsoToFoot[legIdx].block<3, 1>(0, 3);
        Eigen::Vector3d shiftedFootPos = originalFootPos + feetShifts[legIdx];

        baseToFootTransforms[legIdx] = LieAlgebra::RpToTrans(
            Eigen::Matrix3d::Identity(), shiftedFootPos);
    }

    IK(jointAngles, bodyOrientationRPY, bodyPosition, baseToFootTransforms);
}


// void SpotModel::IKFootOverrides(
//     double legJointAngles[NUM_JOINTS], 
//     const Eigen::Vector3d& bodyOrientationRPY, 
//     const Eigen::Vector3d& bodyPosition, 
//     const Eigen::Vector3d feetShifts[NUM_LEGS])
// {
//     Eigen::Matrix4d baseToFootTransforms[NUM_LEGS];

//     for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
//         Eigen::Vector3d originalFootPos = TorsoToFoot[legIdx].block<3, 1>(0, 3);
//         Eigen::Vector3d shiftedFootPos = originalFootPos + feetShifts[legIdx];

//         baseToFootTransforms[legIdx] = LieAlgebra::RpToTrans(
//             Eigen::Matrix3d::Identity(), shiftedFootPos);
//     }

//     IK(jointAngles, bodyOrientationRPY, bodyPosition, baseToFootTransforms);
// }