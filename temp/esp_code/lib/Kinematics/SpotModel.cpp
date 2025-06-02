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

    WorldToHip[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0,  hip_y / 2.0, 0));
    WorldToHip[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0, -hip_y / 2.0, 0));
    WorldToHip[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0,  hip_y / 2.0, 0));
    WorldToHip[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0, -hip_y / 2.0, 0));

    WorldToFoot[FL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0,  foot_y / 2.0, -height));
    WorldToFoot[FR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, -foot_y / 2.0, -height));
    WorldToFoot[RL_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0,  foot_y / 2.0, -height));
    WorldToFoot[RR_m] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, -foot_y / 2.0, -height));
}

void SpotModel::HipToFoot(
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
        Eigen::Matrix4d T_baseToHip = LieAlgebra::TransInv(T_woRL_mdToBase) * WorldToHip[i];
        Eigen::Vector3d hipPosInBase = T_baseToHip.block<3, 1>(0, 3);
        
        Eigen::Vector3d approxHipToFoot = footPosInBase - hipPosInBase;

        Eigen::Matrix4d T_hipToFoot = LieAlgebra::TransInv(T_baseToHip) * baseToFootTransforms[i];
        Eigen::Vector3d footPosInHip = T_hipToFoot.block<3, 1>(0, 3);

        hipToFootVectors[i] = footPosInHip;
    }
}

void SpotModel::IK(
    double jointAngles[NUM_LEGS][NUM_JOINTS],
    const Eigen::Vector3d& bodyOrientationRPY,
    const Eigen::Vector3d& bodyPosition,
    const Eigen::Matrix4d woRL_mdToFootTransforms[NUM_LEGS])
{
    Eigen::Vector3d hipToFootVectors[NUM_LEGS];
    HipToFoot(hipToFootVectors, bodyOrientationRPY, bodyPosition, woRL_mdToFootTransforms);
    
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        double jointAngleSet[NUM_JOINTS];
        Eigen::Vector3d hipToFoot = hipToFootVectors[legIdx];
        LegQuadrant side = (legIdx == FR_m || legIdx == BR) ? Right : Left;

        Legs[legIdx].GetJointAngles(hipToFoot[0], hipToFoot[1], hipToFoot[2], side, jointAngleSet);

        for (int jointIdx = 0; jointIdx < NUM_JOINTS; ++jointIdx) {
            jointAngles[legIdx][jointIdx] = jointAngleSet[jointIdx];
        }
    }
}

void SpotModel::IKFootOverrides(
    double jointAngles[NUM_LEGS][NUM_JOINTS], 
    const Eigen::Vector3d& bodyOrientationRPY, 
    const Eigen::Vector3d& bodyPosition, 
    const Eigen::Vector3d footShifts[NUM_LEGS])
{
    Eigen::Matrix4d baseToFootTransforms[NUM_LEGS];

    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        Eigen::Vector3d originalFootPos = WorldToFoot[legIdx].block<3, 1>(0, 3);
        Eigen::Vector3d shiftedFootPos = originalFootPos + footShifts[legIdx];

        baseToFootTransforms[legIdx] = LieAlgebra::RpToTrans(
            Eigen::Matrix3d::Identity(), shiftedFootPos);
    }

    IK(jointAngles, bodyOrientationRPY, bodyPosition, baseToFootTransforms);
}
