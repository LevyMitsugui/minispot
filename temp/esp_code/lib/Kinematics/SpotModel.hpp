// SpotModel.hpp
#ifndef SPOTMODEL_HPP
#define SPOTMODEL_HPP

#include <ArduinoEigen/Eigen/Dense>
#include <map>
#include <string>
#include "LieAlgebra.hpp"
#include "Kinematics.hpp"

#define NUM_LEGS 4
#define NUM_JOINTS 3

enum legs {FL_m, FR_m, RL_m, RR_m};

class SpotModel {
public:
    SpotModel(double shoulder_length = 0.045,
              double elbow_length = 0.08,
              double wrist_length = 0.103,
              double hip_x = 0.185,
              double hip_y = 0.077,
              double foot_x = 0.185,
              double foot_y = 0.17,
              double height = 0.145);

    void HipToFeet(
        Eigen::Vector3d HTF_vectors[NUM_LEGS], 
        const Eigen::Vector3d& orn, 
        const Eigen::Vector3d& pos, 
        const Eigen::Matrix4d TorsoToFoot[NUM_LEGS]);

    void IK(
        double jointAngles[NUM_LEGS][NUM_JOINTS],
        const Eigen::Vector3d& bodyOrientationRPY,
        const Eigen::Vector3d& bodyPosition);

    // void IK(
    //     double joint_angles[NUM_LEGS][NUM_JOINTS], 
    //     const Eigen::Vector3d& orn, 
    //     const Eigen::Vector3d& pos, 
    //     const Eigen::Matrix4d T_fb[NUM_LEGS]);

    void IK_singular(
        double legJointAngles[NUM_JOINTS],
        Eigen::Vector3d newFootPos,         // new foot pos relative to robot torso. 
        int leg);
   
    void IKFeetOverrides(
        double jointAngles[NUM_LEGS][NUM_JOINTS], 
        const Eigen::Vector3d& bodyOrientationRPY, 
        const Eigen::Vector3d& bodyPosition, 
        const Eigen::Vector3d feetShifts[NUM_LEGS]);

    void FK_singular(
        double legJointAngles[NUM_JOINTS],
        int leg, 
        Eigen::Vector3d& footPosition);

    /**
     * \brief Apply transforms to the FEET so the body performs a rotation
     */
    void rotateBody(
        double jointAngles[NUM_LEGS][NUM_JOINTS],
        const Eigen::Vector3d& bodyOrientationRPY);

    /**
     * \brief Apply transforms to the FEET so the body performs a translation
     */
    void translateBody(
        double jointAngles[NUM_LEGS][NUM_JOINTS],
        const Eigen::Vector3d& bodyPosition);

    Eigen::Matrix4d T_bh[NUM_LEGS]; // Transformation matrixes from center of the body (torso) to each shoulder joint
    Eigen::Matrix4d T_bf[NUM_LEGS]; // Transformation matrixes from center of the body (torso) to each foot
    Kinematics Legs[NUM_LEGS];
    
private:
    double shoulder_length, elbow_length, wrist_length;
    double hip_x, hip_y, foot_x, foot_y, height;

    //std::map<std::string, Kinematics> Legs;
};

#endif // SPOTMODEL_HPP
