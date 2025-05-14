// SpotModel.hpp
#ifndef SPOTMODEL_HPP
#define SPOTMODEL_HPP

#include <ArduinoEigen/Eigen/Dense>
#include <map>
#include <string>
#include "LieAlgebra.hpp"
#include "Kinematics.hpp"

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

    std::map<std::string, Eigen::Vector3d> HipToFoot(const Eigen::Vector3d& orn,
                                                     const Eigen::Vector3d& pos,
                                                     const std::map<std::string, Eigen::Matrix4d>& T_bf);

    std::array<std::array<double, 3>, 4> IK(const Eigen::Vector3d& orn,
                                           const Eigen::Vector3d& pos,
                                           const std::map<std::string, Eigen::Matrix4d>& T_bf);
    
    std::map<std::string, Eigen::Matrix4d> WorldToHip;
    std::map<std::string, Eigen::Matrix4d> WorldToFoot;
    
private:
    double shoulder_length, elbow_length, wrist_length;
    double hip_x, hip_y, foot_x, foot_y, height;

    std::map<std::string, Kinematics> Legs;
};

#endif // SPOTMODEL_HPP
