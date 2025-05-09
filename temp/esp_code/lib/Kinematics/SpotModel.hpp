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
    SpotModel();

    std::map<std::string, Eigen::Vector3d> HipToFoot(const Eigen::Vector3d& orn,
                                                     const Eigen::Vector3d& pos,
                                                     const std::map<std::string, Eigen::Matrix4d>& T_bf);

    std::array<std::array<double, 3>, 4> IK(const Eigen::Vector3d& orn,
                                           const Eigen::Vector3d& pos,
                                           const std::map<std::string, Eigen::Matrix4d>& T_bf);

private:
    double shoulder_length, elbow_length, wrist_length;
    double hip_x, hip_y, foot_x, foot_y, height;

    std::map<std::string, Kinematics> Legs;
    std::map<std::string, Eigen::Matrix4d> WorldToHip;
    std::map<std::string, Eigen::Matrix4d> WorldToFoot;
};

#endif // SPOTMODEL_HPP
