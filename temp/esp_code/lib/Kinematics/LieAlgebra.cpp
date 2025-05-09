// LieAlgebra.cpp
#include "LieAlgebra.hpp"

Eigen::Matrix3d LieAlgebra::VecToso3(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d so3;
    so3 <<     0, -omega(2),  omega(1),
           omega(2),      0, -omega(0),
          -omega(1), omega(0),      0;
    return so3;
}

Eigen::Matrix4d LieAlgebra::RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = p;
    return T;
}

void LieAlgebra::TransToRp(const Eigen::Matrix4d& T, Eigen::Matrix3d& R, Eigen::Vector3d& p) {
    R = T.block<3,3>(0,0);
    p = T.block<3,1>(0,3);
}

Eigen::Matrix4d LieAlgebra::TransInv(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    TransToRp(T, R, p);
    Eigen::Matrix3d Rt = R.transpose();
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3,3>(0,0) = Rt;
    T_inv.block<3,1>(0,3) = -Rt * p;
    return T_inv;
}

Eigen::Matrix4d LieAlgebra::RPY(double roll, double pitch, double yaw) {
    Eigen::Matrix4d Roll = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Pitch = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Yaw = Eigen::Matrix4d::Identity();

    Roll.block<3,3>(0,0) <<
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll),  cos(roll);

    Pitch.block<3,3>(0,0) <<
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Yaw.block<3,3>(0,0) <<
        cos(yaw), -sin(yaw), 0,
        sin(yaw),  cos(yaw), 0,
        0, 0, 1;

    return Roll * Pitch * Yaw;
}

Eigen::Vector3d LieAlgebra::TransformVector(const Eigen::Vector3d& xyz_coord, 
                                            const Eigen::Matrix3d& rotation, 
                                            const Eigen::Vector3d& translation) {
    Eigen::Matrix4d T = RpToTrans(rotation, translation);
    Eigen::Vector4d vec_homogeneous;
    vec_homogeneous << xyz_coord, 1.0;
    Eigen::Vector4d result = T * vec_homogeneous;
    return result.head<3>();
}
