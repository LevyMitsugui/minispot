// LieAlgebra.hpp
#ifndef LIEALGEBRA_HPP
#define LIEALGEBRA_HPP

#include <Eigen/Dense>

class LieAlgebra {
public:
    static Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omega);

    static Eigen::Matrix4d RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);

    static void TransToRp(const Eigen::Matrix4d& T, Eigen::Matrix3d& R, Eigen::Vector3d& p);

    static Eigen::Matrix4d TransInv(const Eigen::Matrix4d& T);

    static Eigen::Matrix4d RPY(double roll, double pitch, double yaw);

    static Eigen::Vector3d TransformVector(const Eigen::Vector3d& xyz_coord, 
                                           const Eigen::Matrix3d& rotation, 
                                           const Eigen::Vector3d& translation);
};

#endif // LIEALGEBRA_HPP
