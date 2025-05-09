// SpotModel.cpp
#include "SpotModel.hpp"
#include <vector>

SpotModel::SpotModel() {
    shoulder_length = 0.04;
    elbow_length = 0.07;
    wrist_length = 0.11;
    hip_x = 0.192;
    hip_y = 0.085;
    foot_x = 0.192;
    foot_y = 0.17;
    height = 0.10;

    Legs["FL"].Initialize(shoulder_length, elbow_length, wrist_length);
    Legs["FR"].Initialize(shoulder_length, elbow_length, wrist_length);
    Legs["BL"].Initialize(shoulder_length, elbow_length, wrist_length);
    Legs["BR"].Initialize(shoulder_length, elbow_length, wrist_length);

    Eigen::Matrix3d Rwb = Eigen::Matrix3d::Identity();

    WorldToHip["FL"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0,  hip_y / 2.0, 0));
    WorldToHip["FR"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( hip_x / 2.0, -hip_y / 2.0, 0));
    WorldToHip["BL"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0,  hip_y / 2.0, 0));
    WorldToHip["BR"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-hip_x / 2.0, -hip_y / 2.0, 0));

    WorldToFoot["FL"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0,  foot_y / 2.0, -height));
    WorldToFoot["FR"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d( foot_x / 2.0, -foot_y / 2.0, -height));
    WorldToFoot["BL"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0,  foot_y / 2.0, -height));
    WorldToFoot["BR"] = LieAlgebra::RpToTrans(Rwb, Eigen::Vector3d(-foot_x / 2.0, -foot_y / 2.0, -height));
}

std::map<std::string, Eigen::Vector3d> SpotModel::HipToFoot(const Eigen::Vector3d& orn,
                                                            const Eigen::Vector3d& pos,
                                                            const std::map<std::string, Eigen::Matrix4d>& T_bf) {
    std::map<std::string, Eigen::Vector3d> HipToFoot_List;

    Eigen::Matrix4d T_wb = LieAlgebra::RpToTrans(LieAlgebra::RPY(orn[0], orn[1], orn[2]).block<3,3>(0,0), pos);

    for (const auto& [key, T_wh] : WorldToHip) {
        Eigen::Vector3d p_bf = T_bf.at(key).block<3,1>(0,3);
        Eigen::Matrix4d T_bh = LieAlgebra::TransInv(T_wb) * T_wh;
        Eigen::Vector3d p_bh = T_bh.block<3,1>(0,3);
        Eigen::Vector3d p_hf0 = p_bf - p_bh;
        Eigen::Matrix4d T_hf = LieAlgebra::TransInv(T_bh) * T_bf.at(key);
        Eigen::Vector3d p_hf1 = T_hf.block<3,1>(0,3);
        Eigen::Vector3d p_hf = p_hf1; // Prefer transform-based result
        HipToFoot_List[key] = p_hf;
    }

    return HipToFoot_List;
}

std::array<std::array<double, 3>, 4> SpotModel::IK(const Eigen::Vector3d& orn,
                                                   const Eigen::Vector3d& pos,
                                                   const std::map<std::string, Eigen::Matrix4d>& T_bf) {
    std::array<std::array<double, 3>, 4> joint_angles{};
    std::vector<std::string> order = {"FL", "FR", "BL", "BR"};
    auto HipToFootVectors = HipToFoot(orn, pos, T_bf);

    for (size_t i = 0; i < order.size(); ++i) {
        double angles[3];
        const auto& key = order[i];
        Eigen::Vector3d p_hf = HipToFootVectors[key];
        Legs[key].GetJointAngles(p_hf[0], p_hf[1], p_hf[2], key == "FR" || key == "BR" ? Right : Left, angles);
        for (int j = 0; j < 3; ++j) {
            joint_angles[i][j] = angles[j];
        }
    }

    return joint_angles;
}
