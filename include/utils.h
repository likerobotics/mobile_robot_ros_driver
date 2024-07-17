#pragma once

#include <Eigen/Dense>

void fromYawToQuaternioun(float & yaw, Eigen::Quaterniond & q){
    q = Eigen::AngleAxisd{yaw, Eigen::Vector3d{0, 0, 1}}
}