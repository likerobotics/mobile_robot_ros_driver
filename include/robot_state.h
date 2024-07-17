#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

class RobotState{
public:
    Eigen::Vector4f jointsPosition;
    Eigen::Vector4f jointsVelocity;

    Eigen::Vector3f odomPose;

    void reset(){
        jointsPosition<<0,0,0,0;
        jointsVelocity<<0,0,0,0;
        odomPose<<0,0,0;
    }
};