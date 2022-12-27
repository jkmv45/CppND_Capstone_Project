#include "SimObjABC.hpp"

// Getters
Eigen::Matrix4d SimObj::GetCurrentPose(){ return pose; };

Eigen::Matrix3d SimObj::GetCurrentAttitude() {
    return pose.block<3,3>(0,0);
}
Eigen::Vector3d SimObj::GetCurrentPosition() {
    return pose.block<3,1>(0,3);
}

Eigen::Vector3d SimObj::GetCurrentAngVel(){ return angVel; };
double SimObj::GetCurrentSpeed(){ return fwdSpd; };

Eigen::Matrix3d SimObj::GetAttitudeMatrix(Eigen::Matrix4d &inpose){ return inpose.block<3,3>(0,0); }
Eigen::Vector3d SimObj::GetTangentVec(Eigen::Matrix3d &inpose){ return inpose.block<1,3>(0,0); }
Eigen::Vector3d SimObj::GetNormalVec(Eigen::Matrix3d &inpose){ return inpose.block<1,3>(1,0); }
Eigen::Vector3d SimObj::GetBinormalVec(Eigen::Matrix3d &inpose){ return inpose.block<1,3>(2,0); }

// Setters
void SimObj::SetCurrentPose(Eigen::Matrix4d newPose){ 
    // TODO: Add check for non-orthogonal transform matrix
    pose = newPose; 
}
void SimObj::SetCurrentAngVel(Eigen::Vector3d newVel){ 
    angVel = newVel; 
}

void SimObj::SetCurrentSpeed(double newSpeed){ 
    fwdSpd = newSpeed; 
}

Eigen::Matrix3d SimObj::skewSymmetric(Eigen::Vector3d vec){
    Eigen::Matrix3d result;
    result <<   0,      -vec[2],     vec[1],
                vec[2],  0,         -vec[0],
               -vec[1],  vec[0],     0;
    return result;
}