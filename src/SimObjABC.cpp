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

Eigen::Vector3d SimObj::GetPositionVec(Eigen::Matrix4d &inpose){ return inpose.block<3,1>(0,3); }
Eigen::Matrix3d SimObj::GetAttitudeMatrix(Eigen::Matrix4d &inpose){ return inpose.block<3,3>(0,0); }
Eigen::Vector3d SimObj::GetTangentVec(Eigen::Matrix3d &inAtt){ return inAtt.block<1,3>(0,0); }
Eigen::Vector3d SimObj::GetNormalVec(Eigen::Matrix3d &inAtt){ return inAtt.block<1,3>(1,0); }
Eigen::Vector3d SimObj::GetBinormalVec(Eigen::Matrix3d &inAtt){ return inAtt.block<1,3>(2,0); }
void SimObj::NormalizeAttMat(Eigen::Matrix3d &inputMat){
    // Eigen::JacobiSVD<Eigen::Matrix3d> svd(inputMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // std::cout << svd.computeU() << std::endl;
    // Eigen::Matrix3d matU = svd.matrixU();
    // Eigen::Matrix3d matV = svd.matrixV();
    // inputMat = matU*matV.transpose();
    inputMat = 0.5 * (3.0*Eigen::Matrix3d::Identity() - inputMat*inputMat.transpose()) * inputMat;
}

// Setters
void SimObj::SetCurrentPose(Eigen::Matrix4d &newPose){ 
    SetCurrentPosition(GetPositionVec(newPose));
    SetCurrentAttitude(GetAttitudeMatrix(newPose));
}

void SimObj::SetCurrentPosition(Eigen::Vector3d newPos){
    pose.block<3,1>(0,3) = newPos;
}

void SimObj::SetCurrentAttitude(Eigen::Matrix3d newAtt){
    // Check if Attitude Matrix needs to be Normalized and is Right Handed
    double detVal = newAtt.determinant();
    double orthoCheck = abs(detVal - 1);
    if ( orthoCheck > unitLTOL && orthoCheck < unitUTOL && detVal > 0){ 
        // std::cout << "Normalized" << std::endl;
        NormalizeAttMat(newAtt); 
    } else if (orthoCheck > unitUTOL){ 
        // std::cout << "Rejected" << std::endl;
        return; // Invalid Matrix
    } else { ; } // no-op
    // std::cout << "Unaltered" << std::endl;
    pose.block<3,3>(0,0) = newAtt;
}

void SimObj::SetCurrentAngVel(Eigen::Vector3d &newVel){ 
    angVel = newVel; 
}

void SimObj::SetCurrentSpeed(double &newSpeed){ 
    if (newSpeed < 0){
        return; // Speed cannot be negative
    } else {
        fwdSpd = newSpeed; 
    }
}

Eigen::Matrix3d SimObj::skewSymmetric(Eigen::Vector3d &vec){
    Eigen::Matrix3d result;
    result <<   0,      -vec[2],     vec[1],
                vec[2],  0,         -vec[0],
               -vec[1],  vec[0],     0;
    return result;
}