#include "swarmAgent.hpp"

// Constructor
SwarmAgent::SwarmAgent(AgentRole role, double tstep){
    this -> myRole = role;
    this -> dt = tstep;
    // Initialize Private Variables
    // this -> inputU = {0, 0, 0, 0};
    // this -> tanVecT = {1, 0, 0};
    // this -> normVecN = {0, 1, 0};
    // this -> bnormVecB = {0, 0, 1};
    // this -> posVecR = {0, 0, 0};
    // this -> angVel = {0, 0, 0};
    // this -> fwdSpd = 0;
    this -> sensor = Sensor(vehParams.senseRadius);
}

// Public Methods
void SwarmAgent::Simulate(){

}

// Getters
// Eigen::Matrix4d SwarmAgent::GetCurrentPose(){
//     Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
//     Eigen::Matrix3d rotm;
//     rotm << tanVecT, normVecN, bnormVecB;
//     pose.topLeftCorner(3,3) = rotm;
//     pose.block<3,1>(0,3) << posVecR.x(), posVecR.y(), posVecR.z();
//     return pose;
// }

// Eigen::Vector3d SwarmAgent::GetCurrentAngVel(){
//     return angVel;
// }

// double SwarmAgent::GetCurrentSpeed(){
//     return fwdSpd;
// }

// Private Methods
// Eigen::Matrix4d SwarmAgent::SenseNeighborPose(SwarmAgent* neighbor){
//     return neighbor->GetCurrentPose();
// }

// Eigen::Vector4d SwarmAgent::SenseNeighborVel(SwarmAgent* neighbor){
//     Eigen::Vector4d vvec;
//     vvec[0] = neighbor->GetCurrentSpeed();
//     vvec.block<1,3>(0,1) << neighbor->GetCurrentAngVel();
//     return vvec;
// }

Eigen::Vector4d SwarmAgent::ComputeControlInputs(){
    Eigen::Vector4d u = Eigen::Vector4d::Zero();
    return u;
}

void SwarmAgent::PropagateStates(){

}

double SwarmAgent::ComputeAPF(double rrel){
    return rrel;
}