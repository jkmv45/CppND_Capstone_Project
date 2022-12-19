#include "swarmAgent.hpp"

// Constructor
swarmAgent::swarmAgent(agentRole role, double tstep, std::shared_ptr<environmentManager> env){
    myRole = role;
    dt = tstep;
    myEnvPtr = env;
    // Initialize Private Variables
    inputU = {0, 0, 0, 0};
    tanVecT = {1, 0, 0};
    normVecN = {0, 1, 0};
    bnormVecB = {0, 0, 1};
    posVecR = {0, 0, 0};
    angVel = {0, 0, 0};
    fwdSpd = 0;
    mySensorPtr = std::make_unique<sensor>(this,env,vehParams.senseRadius);
}

// Public Methods
void swarmAgent::Simulate(){

}

// Getters
Eigen::Matrix4d swarmAgent::getCurrentPose(){
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotm;
    rotm << tanVecT, normVecN, bnormVecB;
    pose.topLeftCorner(3,3) = rotm;
    pose.block<3,1>(0,3) << posVecR.x(), posVecR.y(), posVecR.z();
    return pose;
}

Eigen::Vector3d swarmAgent::getCurrentAngVel(){
    return angVel;
}

double swarmAgent::getCurrentSpeed(){
    return fwdSpd;
}

// Private Methods
Eigen::Matrix4d swarmAgent::senseNeighborPose(swarmAgent* neighbor){
    return neighbor->getCurrentPose();
}

Eigen::Vector4d swarmAgent::senseNeighborVel(swarmAgent* neighbor){
    Eigen::Vector4d vvec;
    vvec[0] = neighbor->getCurrentSpeed();
    vvec.block<1,3>(0,1) << neighbor->getCurrentAngVel();
    return vvec;
}

Eigen::Vector4d swarmAgent::computeControlInputs(){
    Eigen::Vector4d u = Eigen::Vector4d::Zero();
    return u;
}

void swarmAgent::propagateStates(){

}

double swarmAgent::computeAPF(double rrel){
    return rrel;
}