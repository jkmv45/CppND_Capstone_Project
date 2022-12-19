#include "sensor.hpp"

// Constructor
sensor::sensor(swarmAgent* agent, std::shared_ptr<environmentManager> env, double range){
    owner = agent;
    _env = env;
    sensRange = range;
    poseData.assign(1,Eigen::Matrix4d::Identity());
    rateData.assign(1,Eigen::Vector4d::Zero());
}

void sensor::samplePoseSensor(){
    
    
    // poseData.push_back(neighbor->getCurrentPose());
}

// Eigen::Vector4d sensor::senseNeighborVel(swarmAgent* neighbor){
    // Eigen::Vector4d vvec;
    // vvec[0] = neighbor->getCurrentSpeed();
    // vvec.block<1,3>(0,1) << neighbor->getCurrentAngVel();
    // return vvec;
// }

// Eigen::Matrix4d sensor::senseNeighborPose(swarmAgent* neighbor){
//     return neighbor->getCurrentPose();
// }

// Eigen::Vector4d sensor::senseNeighborVel(swarmAgent* neighbor){
//     Eigen::Vector4d vvec;
//     vvec[0] = neighbor->getCurrentSpeed();
//     vvec.block<1,3>(0,1) << neighbor->getCurrentAngVel();
//     return vvec;
// }