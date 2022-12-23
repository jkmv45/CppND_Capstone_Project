#include "sensor.hpp"

// Constructor
Sensor::Sensor(double range){
    this -> sensRange = range;
    this -> poseData.assign(1,Eigen::Matrix4d::Identity());
    this -> rateData.assign(1,Eigen::Vector4d::Zero());
}

void Sensor::SetDetectedObjects(std::vector<SimObj*> objInRange){
    newDataAvailable = (objDet != objInRange);
    objDet = objInRange;
    noTargetInRange = (objInRange.size() == 0);
}

void Sensor::SamplePoseSensor(){
    poseData.resize(objDet.size());
    for (SimObj* obj : objDet){
        poseData.push_back(obj->GetCurrentPose());
    }
}

// Eigen::Vector4d Sensor::SenseNeighborVel(SwarmAgent* neighbor){
    // Eigen::Vector4d vvec;
    // vvec[0] = neighbor->GetCurrentSpeed();
    // vvec.block<1,3>(0,1) << neighbor->GetCurrentAngVel();
    // return vvec;
// }

// Eigen::Matrix4d Sensor::SenseNeighborPose(SwarmAgent* neighbor){
//     return neighbor->GetCurrentPose();
// }

// Eigen::Vector4d Sensor::SenseNeighborVel(SwarmAgent* neighbor){
//     Eigen::Vector4d vvec;
//     vvec[0] = neighbor->GetCurrentSpeed();
//     vvec.block<1,3>(0,1) << neighbor->GetCurrentAngVel();
//     return vvec;
// }