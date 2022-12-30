#include "sensor.hpp"

// Constructor
// Sensor::Sensor(double range){
//     this -> sensRange = range;
//     this -> poseData.assign(1,Eigen::Matrix4d::Identity());
//     this -> rateData.assign(1,Eigen::Vector4d::Zero());
// }

Sensor::Sensor(){
    this -> poseData.assign(1,Eigen::Matrix4d::Identity());
    this -> speedData.assign(1,0);
}

// void Sensor::SetDetectedObjects(std::vector<std::shared_ptr<SimObj>> objInRange){
//     neighborChange = (objDet != objInRange);
//     objDet.clear();
//     objDet = objInRange;
//     noTargetInRange = (objInRange.size() == 0);
// }

// Setters
void Sensor::SetSensorData(std::vector<Eigen::Matrix4d> &newPoseData,std::vector<double> &newSpeedData){
    // Update Boolean States
    neighborChange = (poseData != newPoseData);
    noTargetInRange = (newPoseData.size() == 0);
    
    // Only clear if we have targets in range
    if (!noTargetInRange){
        poseData.clear(); 
        speedData.clear();
    }
    // Update Internal Data
    poseData = newPoseData;
    speedData = newSpeedData;  
}

// Getters
std::vector<Eigen::Matrix4d> Sensor::GetPoseData(){ return poseData; };
std::vector<double> Sensor::GetSpeedData(){ return speedData; };

// void Sensor::SampleRateSensor(std::vector<SimObj> &objInRange){
//     if (!noTargetInRange){
//         rateData.clear(); // Only clear if we have targets in range
//     }
//     for (SimObj obj : objInRange){
//         Eigen::Vector4d vvec;
//         vvec[0] = obj.GetCurrentSpeed();
//         vvec.block<3,1>(1,0) << obj.GetCurrentAngVel();
//         rateData.push_back(vvec);
//     }
// }