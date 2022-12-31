#include "sensor.hpp"

// Constructor
Sensor::Sensor(){
    this -> poseData.assign(1,Eigen::Matrix4d::Identity());
    this -> speedData.assign(1,0);
}

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