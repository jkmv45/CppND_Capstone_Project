#include "sensor.hpp"

// *************************************** Constructor ***************************************
/**
 * @brief Construct a new Sensor:: Sensor object
 * 
 */
Sensor::Sensor(){
    this -> poseData.assign(1,Eigen::Matrix4d::Identity());
    this -> speedData.assign(1,0);
}

// *************************************** Setters ***************************************
/**
 * @brief Setter for new pose and speed data from objects within the sensor's range.  The boolean statuses will also be updated accordingly:
 *        neighborChange will indicate if the input data is different than the existing data.
 *        noTargetInRange will indicate if the new data has a size of zero (i.e. no neighbors).
 * 
 * @param newPoseData  STL vector of new pose data to be set, as 4x4 transformation matrices
 * @param newSpeedData STL vector of new speed data to be set, as a scalar
 */
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

// *************************************** Getters ***************************************
/**
 * @brief Get latest pose data.
 * 
 * @return std::vector<Eigen::Matrix4d> STL vector of pose data for objects seen by the sensor.
 */
std::vector<Eigen::Matrix4d> Sensor::GetPoseData(){ return poseData; };

/**
 * @brief Get latest speed data.
 * 
 * @return std::vector<double> STL vector of speed data for objects seen by the sensor.
 */
std::vector<double> Sensor::GetSpeedData(){ return speedData; };