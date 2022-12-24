#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Geometry"

#include "SimObjABC.hpp"

class Sensor {
    public:
    // Attributes
    bool noTargetInRange;
    bool newDataAvailable;

    // Methods
    void SetDetectedObjects(std::vector<std::shared_ptr<SimObj>>);
    void SamplePoseSensor();
    void SampleRateSensor();
    std::vector<Eigen::Matrix4d> GetPoseData(){ return poseData; };
    std::vector<Eigen::Vector4d> GetRateData(){ return rateData; };
    std::vector<std::shared_ptr<SimObj>> GetDetectedObjects(){ return objDet; };
    double GetSensorRange(){ return sensRange; };

    // Eigen::Matrix4d SenseNeighborPose(SwarmAgent* neighbor);
    // Eigen::Vector4d SenseNeighborVel(SwarmAgent* neighbor);

    // Constructor
    Sensor(){};
    Sensor(double);

    private:
    std::vector<std::shared_ptr<SimObj>> objDet;
    std::vector<Eigen::Matrix4d> poseData;
    std::vector<Eigen::Vector4d> rateData;
    double sensRange;
};