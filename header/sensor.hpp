#pragma once

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
    void SetDetectedObjects(std::vector<SimObj*>);
    void SamplePoseSensor();
    void SampleRateSensor();
    std::vector<Eigen::Matrix4d> GetPoseData(){ return poseData; };
    std::vector<Eigen::Vector4d> GetRateData(){ return rateData; };
    std::vector<SimObj*> GetDetectedObjects(){ return objDet; };

    // Eigen::Matrix4d SenseNeighborPose(SwarmAgent* neighbor);
    // Eigen::Vector4d SenseNeighborVel(SwarmAgent* neighbor);

    // Constructor
    Sensor(){};
    Sensor(double);

    private:
    std::vector<SimObj*> objDet;
    std::vector<Eigen::Matrix4d> poseData;
    std::vector<Eigen::Vector4d> rateData;
    double sensRange;
};