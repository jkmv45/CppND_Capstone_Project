#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Geometry"

#include "SimObjABC.hpp"

class Sensor {
    public:
    // Attributes
    bool noTargetInRange{false};
    bool neighborChange{false};

    // Methods
    // void SetDetectedObjects(std::vector<std::shared_ptr<SimObj>>);
    // void SamplePoseSensor();
    // void SampleRateSensor();
    void SetSensorData(std::vector<Eigen::Matrix4d>&,std::vector<double>&);
    std::vector<Eigen::Matrix4d> GetPoseData();
    std::vector<double> GetSpeedData();
    // std::vector<std::shared_ptr<SimObj>> GetDetectedObjects(){ return objDet; };
    // double GetSensorRange(){ return sensRange; };

    // Eigen::Matrix4d SenseNeighborPose(SwarmAgent* neighbor);
    // Eigen::Vector4d SenseNeighborVel(SwarmAgent* neighbor);

    // Constructor
    Sensor();
    // Sensor(double);

    private:
    // std::vector<std::shared_ptr<SimObj>> objDet;
    std::vector<Eigen::Matrix4d> poseData;
    std::vector<double> speedData;
    // double sensRange;
};