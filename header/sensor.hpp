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
    void SetSensorData(std::vector<Eigen::Matrix4d>&,std::vector<double>&);
    std::vector<Eigen::Matrix4d> GetPoseData();
    std::vector<double> GetSpeedData();

    // Constructor
    Sensor();

    private:
    std::vector<Eigen::Matrix4d> poseData;
    std::vector<double> speedData;
};