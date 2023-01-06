/**
 * @file sensor.hpp
 * @author John Mills (jkmv45@gmail.com)
 * @brief Represents a generic sensor attached to each agent. Obtains pose and speed data for neighboring agents from the environment for use by the owning agent.
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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