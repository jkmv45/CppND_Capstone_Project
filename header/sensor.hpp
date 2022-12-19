#pragma once

#include <memory>

#include "environmentManager.hpp"
#include "swarmAgent.hpp"

class swarmAgent;
class environmentManager;

class sensor {
    public:
    // Attributes
    bool noTargetInRange;
    bool newDataAvailable;

    // Methods
    void samplePoseSensor();
    void sampleRateSensor();
    std::vector<Eigen::Matrix4d> getPoseData();
    std::vector<Eigen::Vector4d> getRateData();

    // Eigen::Matrix4d senseNeighborPose(swarmAgent* neighbor);
    // Eigen::Vector4d senseNeighborVel(swarmAgent* neighbor);

    // Constructor
    sensor(swarmAgent*, std::shared_ptr<environmentManager>, double);

    private:
    swarmAgent* owner;
    std::shared_ptr<environmentManager> _env;
    std::vector<Eigen::Matrix4d> poseData;
    std::vector<Eigen::Vector4d> rateData;
    double sensRange;
};