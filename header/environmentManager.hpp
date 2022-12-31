#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <iostream>

#include "eigen3/Eigen/Geometry"

#include "SimObjABC.hpp"
#include "swarmAgent.hpp"

class EnvironmentManager {
    public:
    // Attributes
    uint numAgents{1};
    uint numRelStates{1};
    std::vector<double> relativePositions, relativeHeadings, relativeSpeeds;

    // Methods
    void Init(std::vector<Eigen::Matrix4d>);
    void Simulate();
    Eigen::MatrixXi ComputeLaplacian();
    void SimulateSensor(uint);
    std::vector<SwarmAgent>* GetAgentList();
    void ComputeRelativeStates();

    // Constructor
    EnvironmentManager(){};
    EnvironmentManager(uint, double);

    private:
    // Attributes
    Eigen::MatrixXi laplacian;
    std::vector<SwarmAgent> agentList;
};