#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"

#include "SimObjABC.hpp"
#include "swarmAgent.hpp"

class EnvironmentManager {
    public:
    // Attributes
    uint numAgents{1};

    // Methods
    void Simulate();
    Eigen::MatrixXi ComputeLaplacian();
    // std::vector<std::shared_ptr<SwarmAgent>> GetNeighborhood(uint);
    void SimulateSensor(uint);
    // std::vector<std::shared_ptr<SwarmAgent>> GetAgentList();
    std::vector<SwarmAgent>* GetAgentList();

    // Constructor
    EnvironmentManager() {};
    // EnvironmentManager(std::vector<std::shared_ptr<SwarmAgent>> agents) : agentList(agents), numAgents(agents.size()), laplacian(Eigen::MatrixXi::Zero(agents.size(),agents.size())) {};
    EnvironmentManager(std::vector<SwarmAgent> agents) : agentList(agents), numAgents(agents.size()), laplacian(Eigen::MatrixXi::Zero(agents.size(),agents.size())) {};

    private:
    // Attributes
    Eigen::MatrixXi laplacian;
    // std::vector<std::shared_ptr<SwarmAgent>> agentList;
    std::vector<SwarmAgent> agentList;

    // Methods
};