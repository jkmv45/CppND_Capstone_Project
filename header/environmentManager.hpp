#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"

#include "SimObjABC.hpp"
#include "swarmAgent.hpp"

class EnvironmentManager : public SimObj {
    public:
    // Attributes

    // Methods
    void Simulate();
    std::vector<SwarmAgent*> GetNeighborhood();

    // Constructor
    EnvironmentManager(){};
    EnvironmentManager(std::vector<SwarmAgent*>);

    private:
    // Attributes
    Eigen::MatrixXd laplacian;
    std::vector<SwarmAgent*> agentList;

    // Methods
    Eigen::MatrixXd ComputeLaplacian();
};