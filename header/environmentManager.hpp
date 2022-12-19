#pragma once

#include <cmath>
#include "eigen3/Eigen/Geometry"
#include "swarmAgent.hpp"

class swarmAgent;

class environmentManager{
    public:
    // Attributes

    // Methods
    Eigen::VectorX<swarmAgent*> getNeighborhood();

    private:
    // Attributes
    Eigen::MatrixXd laplacian;
    Eigen::VectorX<swarmAgent*> agentList;

    // Methods
    Eigen::MatrixXd computeLaplacian();
};