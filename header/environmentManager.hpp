#pragma once

#include <cmath>
#include "eigen3/Eigen/Geometry"
#include "swarmAgent.hpp"

class environmentManager{
    public:
    // Attributes

    // Methods
    Eigen::MatrixXd computeLaplacian();

    private:
    // Attributes
    Eigen::MatrixXd laplacian;
    Eigen::VectorX<swarmAgent*> agentList;

    // Methods
};