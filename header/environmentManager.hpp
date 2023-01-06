/**
 * @file environmentManager.hpp
 * @author John Mills (jkmv45@gmail.com)
 * @brief  This object basically holds all objects that may exist in the simulation environment and manages relevant data. It performs the following actions:
 *          1) Executes a simulation step and handles data that needs to be passed between objects. 
 *          2) Computes relative states between agents for logging purposes. 
 *          3) Computes the graph Laplacian which is a way of tracking which agents can communicate or sense each other.
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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
    void ComputeRelativeStates();

    // Getters
    uint GetAgentListSize();
    Eigen::Matrix4d GetAgentPose(uint);
    std::vector<Eigen::Matrix4d> GetSensorData(uint);

    // Constructor
    EnvironmentManager(){};
    EnvironmentManager(uint, double);

    private:
    // Attributes
    Eigen::MatrixXi laplacian;
    std::vector<SwarmAgent> agentList;
    
    
};