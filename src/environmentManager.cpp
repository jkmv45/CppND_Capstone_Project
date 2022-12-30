#include "environmentManager.hpp"

// EnvironmentManager::EnvironmentManager(std::vector<std::shared_ptr<SwarmAgent>> agents){
//     this -> agentList = agents;
//     this -> numAgents = agents.size(); // Find number of active agents
//     this -> laplacian.resize(numAgents); // Set size of Laplacian matrix
// }

void EnvironmentManager::Simulate(){
    ComputeLaplacian();
    for(uint ai = 0; ai < numAgents; ai++){
        SimulateSensor(ai);
        agentList[ai].Simulate();
        // Compute Lyapunov
        // Log Data
    }
}

std::vector<SwarmAgent>* EnvironmentManager::GetAgentList(){ return &agentList; }

// std::vector<std::shared_ptr<SwarmAgent>> EnvironmentManager::GetNeighborhood(uint agentIdx){
//     std::vector<std::shared_ptr<SwarmAgent>> neighborhood;
//     neighborhood.resize(laplacian(agentIdx,agentIdx)); // Diagonal of Laplacian indicates neighborhood size for that agent
    
//     // Identify the Agents who are neighbors with agentIdx
//     uint idx = 0;
//     for (uint j = 0; j < numAgents; j++){ 
//         if (j != agentIdx){
//             if (laplacian(agentIdx,j) != 0){
//                 neighborhood.at(idx) = agentList[j];
//                 idx++;
//             }
//         }
//     }

//     return neighborhood;
// }

void EnvironmentManager::SimulateSensor(uint agentIdx){
    std::vector<Eigen::Matrix4d> poseData;
    std::vector<double> speedData;
    // Diagonal of Laplacian indicates neighborhood size for that agent
    // Use this to resize data vectors
    uint neighborhoodSize = laplacian(agentIdx,agentIdx);
    poseData.resize(neighborhoodSize); 
    speedData.resize(neighborhoodSize);
    
    // Identify the Agents who are neighbors with agentIdx
    uint idx = 0;
    for (uint j = 0; j < numAgents; j++){ 
        if (j != agentIdx){
            if (laplacian(agentIdx,j) != 0){
                poseData.at(idx) = agentList[j].GetCurrentPose();
                speedData.at(idx) = agentList[j].GetCurrentSpeed();
                idx++;
            }
        }
    }
    // Update Sensor
    agentList[agentIdx].sensor.SetSensorData(poseData,speedData);
}

Eigen::MatrixXi EnvironmentManager::ComputeLaplacian(){
    Eigen::MatrixXi degMat, adjMat;
    degMat.resize(numAgents,numAgents);
    adjMat.resize(numAgents,numAgents);
    degMat = Eigen::MatrixXi::Zero(numAgents,numAgents);
    adjMat = Eigen::MatrixXi::Zero(numAgents,numAgents);
    Eigen::Vector3d rjk;

    uint neighborhoodSize;
    for(uint j = 0; j < numAgents; j++){
        neighborhoodSize = 0;
        for(uint k = 0; k < numAgents; k++){
            if (j != k){
                rjk = agentList[j].GetCurrentPosition() - agentList[k].GetCurrentPosition();

                // Check to see if agent k is inside agent j's sensing range
                if (rjk.norm() < agentList[j].GetSensingRange()){
                    adjMat(j,k) = 1;
                    neighborhoodSize++;  // Agent k is a neighbor of agent j
                } else {
                    adjMat(j,k) = 0;
                }
            }
        }
        degMat(j,j) = neighborhoodSize;
    }
    laplacian = degMat - adjMat;
    return laplacian;
}