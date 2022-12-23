#include "environmentManager.hpp"

EnvironmentManager::EnvironmentManager(std::vector<SwarmAgent*> agents){
    this -> agentList = agents;
}

void EnvironmentManager::Simulate(){

}

std::vector<SwarmAgent*> EnvironmentManager::GetNeighborhood(){
    return agentList; // Placeholder
}

Eigen::MatrixXd EnvironmentManager::ComputeLaplacian(){
    return Eigen::Matrix3d::Identity(); // Placeholder
}