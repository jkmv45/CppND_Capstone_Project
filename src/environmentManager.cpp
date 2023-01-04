#include "environmentManager.hpp"

// Constructor
EnvironmentManager::EnvironmentManager(uint swarmSize, double timeStep){
    // Set size of variables based on number of agents
    this -> numAgents = swarmSize;                  
    this -> laplacian.resize(numAgents,numAgents);  
    this -> agentList.resize(numAgents);
    this -> numRelStates = 0.5*(numAgents-1)*numAgents;
    this -> relativePositions.resize(numRelStates);
    this -> relativeSpeeds.resize(numRelStates);
    this -> relativeHeadings.resize(numRelStates);
    // Create agents and add to list
    for (uint ai = 0; ai < numAgents; ai++){
        this -> agentList[ai] = SwarmAgent(AgentRole::follower,timeStep,numAgents);
    }
}

// Public Methods
void EnvironmentManager::Init(std::vector<Eigen::Matrix4d> initCond){
    if (initCond.size() != numAgents){
        // Error, initial conditions must be same size as number of agents.
        // TODO: Create error status update
        std::cout << "Error: Not all agents have initial conditions. Input Size: " << initCond.size() << " Swarm Size: " << numAgents << std::endl;
        return;
    }
    for(uint ai = 0; ai < numAgents; ai++){
        agentList[ai].SetCurrentPose(initCond.at(ai));
    }
}

void EnvironmentManager::Simulate(){
    ComputeLaplacian();
    for(uint ai = 0; ai < numAgents; ai++){
        SimulateSensor(ai);
        agentList[ai].Simulate();
        // std::cout << " Pose of Agent: " << ai << std::endl;
        // std::cout << agentList[ai].GetCurrentPose() << std::endl;
        // std::cout << std::endl;
        // std::cout << "Position of Agent: " << ai+1 << " : " <<  agentList[ai].GetCurrentPosition().norm() << std::endl;
        // std::cout << "Speed of Agent: " << ai+1 << " : " <<  agentList[ai].GetCurrentSpeed() << std::endl;

        // Compute Lyapunov
        // Log Data
    }
}

void EnvironmentManager::ComputeRelativeStates(){
    uint p = 0; // This index keeps track of the relative state we are updating
    Eigen::Vector3d a1Heading, a2Heading;
    a1Heading = Eigen::Vector3d::Zero();
    a2Heading = Eigen::Vector3d::Zero();
    Eigen::Matrix3d a1Att, a2Att;
    a1Att = Eigen::Matrix3d::Identity();
    a2Att = Eigen::Matrix3d::Identity();
    for(uint j = 0; j < numAgents; j++){
        for(uint k = 0; k < numAgents; k++){
            if(k > j) {
                relativePositions[p] = (agentList[j].GetCurrentPosition() - agentList[k].GetCurrentPosition()).norm();
                relativeSpeeds[p] = abs(agentList[j].GetCurrentSpeed() - agentList[k].GetCurrentSpeed());
                a1Att = agentList[j].GetCurrentAttitude();
                a2Att = agentList[k].GetCurrentAttitude();
                a1Heading = agentList[j].GetTangentVec(a1Att);
                a2Heading = agentList[k].GetTangentVec(a2Att);
                relativeHeadings[p] = abs(a1Heading.dot(a2Heading));
                // TODO: Add relative "roll" (may not be needed)
                p++;
            }
        }
    }
}

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

// Getters
std::vector<SwarmAgent>* EnvironmentManager::GetAgentList(){ return &agentList; }