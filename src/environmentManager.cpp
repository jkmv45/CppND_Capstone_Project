#include "environmentManager.hpp"

// *************************************** Constructor ***************************************
/**
 * @brief Construct a new Environment Manager:: Environment Manager object
 * 
 * @param swarmSize Number of vehicles (a.k.a agents) in the swarm to be simulated
 * @param timeStep  Time step of the simulation.  This is used to create the agents so they can integrate their states.
 */
EnvironmentManager::EnvironmentManager(uint swarmSize, double timeStep){
    // Set size of variables based on number of agents
    this -> numAgents = swarmSize;                  
    this -> laplacian.resize(numAgents,numAgents);  
    this -> agentList.resize(numAgents);
    // Calculate the number of relative states and resize vectors
    // The equation below is 1/2 * N^2 - N. Think of the relationships between agents depicted as an NxN matrix. The diagonal represents an agent relative to itself (not useful).
    // N^2 is the total # of pairs so subtracting N removes the diagonal leaving the upper and lower triangles.  We only need one since the other is the opposite polarity of the first. So we divide by 2.
    this -> numRelStates = 0.5*(numAgents-1)*numAgents;
    this -> relativePositions.resize(numRelStates);
    this -> relativeSpeeds.resize(numRelStates);
    this -> relativeHeadings.resize(numRelStates);
    // Create agents and add to list
    for (uint ai = 0; ai < numAgents; ai++){
        this -> agentList[ai] = SwarmAgent(AgentRole::follower,timeStep,numAgents);
    }
}

// *************************************** Public Methods ***************************************
/**
 * @brief Set the initial conditions of each swarm agent.  An error will occur if the input vector size is not equal to the number of agents.
 * 
 * @param initCond A STL vector of 4x4 Eigen matrices that represent the initial pose of each swarm agent.  Each must belong to the mathematical group SE(3), or in other words, the upper 3x3 must have a determinant of 1.
 */
void EnvironmentManager::Init(std::vector<Eigen::Matrix4d> initCond){
    if (initCond.size() != numAgents){
        // Error, initial conditions must be same size as number of agents.
        std::cout << "Error: Not all agents have initial conditions. Input Size: " << initCond.size() << " Swarm Size: " << numAgents << std::endl;
        return;
    }
    for(uint ai = 0; ai < numAgents; ai++){
        agentList[ai].SetCurrentPose(initCond.at(ai));
    }
}

/**
 * @brief Simulate all swarm agents managed by this object.  The Laplacian is computed first to establish the network graph.  Then each sensor and agent are simulated one after another.
 * 
 */
void EnvironmentManager::Simulate(){
    ComputeLaplacian();
    for(uint ai = 0; ai < numAgents; ai++){
        SimulateSensor(ai);
        agentList[ai].Simulate();
    }
}

/**
 * @brief Compute the relative distances, speeds, and heading metric between each pair of swarm agents.  For this simulation, relative heading is computed as the dot product between the tangent vectors of each agent.
 * 
 */
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
                p++;
            }
        }
    }
}

/**
 * @brief Simulate the sensor of the chosen agent. The graph Laplacian is used to determine who is in each agents "neighborhood" (a.k.a. sensing range).  
 *        This function will store the pose and speed of all agents in the chosen agents neighborhood and pass this data to its sensor. 
 *        This architecture is chosen to emulate a the way these objects would exist in the real application from an abstracted level. 
 * 
 * @param agentIdx Index of the chosen agent
 */
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

/**
 * @brief Compute the Graph Laplacian of the swarm network.  The Laplacian is a mathematical object in graph theory that represents "edges" connecting each "vertex" in a graph.
 *        In our application, nodes are the agents, and edges are the sensing/communication link between agents.  The Laplacian can be decomposed into the difference of two matrices: the degree matrix and the adjacency matrix.
 *        The degree matrix is a diagonal matrix where each entry is the number of agents that share an edge with the agent at the given index (i.e. the neighborhood size).
 *        The adjacency matrix is a symmetric matrix where each entry {a_ij} is 1 if the two agents share an edge and 0 otherwise.
 *        The Laplacian matrix is the difference between the degree and adjacency matrix: {L = D - A}.
 * 
 * @return Eigen::MatrixXi The Laplacian Matrix which is an NxN matrix where N is the number of agents in the swarm.
 */
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

// *************************************** Getters ***************************************
/**
 * @brief Return the size of the agent list managed by this object.  NOTE: Primarily for test code.
 * 
 * @return uint Size of the vector of agents
 */
uint EnvironmentManager::GetAgentListSize(){ return agentList.size(); }

/**
 * @brief Get the pose of a specified agent.  NOTE: Primarily for test code.
 * 
 * @param idx Index of the chosen agent.
 * @return Eigen::Matrix4d Pose of the chosen agent.
 */
Eigen::Matrix4d EnvironmentManager::GetAgentPose(uint idx){ return agentList.at(idx).GetCurrentPose(); }

/**
 * @brief Get the pose data stored in the sensor of the specified agent. NOTE: Primarily for test code.
 * 
 * @param idx Index of the chosen agent.
 * @return std::vector<Eigen::Matrix4d> The STL vector of poses stored in the sensor.
 */
std::vector<Eigen::Matrix4d> EnvironmentManager::GetSensorData(uint idx){ return agentList.at(idx).sensor.GetPoseData(); };