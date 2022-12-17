#pragma once

#include <cmath>
#include "eigen3/Eigen/Geometry"

struct VehicleParams{
    // Scalars
    double mass;            // Takeoff Mass [kg]
    double cruiseSpeed;     // Nominal Cruise Speed [m/s]
    double maxSpeed;        // Max Allowable Speed [m/s]
    double senseRadius;     // Effective Sensing Radius [m]
    double wingSpan;        // Wingspan [m]
    double turnRadius;      // Turning Raidus [m]
    double maxRange;        // Maximum Range of Travel [m]
    double maxAlt;          // Maximum Altitude [m]
    double opAlt;           // Operational Altitude [m]

    // Generic UAV Initializer List
    VehicleParams() : mass(20), cruiseSpeed(10), maxSpeed(25), senseRadius(150), wingSpan(2), turnRadius(10), maxRange(5E5), maxAlt(5000), opAlt(2000){};
};

struct ControlParams{
    Eigen::Vector4d gainAPF;    // Artificial Potential Function Control Gains
    Eigen::Vector4d gainCons;   // Consensus Control Gains
    Eigen::Vector4d gainObj;    // Objective Control Gains
    Eigen::Vector4d maxU;       // Control Input Limits
    double damping;             // Acceleration Damping Gain

    // Contructor w/ Default Params
    ControlParams(){
        Eigen::Vector4d gainAPF = {0.5, 0, 0.003, 0.005};
        Eigen::Vector4d gainCons = {5, 0, 4, 6};
        Eigen::Vector4d gainObj = {0.006, 0, 0.001, 0.001};
        Eigen::Vector4d maxU = {5, 1, 2, 2};
        double damping = 6.0;    
    }
};

enum agentRole {leader,follower};

class swarmAgent{
    public:
    // Attributes
    double dt;          // Sample Time [s]
    agentRole myRole;   // This agents role in the swarm (leader or follower) 

    // Methods
    void Simulate();
    Eigen::Matrix4d getCurrentPose();
    Eigen::Vector3d getCurrentAngVel();
    double getCurrentSpeed();

    private:
    // Attributes
    // Structs
    VehicleParams vehParams_t;
    ControlParams ctrlParams_t;
    // Vectors
    Eigen::Vector4d inputU;
    Eigen::Vector3d tanVecT;
    Eigen::Vector3d normVecN;
    Eigen::Vector3d bnormVecB;
    Eigen::Vector3d posVecR;
    Eigen::Vector3d angVel;
    // Scalars
    double fwdSpd;

    // Methods
    Eigen::Matrix4d senseNeighborPose(swarmAgent* neighbor);
    Eigen::Vector4d senseNeighborVel(swarmAgent* neighbor);
    Eigen::Vector4d computeControlInputs();
    void propagateStates();
    double computeAPF(double);
};