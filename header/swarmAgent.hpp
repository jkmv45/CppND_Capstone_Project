#pragma once

#include <cmath>
#include <memory>

#include "eigen3/Eigen/Geometry"

#include "SimObjABC.hpp"
#include "sensor.hpp"

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
    double aAPF;                // APF Attraction Coefficient
    double bAPF;                // APF Repulsion Coefficient
    double cAPF;                // APF Shaping Coefficient
    double eAPF;                // Wingspan Multiplier for APF Shaping (factor of safety)
    double c0APF;               // Integration constant for APF

    // Contructor w/ Default Params
    ControlParams(){
        gainAPF = {0.5, 0, 0.001, 0.001};
        gainCons = {5, 0, 1, 1};
        gainObj = {0.006, 0, 0.001, 0.001};
        maxU = {5, 1, 2, 2};
        damping = 6.0;    
        aAPF = 2;
        bAPF = 0;
        cAPF = 40;
        eAPF = 10;
        c0APF = 0;
    }
};

enum AgentRole {leader,follower};

class SwarmAgent : public SimObj {
    public:
    // Attributes
    double dt;          // Sample Time [s]
    double jFlock;      // Potential Energy related to Flocking (for Lyapunov Fcn computation)
    AgentRole myRole;   // This agents role in the swarm (leader or follower) 
    // Objects
    Sensor sensor;

    // Constructor
    // swarmAgent(agentRole role, double tstep) : myRole(role), dt(tstep) {}
    SwarmAgent(){};
    SwarmAgent(AgentRole, double, uint);

    // Methods
    void Simulate();
    // Eigen::Matrix4d GetCurrentPose();
    // Eigen::Vector3d GetCurrentAngVel();
    // double GetCurrentSpeed();

    private:
    // Attributes
    // Structs
    VehicleParams vehParams;
    ControlParams ctrlParams;
    // Vectors
    Eigen::Vector4d inputU;     // Control input vector for agent controller
    // Eigen::Vector3d tanVecT;    // Unit vector indicating forward direction along path
    // Eigen::Vector3d normVecN;   // Unit vector indicating normal direction along path
    // Eigen::Vector3d bnormVecB;  // Unit vector creating right hand system with tanVecT and normVecN (i.e. T x N)
    // Eigen::Vector3d posVecR;    // Current position of agent [m]
    // Eigen::Vector3d angVel;     // Current angular velocity of agent [rad/s]
    // Scalars
    // double fwdSpd;              // Current forward speed of agent [m/s]
    uint numAgents;

    // Methods
    // Eigen::Matrix4d senseNeighborPose(swarmAgent* neighbor);
    // Eigen::Vector4d senseNeighborVel(swarmAgent* neighbor);
    void ComputeControlInputs();
    void PropagateStates();
    Eigen::Vector2d ComputeAPF(double);
};