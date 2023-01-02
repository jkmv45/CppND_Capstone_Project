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
    VehicleParams() :   mass(20), cruiseSpeed(10), maxSpeed(25), senseRadius(150), wingSpan(2), 
                        turnRadius(10), maxRange(5E5), maxAlt(5000), opAlt(2000){};
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

    // Default Control Params Initializer List
    ControlParams()  :  gainAPF({0.5, 0, 0.001, 0.001}), gainCons({5, 0, 1, 1}), gainObj({0.006, 0, 0.001, 0.001}), maxU({5, 1, 2, 2}),damping(6.0), 
                        aAPF(2), bAPF(0), cAPF(40), eAPF(10), c0APF(0) {};
};

enum AgentRole {leader,follower};

class SwarmAgent : public SimObj {
    public:
    // Attributes
    double dt;          // Sample Time [s]
    double jFlock;      // Potential Energy related to Flocking (for Lyapunov Fcn computation)
    AgentRole myRole;   // This agents role in the swarm (leader or follower) ***(Future upgrade)***
    Sensor sensor;      // Agent's sensor for nearby agent's pose and speed

    // Constructor
    SwarmAgent(){};
    SwarmAgent(AgentRole, double, uint);

    // Methods
    void Simulate();
    double GetSensingRange();

    private:
    // Attributes
    // Structs
    VehicleParams vehParams;
    ControlParams ctrlParams;
    // Vectors
    Eigen::Vector4d inputU;     // Control input vector for agent controller
    // Scalars
    uint numAgents;

    // Methods
    void ComputeControl();
    void PropagateStates();
    Eigen::Vector2d ComputeAPF(double);
};