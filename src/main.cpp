// Standard Lib
#include <iostream>
#include <vector>
#include <array>
#include <random>

// Third Party
#include "matplotlibcpp.h"
#include "eigen3/Eigen/Geometry"

// Project Headers
#include "swarmAgent.hpp"
#include "environmentManager.hpp"

namespace plt = matplotlibcpp;

int main() {
    // TODO: CLI for numAgents
    
    // Simulation Parameters
    uint numAgents = 2;
    double timeStep = 0.01;

    // Test Criteria
    double rmin;                // Minimum Separation Distance
    double const rtol = 2.0;    // Separation Consensus Tolerance
    double const htol = 0.1;    // Heading Consensus Tolerance
    double const ptol = 0.1;    // Pitch Consensus Tolerance
    double const spdtol = 1.0;  // Speed Consensus Tolerance

    // Simulation Setup
    EnvironmentManager env = EnvironmentManager(numAgents, timeStep);
    // TODO: Get rid of this...or use smart pointer
    std::vector<SwarmAgent>* mySwarmPtr = env.GetAgentList();
    // TODO: make this better
    VehicleParams vParams = VehicleParams();
    ControlParams cParams = ControlParams();
    rmin = vParams.wingSpan * cParams.eAPF;

    // Setup RNG
    double senseRange = vParams.senseRadius;
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<double> udPos(vParams.wingSpan+1.0, senseRange-1.0);
    std::uniform_real_distribution<double> udAngle(-0.5*M_PI, 0.5*M_PI);
    std::uniform_real_distribution<double> udUnitVec(0.0,1.0);

    // Create Initial Conditions
    Eigen::Vector3d rvec, uvec = Eigen::Vector3d::Zero();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d dcm = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd aa;
    Eigen::Quaterniond quat;
    std::vector<Eigen::Matrix4d> initCond;
    initCond.resize(numAgents);

    // Set Initial Conditions
    for(uint i = 0; i < numAgents; i++){
        // Set Position Vector
        uvec.x() = udUnitVec(rng);
        uvec.y() = udUnitVec(rng);
        uvec.z() = udUnitVec(rng);
        uvec.normalize();
        rvec = uvec * udPos(rng);
        // Set Rotation Matrix
        aa = Eigen::AngleAxisd(udAngle(rng), uvec);
        quat = Eigen::Quaterniond(aa);
        dcm = quat.matrix();
        // Build Pose
        pose.block<3,3>(0,0) = dcm;
        pose.block<3,1>(0,3) = rvec;
        // Add Pose to IC Vector
        initCond.at(i) = pose;
    }
    env.Init(initCond);

    // Estimate number of simulation steps needed to reach terminal conditions
    env.ComputeRelativeStates();
    double timeEst = 1.1 * abs(env.relativePositions[0] - rmin) * vParams.cruiseSpeed;
    uint const numSteps = (uint)(timeEst/timeStep);
    Eigen::VectorXd timeVec = Eigen::VectorXd::LinSpaced(numSteps,0,timeStep*numSteps);
    
    // Initialize Relative State Histories
    std::vector<std::vector<double>> relDistHist(env.numRelStates,std::vector<double>(numSteps,0));
    std::vector<std::vector<double>> relSpeedHist(env.numRelStates,std::vector<double>(numSteps,0));
    std::vector<std::vector<double>> relHeadingHist(env.numRelStates,std::vector<double>(numSteps,0));

    // Run Simulation
    for (uint ti = 0; ti < numSteps; ti++){
        // Simulate Swarm
        env.Simulate();
        // Log Relative States
        env.ComputeRelativeStates();
        for(uint pi = 0; pi < env.numRelStates; pi++){
            relDistHist[pi][ti] = env.relativePositions[pi];
            relSpeedHist[pi][ti] = env.relativeSpeeds[pi];
            relHeadingHist[pi][ti] = env.relativeHeadings[pi];
            // Relative distance must always be greater than the wingspan of a single vehicle
            // EXPECT_GT(relDistHist[pi][ti], vParams.wingSpan);  // TODO: Make this a boolean check and break if true
        }
    }

    // Check that relative position, speed, and heading have decreased and are in spec
    for(uint pi = 0; pi < env.numRelStates; pi++){
        // EXPECT_LT(relDistHist[pi][numSteps-1], relDistHist[pi][0]);
        // EXPECT_NEAR(relDistHist[pi][numSteps-1], rmin, rtol);
        // EXPECT_NEAR(relSpeedHist[pi][numSteps-1],0,spdtol);
        // EXPECT_NEAR(relHeadingHist[pi][numSteps-1], 1 ,htol);
    }

    // Plot Results
    std::vector<double> tvec;
    tvec.resize(timeVec.size());
    Eigen::Map<Eigen::VectorXd>(tvec.data(), tvec.size()) = timeVec;
    // Only two agents for this test so lets just extract the first element TODO: plot all relative states
    std::vector<double> drvec = relDistHist[0];
    std::vector<double> dhvec = relHeadingHist[0];
    std::vector<double> dsvec = relSpeedHist[0];

    // TODO: Plot limit
    plt::figure(1);
    plt::plot(tvec, drvec, "bo-");
    plt::title("Relative States over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Relative Distance [m]");
    plt::grid(true);
    // plt::show();

    plt::figure(2);
    plt::plot(tvec, dhvec, "ro-");
    plt::title("Relative Heading over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Relative Heading [UL]");
    plt::grid(true);

    plt::figure(3);
    plt::plot(tvec, dsvec, "ko-");
    plt::title("Relative Speed over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Relative Speed [m/s]");
    plt::grid(true);
    plt::show();

    return 0;
}