// Standard Lib
#include <iostream>
#include <iomanip>
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
    // User Interface
    std::cout << "Hi! Welcome to the autonomous swarm simulator.\nPlease select the number of agents you would like to simulate from the options below:\n";
    std::cout << "\t(2) Two Agents\n \t(3) Three Agents\n \t(4) Four Agents\n \t(5) Five Agents\n \t(0) Exit Application\n";
    bool validInput = false;
    uint numAgents = 0;
    while(!validInput){
        std::cout << "\rEntry: ";
        std::cin >> numAgents;
        if(std::cin.fail()){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            validInput = false;
            std::cout << "Error: Invalid input type.  Please enter an integer number of agents." << std::endl;
        } else {
            if (numAgents == 0){
                std::cout << "Exitting Application. Goodbye!" << std::endl;
                return 0; // End application
            }
            else if (numAgents < 2 || numAgents > 5){
                validInput = false;
                std::cout << "Error: Invalid number of agents. Please enter a number in the range provided." << std::endl;
            } else {
                validInput = true;
                std::cout << "Number of agents selected: " << numAgents << std::endl;
            }
        }
    } 

    // Test Criteria
    bool cohesionChk, collisionChk, speedCons, headCons, posCons;
    double rmin;                // Minimum Separation Distance
    double const rtol = 2.0;    // Separation Consensus Tolerance
    double const htol = 0.1;    // Heading Consensus Tolerance
    double const ptol = 0.1;    // Pitch Consensus Tolerance
    double const spdtol = 1.0;  // Speed Consensus Tolerance

    // Simulation Setup
    double timeStep = 0.01;
    EnvironmentManager env = EnvironmentManager(numAgents, timeStep);
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
    rvec = Eigen::Vector3d::Zero();
    uvec = Eigen::Vector3d::Zero();
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
        // std::cout << "Pose for agent " << i+1 << " has been defined as: " << std::endl;
        // std::cout << pose << std::endl;
    }
    env.Init(initCond);
    std::cout << "Initial Conditions have been set" << std::endl;

    // Estimate number of simulation steps needed to reach terminal conditions
    env.ComputeRelativeStates();
    double maxDist = 0;
    for(uint pi = 0; pi < env.numRelStates; pi++){
        if (env.relativePositions[pi] > maxDist){
            maxDist = env.relativePositions[pi];
        }
    }
    double timeEst = 1.1 * abs(maxDist - rmin) * vParams.cruiseSpeed;
    uint const numSteps = (uint)(timeEst/timeStep);
    Eigen::VectorXd timeVec = Eigen::VectorXd::LinSpaced(numSteps,0,timeStep*numSteps);
    std::cout << "\nSimulation time duration is (~): " << (int)timeEst << " seconds\n" << std::endl;
    
    // Initialize Relative State Histories
    std::vector<std::vector<double>> relDistHist(env.numRelStates,std::vector<double>(numSteps,0));
    std::vector<std::vector<double>> relSpeedHist(env.numRelStates,std::vector<double>(numSteps,0));
    std::vector<std::vector<double>> relHeadingHist(env.numRelStates,std::vector<double>(numSteps,0));

    // Run Simulation
    uint percentComplete = 0;
    collisionChk = true;
    std::cout << "Percentage Complete: ";
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
            if (env.relativePositions[pi] <= vParams.wingSpan){
                std::cout << std::endl << "Error: Vehicle collision detected!" << std::endl;
                collisionChk = false;
            }
        }
        // Report status to user
        if (ti%10 == 0){ // to limit the number of cout calls
            percentComplete = 100*(ti/(double)numSteps) + 1;
            std::cout << std::setw(3) << percentComplete << "%\b\b\b\b"; // I did this to make the cursor behavior more consistent
        }
    }

    // Check that relative position, speed, and heading have decreased and are in spec
    cohesionChk = true;
    posCons = false; // We initialize this as false b/c we just care if at least the minimum value is within tolerance.  Larger swarms make it impractical for all to be within spec.
    headCons = true;
    speedCons = true;
    for(uint pi = 0; pi < env.numRelStates; pi++){
        cohesionChk = cohesionChk && (abs(relDistHist[pi][numSteps-1] - rmin) < abs(relDistHist[pi][0]- rmin));
        posCons = posCons || (abs(relDistHist[pi][numSteps-1] - rmin) < rtol); // Note the OR condition.  This is intentional.  See comment above.
        headCons = headCons && (abs(relHeadingHist[pi][numSteps-1] - 1.0) < htol);
        speedCons = speedCons && (abs(relSpeedHist[pi][numSteps-1]) < spdtol);
    }

    // Report Results
    auto passOrFail = [](bool& chk){ if(chk){ return "PASS";} else{ return "FAIL";} };
    std::cout << "\n\n";
    std::cout << "********** Simulation Results Summary **********" << std::endl;
    std::cout << "(1) Collision Avoidance Check:\t\t" << passOrFail(collisionChk) << "\n";
    std::cout << "(2) Sepration Distance Check: \t\t" << passOrFail(posCons) << "\n";
    std::cout << "(3) Heading Consensus Check:  \t\t" << passOrFail(headCons) << "\n";
    std::cout << "(4) Speed Consensus Check:    \t\t" << passOrFail(speedCons) << "\n";
    std::cout << "(5) Swarm Cohesion Check:     \t\t" << passOrFail(cohesionChk) << "\n";

    // Prepare Vectors to be Plotted
    std::vector<double> tvec;
    tvec.resize(timeVec.size());
    Eigen::Map<Eigen::VectorXd>(tvec.data(), tvec.size()) = timeVec;
    std::vector<double> yaxis(numSteps,0.0);
    std::vector<double> rminvec(numSteps,rmin);
    std::vector<double> rtolvecUL(numSteps,rmin+rtol);
    std::vector<double> rtolvecLL(numSteps,rmin-rtol);
    std::vector<double> htrgvec(numSteps,1.0);
    std::vector<double> htolvecUL(numSteps,1.0+htol);
    std::vector<double> htolvecLL(numSteps,1.0-htol);
    std::vector<double> sptolvecUL(numSteps,spdtol);
    std::vector<double> sptolvecLL(numSteps,-spdtol);

    // Create Legend for Relative States
    std::vector<std::string> legendEntries;
    legendEntries.resize(env.numRelStates);
    uint p = 0;
    for(uint j = 0; j < numAgents; j++){
        for(uint k = 0; k < numAgents; k++){
            if(k > j) {
                legendEntries[p] = "Agent " + std::to_string(j+1) + " to " + std::to_string(k+1);
                p++;
            }
        }
    }

    // Plot Results
    plt::figure(1);
    plt::plot(tvec,rminvec,"m-.");
    plt::plot(tvec,rtolvecUL,"r--");
    plt::plot(tvec,rtolvecLL,"r--");
    plt::title("Relative Distance over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Relative Distance [m]");
    plt::grid(true);
    for(uint pi = 0; pi < env.numRelStates; pi++){
        yaxis = relDistHist[pi];
        plt::plot(tvec, yaxis, {{"label",legendEntries[pi]}});
    }
    plt::legend();

    plt::figure(2);
    plt::plot(tvec,htrgvec,"m-.");
    plt::plot(tvec,htolvecUL,"r--");
    plt::plot(tvec,htolvecLL,"r--");
    plt::title("Relative Heading over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Relative Heading [UL]");
    plt::grid(true);
    for(uint pi = 0; pi < env.numRelStates; pi++){
        yaxis = relHeadingHist[pi];
        plt::plot(tvec, yaxis, {{"label",legendEntries[pi]}});
    }
    plt::legend();

    plt::figure(3);
    plt::plot(tvec,sptolvecUL,"r--");
    plt::plot(tvec,sptolvecLL,"r--");
    plt::title("Relative Speed over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Relative Speed [m/s]");
    plt::grid(true);
    for(uint pi = 0; pi < env.numRelStates; pi++){
        yaxis = relSpeedHist[pi];
        plt::plot(tvec, yaxis, {{"label",legendEntries[pi]}});
    }
    plt::legend();
    
    plt::show();

    return 0;
}