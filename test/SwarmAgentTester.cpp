// Standard Lib
#include <iostream>
#include <memory>
#include <array>
#include <random>

// Third Party
#include "gtest/gtest.h"
#include "matplotlibcpp.h"
#include "eigen3/Eigen/Geometry"

// Project Headers
#include "environmentManager.hpp"

namespace plt = matplotlibcpp;

// ***************************************** Test Fixtures ****************************************
class SwarmAgentTest : public ::testing::Test{
    protected:
    SwarmAgentTest() {
        uint numAgents = 2;
        double timeStep = 0.01;
        this -> agent1 = SwarmAgent(AgentRole::leader,timeStep,numAgents);
        this -> agent2 = SwarmAgent(AgentRole::follower,timeStep,numAgents);
        this -> testDCM = Eigen::Matrix3d::Identity();
        this -> vParams = VehicleParams();
        this -> cParams = ControlParams();
    }

    // void TearDown() override {}

    // EnvironmentManager env;
    SwarmAgent agent1;
    SwarmAgent agent2;
    Eigen::Matrix3d testDCM;
    VehicleParams vParams;
    ControlParams cParams;
};

class EnvironmentTest : public ::testing::Test{
    protected:
    EnvironmentTest() {
        this -> numAgents = 3;
        double timeStep = 0.01;
        this -> env = EnvironmentManager(numAgents, timeStep);
        this -> mySwarmPtr = env.GetAgentList();
        this -> v1 = {1, 2, 3};
        this -> v2 = v1;
        this -> v3 = v1;
        this -> t1 = Eigen::Matrix4d::Identity(); 
        this -> t2 = Eigen::Matrix4d::Identity();
        this -> t3 = Eigen::Matrix4d::Identity(); 
        double senseRange = mySwarmPtr->at(0).GetSensingRange();

        // Setup RNG
        this -> udInside = std::uniform_real_distribution(0.5*senseRange+1.0, senseRange-1.0);
        this -> udOutside = std::uniform_real_distribution(senseRange, 2.0*senseRange);
    }

    EnvironmentManager env;
    uint numAgents;
    std::vector<SwarmAgent>* mySwarmPtr;
    std::vector<Eigen::Matrix4d> initCond;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Eigen::Matrix4d t1;
    Eigen::Matrix4d t2;
    Eigen::Matrix4d t3;

    std::uniform_real_distribution<double> udInside;
    std::uniform_real_distribution<double> udOutside;
};

class IntegrationTests : public ::testing::Test{
    protected:
    IntegrationTests() {
        // Simulation Parameters
        this -> numAgents = 2;
        this -> timeStep = 0.01;
        // Simulation Setup
        env = EnvironmentManager(numAgents, timeStep);
        this -> mySwarmPtr = env.GetAgentList();
        this -> vParams = VehicleParams();
        this -> cParams = ControlParams();
        this -> rmin = vParams.wingSpan * cParams.eAPF;

        // Setup RNG
        double senseRange = vParams.senseRadius;
        std::random_device rd;
        std::mt19937 rng(rd());
        this->udAttraction = std::uniform_real_distribution(rmin, senseRange-1.0);
        this->udRepulsion = std::uniform_real_distribution(vParams.wingSpan+1.0, rmin);
        this->udAngle = std::uniform_real_distribution(-0.5*M_PI, 0.5*M_PI);

        // Create Initial Conditions
        this -> rvec, uvec = Eigen::Vector3d::Zero();
        this -> pose = Eigen::Matrix4d::Identity();
        this -> dcm = Eigen::Matrix3d::Identity();
        this -> aa;
        this -> quat;
        this -> initCond.resize(numAgents);
    }

    // void TearDown() override {}

    // Simulation Variables
    uint numAgents;
    double timeStep;
    VehicleParams vParams;
    ControlParams cParams;
    EnvironmentManager env;
    std::vector<SwarmAgent>* mySwarmPtr;
    std::vector<Eigen::Matrix4d> initCond;

    Eigen::Vector3d rvec, uvec;
    Eigen::Matrix4d pose;
    Eigen::Matrix3d dcm;
    Eigen::AngleAxisd aa;
    Eigen::Quaterniond quat;

    // Test Criteria
    double rmin;                // Minimum Separation Distance
    double const rtol = 2.0;    // Separation Consensus Tolerance
    double const htol = 0.1;    // Heading Consensus Tolerance
    double const ptol = 0.1;    // Pitch Consensus Tolerance
    double const spdtol = 1.0;  // Speed Consensus Tolerance
    
    // RNG Variables
    std::mt19937 rng;
    std::uniform_real_distribution<double> udUnitVec;
    std::uniform_real_distribution<double> udAttraction;
    std::uniform_real_distribution<double> udRepulsion;
    std::uniform_real_distribution<double> udAngle;
};

// *************************************** Test Suite Definitions **************************************

// *****************************************************************************************************
// ***************************************** Swarm Agent Tests *****************************************
// *****************************************************************************************************
TEST_F(SwarmAgentTest,ConstructorTest){
    AgentRole testrole = AgentRole::leader;
    double testval = 0.01;
    SwarmAgent myAgent = SwarmAgent(testrole, testval,1);
    EXPECT_EQ(myAgent.dt, testval) << "Time Step not set";
    EXPECT_EQ(myAgent.myRole,testrole);
}

TEST_F(SwarmAgentTest,GetPoseTest){
    Eigen::Matrix4d myPose = agent1.GetCurrentPose();
    EXPECT_EQ(myPose,Eigen::Matrix4d::Identity());
}

TEST_F(SwarmAgentTest,GetAngVelTest){
    Eigen::Vector3d myVel = agent1.GetCurrentAngVel();
    EXPECT_EQ(myVel,Eigen::Vector3d::Zero());
}

TEST_F(SwarmAgentTest,GetSpeed){
    double mySpeed = agent1.GetCurrentSpeed();
    EXPECT_EQ(mySpeed,vParams.cruiseSpeed);
}

TEST_F(SwarmAgentTest, SkewSymmetric){
    Eigen::Vector3d vec = {1, 2, 3};
    Eigen::Matrix3d mat;
    mat <<   0, -3,  2, 
             3,  0, -1, 
            -2,  1,  0;
    Eigen::Matrix3d resMat = agent1.skewSymmetric(vec);
    EXPECT_EQ(resMat,mat);
}

TEST_F(SwarmAgentTest, GetCurrentPositionTest){
    EXPECT_EQ(agent1.GetCurrentPosition(), Eigen::Vector3d::Zero());
}

TEST_F(SwarmAgentTest, GetCurrentAttitudeTest){
    EXPECT_EQ(agent1.GetCurrentAttitude(), testDCM);
}

TEST_F(SwarmAgentTest, GetAttitudeTest){
    Eigen::Matrix4d testPose = Eigen::Matrix4d::Identity();
    EXPECT_EQ(agent1.GetAttitudeMatrix(testPose),testDCM);
}
TEST_F(SwarmAgentTest, GetTangentTest){
    Eigen::Vector3d tvec = {1, 0, 0};
    EXPECT_EQ(agent1.GetTangentVec(testDCM), tvec);
}

TEST_F(SwarmAgentTest, GetNormalTest){
    Eigen::Vector3d tvec = {0, 1, 0};
    EXPECT_EQ(agent1.GetNormalVec(testDCM), tvec);
}

TEST_F(SwarmAgentTest, GetBinormalTest){
    Eigen::Vector3d tvec = {0, 0, 1};
    EXPECT_EQ(agent1.GetBinormalVec(testDCM), tvec);
}

TEST_F(SwarmAgentTest, NormalizationTest){
    // std::cout << "Finish me!!!!" << std::endl;
    Eigen::Matrix3d testDCM;
    // Perfect dcm
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = Eigen::Quaterniond(aa);
    testDCM = q.matrix();
    // Near orthogonal dcm
    testDCM = testDCM * sqrt(1.05);

    // Normalize Matrix test
    EXPECT_GT(testDCM.determinant()-1,unitUTOL);
    agent1.NormalizeAttMat(testDCM);
    EXPECT_LT(testDCM.determinant()-1,unitLTOL);
}

TEST_F(SwarmAgentTest, SetAttitudeLogicTest){
    Eigen::Matrix3d testDCM1, testDCM2, testDCM3;
    // Perfect dcm
    Eigen::AngleAxisd aa = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = Eigen::Quaterniond(aa);
    testDCM1 = q.matrix();
    // Near orthogonal dcm
    testDCM2 = testDCM1 * sqrt(1.0000001);
    // Outside normalization range
    testDCM3 = testDCM1 * 1.01;
    
    // Test Input 1 should be within lower normalization threshold.
    EXPECT_NEAR(testDCM2.determinant(),1,unitLTOL);
    // Test Input 2 should be within upper normalization treshold, but not perfect.
    EXPECT_NEAR(testDCM2.determinant(),1,unitUTOL);
    // Test Input 3 should be outside normalization range.
    EXPECT_GT(testDCM3.determinant(),1);
    
    // Test Input 1 should be exactly equal
    agent1.SetCurrentAttitude(testDCM1);
    EXPECT_EQ(agent1.GetCurrentAttitude(),testDCM1);
    // Test Input 2 should be accepted and normalized.
    agent1.SetCurrentAttitude(testDCM2);
    EXPECT_NEAR(agent1.GetCurrentAttitude().determinant(),testDCM2.determinant(),unitUTOL);
    // Test Input 3 should be rejected
    agent1.SetCurrentAttitude(testDCM3);
    EXPECT_NE(agent1.GetCurrentAttitude(),testDCM1);
}

// *****************************************************************************************************
// ******************************************* Sensor Tests ********************************************
// *****************************************************************************************************
TEST(SensorTest, ConstructorAndGetterTest){
    std::vector<Eigen::Matrix4d> testPose;
    std::vector<double> testRates;
    testPose.push_back(Eigen::Matrix4d::Identity());
    testRates.push_back(0);
    Sensor mySensor = Sensor();
    EXPECT_EQ(mySensor.GetPoseData(),testPose);
    EXPECT_EQ(mySensor.GetSpeedData(),testRates);
}

TEST(SensorTest, StatusFlagTest){
    Sensor testSensor = Sensor();
    std::vector<Eigen::Matrix4d> testPose = {};
    std::vector<double> testSpeed = {};
    // Check - No Targets, Data has changed
    testSensor.SetSensorData(testPose,testSpeed);
    EXPECT_TRUE(testSensor.noTargetInRange);
    EXPECT_TRUE(testSensor.neighborChange);
    // Check - No Targets, Data is the same
    testSensor.SetSensorData(testPose,testSpeed);
    EXPECT_TRUE(testSensor.noTargetInRange);
    EXPECT_FALSE(testSensor.neighborChange);
    // Check - Targets in range, Data has changed
    testPose.push_back(Eigen::Matrix4d::Identity());
    testSpeed.push_back(0);
    testSensor.SetSensorData(testPose,testSpeed);
    EXPECT_FALSE(testSensor.noTargetInRange);
    EXPECT_TRUE(testSensor.neighborChange);
    // Check - Targets in range, Data is the smae
    testSensor.SetSensorData(testPose,testSpeed);
    EXPECT_FALSE(testSensor.noTargetInRange);
    EXPECT_FALSE(testSensor.neighborChange);
}

// *****************************************************************************************************
// ***************************************** Environment Tests *****************************************
// *****************************************************************************************************
TEST_F(EnvironmentTest, ConstructorTest){
    EXPECT_EQ(env.numAgents, env.GetAgentList()->size());
    EXPECT_EQ(env.numAgents, numAgents);
}

TEST_F(EnvironmentTest, InitValidTest){
    std::random_device rd;
    std::mt19937 rng(rd());
    // Set Initial Conditions
    v1.x() += udInside(rng); // Inside sensing radius
    v3.x() -= udInside(rng); // Outside sensing radius
    t1.block<3,1>(0,3) = v1;
    t2.block<3,1>(0,3) = v2;
    t3.block<3,1>(0,3) = v3;

    std::vector<Eigen::Matrix4d> ic;
    ic.push_back(t1);
    ic.push_back(t2);
    ic.push_back(t3);

    env.Init(ic);

    EXPECT_EQ(mySwarmPtr->at(0).GetCurrentPose(),t1);
    EXPECT_EQ(mySwarmPtr->at(1).GetCurrentPose(),t2);
    EXPECT_EQ(mySwarmPtr->at(2).GetCurrentPose(),t3);
}

TEST_F(EnvironmentTest, InitInvalidTest){
    std::random_device rd;
    std::mt19937 rng(rd());
    // Set Initial Conditions
    v2.x() += udInside(rng); // Inside sensing radius
    t1.block<3,1>(0,3) = v1;
    t2.block<3,1>(0,3) = v2;

    std::vector<Eigen::Matrix4d> ic;
    ic.push_back(t1);
    ic.push_back(t2);

    env.Init(ic);

    EXPECT_NE(mySwarmPtr->at(0).GetCurrentPose(),t1);
    EXPECT_NE(mySwarmPtr->at(1).GetCurrentPose(),t2);
}

TEST_F(EnvironmentTest, ConnectedGraphTest){
    std::random_device rd;
    std::mt19937 rng(rd());
    // Set Initial Conditions
    v1.x() += udInside(rng); // Inside sensing radius
    v3.x() -= udInside(rng); // Outside sensing radius
    t1.block<3,1>(0,3) = v1;
    t2.block<3,1>(0,3) = v2;
    t3.block<3,1>(0,3) = v3;

    std::vector<Eigen::Matrix4d> ic;
    ic.push_back(t1);
    ic.push_back(t2);
    ic.push_back(t3);

    env.Init(ic);

    Eigen::MatrixXi testLaplacian;
    testLaplacian.resize(3,3);
    testLaplacian << 1, -1,  0,
                    -1,  2, -1,
                     0, -1,  1;
    
    Eigen::MatrixXi laplacian = env.ComputeLaplacian();
    EXPECT_EQ(laplacian,testLaplacian);

    // Check Poses
    env.SimulateSensor(0);
    env.SimulateSensor(1);
    env.SimulateSensor(2);
    std::vector<Eigen::Matrix4d> pose1 = mySwarmPtr->at(0).sensor.GetPoseData();
    std::vector<Eigen::Matrix4d> pose2 = mySwarmPtr->at(1).sensor.GetPoseData();
    std::vector<Eigen::Matrix4d> pose3 = mySwarmPtr->at(2).sensor.GetPoseData();

    EXPECT_EQ(pose1.size(),1);
    EXPECT_EQ(pose2.size(),2);
    EXPECT_EQ(pose3.size(),1);
    EXPECT_EQ(pose1[0],t2);
    EXPECT_EQ(pose2[0],t1);
    EXPECT_EQ(pose2[1],t3);
    EXPECT_EQ(pose3[0],t2);
}

TEST_F(EnvironmentTest, DisconnectedGraphTest){
    std::random_device rd;  
    std::mt19937 rng(rd());
    v2.x() += udOutside(rng); // Outside sensing radius
    v3.x() -= udOutside(rng); // Outside sensing radius 
    t1.block<3,1>(0,3) = v1;
    t2.block<3,1>(0,3) = v2;
    t3.block<3,1>(0,3) = v3;

    std::vector<Eigen::Matrix4d> ic;
    ic.push_back(t1);
    ic.push_back(t2);
    ic.push_back(t3);

    env.Init(ic);

    Eigen::MatrixXi testLaplacian;
    testLaplacian.resize(3,3);
    testLaplacian = Eigen::MatrixXi::Zero(3,3);

    Eigen::MatrixXi laplacian = env.ComputeLaplacian();
    EXPECT_EQ(laplacian,testLaplacian);

    env.SimulateSensor(0);
    env.SimulateSensor(1);
    env.SimulateSensor(2);
    std::vector<Eigen::Matrix4d> pose1 = mySwarmPtr->at(0).sensor.GetPoseData();
    std::vector<Eigen::Matrix4d> pose2 = mySwarmPtr->at(1).sensor.GetPoseData();
    std::vector<Eigen::Matrix4d> pose3 = mySwarmPtr->at(2).sensor.GetPoseData();

    EXPECT_EQ(pose1.size(),0);
    EXPECT_EQ(pose2.size(),0);
    EXPECT_EQ(pose3.size(),0);
}

// *****************************************************************************************************
// ***************************************** Integration Tests *****************************************
// *****************************************************************************************************
TEST_F(IntegrationTests, N2_AttractionAPF_Test){  
    // Set Initial Conditions
    for(uint i = 0; i < numAgents; i++){
        // Set Position Vector
        uvec.x() = udUnitVec(rng);
        uvec.y() = udUnitVec(rng);
        uvec.z() = udUnitVec(rng);
        uvec.normalize();
        rvec = uvec * udAttraction(rng);
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
    double timeEst = 1.1 * (env.relativePositions[0] - rmin) * vParams.cruiseSpeed;
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
            EXPECT_GT(relDistHist[pi][ti], vParams.wingSpan);
        }
    }

    // End of Sim Assertions
    // Check that relative position, speed, and heading have decreased and are in spec
    for(uint pi = 0; pi < env.numRelStates; pi++){
        EXPECT_LT(relDistHist[pi][numSteps-1], relDistHist[pi][0]);
        EXPECT_NEAR(relDistHist[pi][numSteps-1], rmin, rtol);
        EXPECT_NEAR(relSpeedHist[pi][numSteps-1],0,spdtol);
        EXPECT_NEAR(relHeadingHist[pi][numSteps-1], 1 ,htol);
    }

    // Plot Results
    std::vector<double> tvec;
    tvec.resize(timeVec.size());
    Eigen::Map<Eigen::VectorXd>(tvec.data(), tvec.size()) = timeVec;
    // Only two agents for this test so lets just extract the first element
    std::vector<double> drvec = relDistHist[0];
    std::vector<double> dhvec = relHeadingHist[0];
    std::vector<double> dsvec = relSpeedHist[0];

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
}

TEST_F(IntegrationTests, N2_RepulsionAPF_Test){  
    // Set Initial Conditions
    for(uint i = 0; i < numAgents; i++){
        // Set Position Vector
        uvec.x() = udUnitVec(rng);
        uvec.y() = udUnitVec(rng);
        uvec.z() = udUnitVec(rng);
        uvec.normalize();
        rvec = uvec * udRepulsion(rng);
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
    // uint const numSteps = 10000;
    // std::cout << numSteps << std::endl;
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
            EXPECT_GT(relDistHist[pi][ti], vParams.wingSpan);
        }
    }

    // End of Sim Assertions
    // Check that relative position, speed, and heading have decreased and are in spec
    for(uint pi = 0; pi < env.numRelStates; pi++){
        EXPECT_GT(relDistHist[pi][numSteps-1], relDistHist[pi][0]);
        EXPECT_NEAR(relDistHist[pi][numSteps-1], rmin, rtol);
        EXPECT_NEAR(relSpeedHist[pi][numSteps-1],0,spdtol);
        EXPECT_NEAR(relHeadingHist[pi][numSteps-1], 1 ,htol);
    }

    // Plot Results
    std::vector<double> tvec;
    tvec.resize(timeVec.size());
    Eigen::Map<Eigen::VectorXd>(tvec.data(), tvec.size()) = timeVec;
    // Only two agents for this test so lets just extract the first element
    std::vector<double> drvec = relDistHist[0];
    std::vector<double> dhvec = relHeadingHist[0];
    std::vector<double> dsvec = relSpeedHist[0];

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
}