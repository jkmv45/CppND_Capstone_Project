// Standard Lib
#include <iostream>
#include <memory>
#include <array>

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
        double timestep = 0.01;
        this -> agent1 = SwarmAgent(AgentRole::leader,timestep,numAgents);
        this -> agent2 = SwarmAgent(AgentRole::follower,timestep,numAgents);
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
        uint numAgents = 3;
        double timestep = 0.01;
        double range = 100;
        simAgents.push_back(SwarmAgent(AgentRole::leader,timestep,numAgents));
        simAgents.push_back(SwarmAgent(AgentRole::follower,timestep,numAgents));
        simAgents.push_back(SwarmAgent(AgentRole::follower,timestep,numAgents));
        env = EnvironmentManager(simAgents);
    }

    EnvironmentManager env;
    std::vector<SwarmAgent> simAgents;
};

class IntegrationTests : public ::testing::Test{
    protected:
    IntegrationTests() {
        this -> numAgents = 2;
        this -> timestep = 0.01;

        // Set Initial Conditions of each Agent
        Eigen::Matrix3d rotMat = Eigen::Matrix3d::Identity();
        Eigen::Vector3d v1 = {1, 2, 3};
        Eigen::Vector3d v2 = v1;
        v2.x() += 25; // Inside sensing radius, close to rmin
        Eigen::Matrix4d t1 = Eigen::Matrix4d::Identity(); 
        Eigen::Matrix4d t2 = Eigen::Matrix4d::Identity();
        t1.block<3,3>(0,0) = rotMat;

        // Set t2 to a slightly different attitude
        Eigen::AngleAxisd aa = Eigen::AngleAxisd(0.1*M_PI, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = Eigen::Quaterniond(aa);
        rotMat = q.matrix();

        t2.block<3,3>(0,0) = rotMat;

        t1.block<3,1>(0,3) = v1;
        t2.block<3,1>(0,3) = v2;

        std::vector<SwarmAgent> simAgentList;
        simAgentList.reserve(numAgents);
        for (uint i = 0; i < numAgents; i++){
            simAgentList.push_back(SwarmAgent(AgentRole::follower,timestep,numAgents));
        }

        simAgentList[0].SetCurrentPose(t1);
        simAgentList[1].SetCurrentPose(t2);
        
        env = EnvironmentManager(simAgentList);
        this -> vParams = VehicleParams();
        this -> cParams = ControlParams();
    }

    // void TearDown() override {}
    VehicleParams vParams;
    ControlParams cParams;
    EnvironmentManager env;
    // std::vector<SwarmAgent> simAgentList;
    uint numAgents;
    double timestep;
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
    EXPECT_EQ(env.numAgents, simAgents.size());
    // EXPECT_EQ(env.GetAgentList(), simAgents);
}

TEST_F(EnvironmentTest, ConnectedGraphTest){
    Eigen::Matrix3d rotMat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v1 = {1, 2, 3};
    Eigen::Vector3d v2 = v1;
    Eigen::Vector3d v3 = v1;
    v2.x() += 149; // Inside sensing radius
    v3.x() += 151; // Outside sensing radius
    Eigen::Matrix4d t1 = Eigen::Matrix4d::Identity(); 
    Eigen::Matrix4d t2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d t3 = Eigen::Matrix4d::Identity(); 

    t1.block<3,3>(0,0) = rotMat;
    t2.block<3,3>(0,0) = rotMat;
    t3.block<3,3>(0,0) = rotMat;

    t1.block<3,1>(0,3) = v1;
    t2.block<3,1>(0,3) = v2;
    t3.block<3,1>(0,3) = v3;

    std::vector<SwarmAgent>* mySwarmPtr = env.GetAgentList();
    mySwarmPtr->at(0).SetCurrentPose(t1);
    mySwarmPtr->at(1).SetCurrentPose(t2);
    mySwarmPtr->at(2).SetCurrentPose(t3);

    Eigen::MatrixXi testLaplacian;
    testLaplacian.resize(3,3);
    testLaplacian << 1, -1,  0,
                    -1,  2, -1,
                     0, -1,  1;
    
    Eigen::MatrixXi laplacian = env.ComputeLaplacian();
    EXPECT_EQ(laplacian,testLaplacian);

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
    Eigen::Matrix3d rotMat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v1 = {1, 2, 3};
    Eigen::Vector3d v2 = v1;
    Eigen::Vector3d v3 = v1;
    v2.x() += 151; // Outside sensing radius
    v3.x() -= 151; // Outside sensing radius
    Eigen::Matrix4d t1 = Eigen::Matrix4d::Identity(); 
    Eigen::Matrix4d t2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d t3 = Eigen::Matrix4d::Identity(); 

    t1.block<3,3>(0,0) = rotMat;
    t2.block<3,3>(0,0) = rotMat;
    t3.block<3,3>(0,0) = rotMat;

    t1.block<3,1>(0,3) = v1;
    t2.block<3,1>(0,3) = v2;
    t3.block<3,1>(0,3) = v3;

    std::vector<SwarmAgent>* mySwarmPtr = env.GetAgentList();
    mySwarmPtr->at(0).SetCurrentPose(t1);
    mySwarmPtr->at(1).SetCurrentPose(t2);
    mySwarmPtr->at(2).SetCurrentPose(t3);

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
    // Declare and Allocate Time Histories
    uint const numSteps = 1000;
    uint const numVeh = 2;
    Eigen::VectorXd timeVec = Eigen::VectorXd::LinSpaced(numSteps,0,timestep*numSteps);
    // Declare Data Arrays
    std::array<std::array<double,numVeh>,numSteps> speedHist;
    // Note, doing this for now b/c I am only testing 2 agents
    // TODO: Update to handle N agents
    std::array<double,numSteps> relDistHist, relHeadingHist, detCheck;
    std::array<std::array<Eigen::Matrix4d,numVeh>,numSteps> poseHist;
    std::array<std::array<Eigen::Vector3d,numVeh>,numSteps> angvelHist;
    // Initialize Data Arrays
    std::array<double,numVeh> initSpeed;
    std::array<Eigen::Matrix4d,numVeh> initPose;
    std::array<Eigen::Vector3d,numVeh> initAngVel;
    initSpeed.fill(0);
    initPose.fill(Eigen::Matrix4d::Identity());
    initAngVel.fill(Eigen::Vector3d::Zero());
    relDistHist.fill(0);
    relHeadingHist.fill(0);
    speedHist.fill(initSpeed);
    poseHist.fill(initPose);
    angvelHist.fill(initAngVel);

    // Intermediate Variables
    double newSpeed, newDeltaR, newRelHeading = 0;
    Eigen::Matrix4d newPose;
    Eigen::Vector3d newAngVel;
    Eigen::Matrix3d a1Att, a2Att;
    Eigen::Vector3d a1Heading, a2Heading;

    std::vector<SwarmAgent>* simAgentList = env.GetAgentList();
    for (uint ti = 0; ti < numSteps; ti++){
        // Run a Simluation Step
        env.Simulate();
        
        newDeltaR = (simAgentList->at(0).GetCurrentPosition() - simAgentList->at(1).GetCurrentPosition()).norm();
        a1Att = simAgentList->at(0).GetCurrentAttitude();
        a2Att = simAgentList->at(1).GetCurrentAttitude();
        a1Heading = simAgentList->at(0).GetTangentVec(a1Att);
        a2Heading = simAgentList->at(1).GetTangentVec(a2Att);
        newRelHeading = a1Heading.dot(a2Heading);

        relDistHist[ti] = newDeltaR;
        relHeadingHist[ti] = newRelHeading;
        detCheck[ti] = a1Att.determinant();
    }
    // End of Sim Assertions
    // Check that relative position and heading have decreased
    double rmin = vParams.wingSpan * cParams.eAPF;
    EXPECT_LT(relDistHist[numSteps-1] ,relDistHist[0]);
    EXPECT_NEAR(relDistHist[numSteps-1],rmin,0.1);
    EXPECT_NEAR(relHeadingHist[numSteps-1],1,1e-6);

    // Plot Results
    std::vector<double> tvec;
    tvec.resize(timeVec.size());
    Eigen::Map<Eigen::VectorXd>(tvec.data(), tvec.size()) = timeVec;

    std::vector<double> drvec(relDistHist.begin(),relDistHist.end());
    std::vector<double> dhvec(relHeadingHist.begin(),relHeadingHist.end());
    std::vector<double> detChk(detCheck.begin(),detCheck.end());
    
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
    plt::plot(tvec, detChk, "go-");
    plt::title("Agent 1 Determinant over Time");
    plt::xlabel("Time [s]");
    plt::ylabel("Determinant [UL]");
    plt::grid(true);
    plt::show();
}