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

class SensorTest : public ::testing::Test{
    protected:
    SensorTest() {
        uint numAgents = 2;
        double timestep = 0.01;
        double range = 100;
        sensedAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::leader,timestep,numAgents)));
        sensedAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::leader,timestep,numAgents)));
        this -> testSensor = Sensor(range);
    }

    // void TearDown() override {}

    Sensor testSensor;
    std::vector<std::shared_ptr<SimObj>> sensedAgents;
};

class EnvironmentTest : public ::testing::Test{
    protected:
    EnvironmentTest() {
        uint numAgents = 3;
        double timestep = 0.01;
        double range = 100;
        simAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::leader,timestep,numAgents)));
        simAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::follower,timestep,numAgents)));
        simAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::follower,timestep,numAgents)));
        env = EnvironmentManager(simAgents);
    }

    EnvironmentManager env;
    std::vector<std::shared_ptr<SwarmAgent>> simAgents;
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

        this -> simAgentList.reserve(numAgents);
        for (uint i = 0; i < numAgents; i++){
            this -> simAgentList.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::follower,timestep,numAgents)));
        }

        this -> simAgentList[0]->SetCurrentPose(t1);
        this -> simAgentList[1]->SetCurrentPose(t2);
        
        env = EnvironmentManager(simAgentList);
        this -> vParams = VehicleParams();
        this -> cParams = ControlParams();
    }

    // void TearDown() override {}
    VehicleParams vParams;
    ControlParams cParams;
    EnvironmentManager env;
    std::vector<std::shared_ptr<SwarmAgent>> simAgentList;
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

// TEST_F(SwarmAgentTest, SimulateTest){
//     // Set Initial Conditions of each Agent
//     Eigen::Matrix3d rotMat = Eigen::Matrix3d::Identity();
//     Eigen::Vector3d v1 = {1, 2, 3};
//     Eigen::Vector3d v2 = v1;
//     v2.x() += 25; // Inside sensing radius, close to rmin
//     Eigen::Matrix4d t1 = Eigen::Matrix4d::Identity(); 
//     Eigen::Matrix4d t2 = Eigen::Matrix4d::Identity();
//     t1.block<3,3>(0,0) = rotMat;

//     // Set t2 to a slightly different attitude
//     Eigen::AngleAxisd aa = Eigen::AngleAxisd(0.1*M_PI, Eigen::Vector3d::UnitZ());
//     Eigen::Quaterniond q = Eigen::Quaterniond(aa);
//     rotMat = q.matrix();

//     t2.block<3,3>(0,0) = rotMat;

//     t1.block<3,1>(0,3) = v1;
//     t2.block<3,1>(0,3) = v2;

//     agent1.SetCurrentPose(t1);
//     agent2.SetCurrentPose(t2);
    
//     // Store Starting States
//     double startSpeed = agent1.GetCurrentSpeed();
//     Eigen::Vector3d startAngVel = agent1.GetCurrentAngVel();
//     Eigen::Matrix4d startPose = agent1.GetCurrentPose();
//     // Prepare Sensor
//     std::vector<std::shared_ptr<SimObj>> neighbors;
//     neighbors.push_back(std::make_shared<SwarmAgent>(std::move(agent2)));
//     agent1.sensor.SetDetectedObjects(neighbors);
    
//     // Declare and Allocate Time Histories
//     uint num_steps = 1000;
//     Eigen::VectorXd timeVec = Eigen::VectorXd::LinSpaced(num_steps,0,agent1.dt*num_steps);
//     Eigen::VectorXd speedHist, relDistHist, relHeadingHist;
//     Eigen::Matrix3d a1Att, a2Att;
//     Eigen::Vector3d a1Heading, a2Heading;
//     std::vector<Eigen::Matrix4d> poseHist;
//     std::vector<Eigen::Vector3d> angvelHist;
//     speedHist.resize(num_steps);
//     relDistHist.resize(num_steps);
//     relHeadingHist.resize(num_steps);
//     poseHist.reserve(num_steps);
//     angvelHist.reserve(num_steps);
//     // Initialize Time Histories
//     speedHist[0] = startSpeed;
//     poseHist.push_back(startPose);
//     angvelHist.push_back(startAngVel);
//     // Intermediate Variables
//     double newSpeed, newDeltaR, newRelHeading = 0;
//     Eigen::Matrix4d newPose;
//     Eigen::Vector3d newAngVel;
//     for (uint ti = 0; ti < num_steps; ti++){
//         // Run a Simluation Step
//         agent1.Simulate();
//         // Get New States
//         newSpeed = agent1.GetCurrentSpeed();
//         newAngVel = agent1.GetCurrentAngVel();
//         newPose = agent1.GetCurrentPose();
//         newDeltaR = (agent1.GetCurrentPosition() - agent2.GetCurrentPosition()).norm();
//         a1Att = agent1.GetCurrentAttitude();
//         a2Att = agent2.GetCurrentAttitude();
//         a1Heading = agent1.GetTangentVec(a1Att);
//         a2Heading = agent2.GetTangentVec(a2Att);
//         newRelHeading = a1Heading.dot(a2Heading);
//         // Add New States to History
//         speedHist[ti] = newSpeed;
//         angvelHist.push_back(newAngVel);
//         poseHist.push_back(newPose);
//         relDistHist[ti] = newDeltaR;
//         relHeadingHist[ti] = newRelHeading;
//         // Assert No NANs
//         EXPECT_FALSE(std::isnan(newSpeed)) << "New Speed is NaN" << std::endl << newSpeed;
//         EXPECT_FALSE(std::isnan(newDeltaR)) << "New Relative Position is NaN" << std::endl << newDeltaR;
//         EXPECT_FALSE(newPose.hasNaN()) << "Pose Has NaN at index: " << ti << std::endl << newPose;
//         EXPECT_FALSE(newAngVel.hasNaN()) << "AngVel Has NaN at index: " << ti << std::endl << newAngVel;
//     }
//     // End of Sim Assertions
//     a1Att = agent1.GetCurrentAttitude();
//     a2Att = agent2.GetCurrentAttitude();
//     a1Heading = agent1.GetTangentVec(a1Att);
//     a2Heading = agent2.GetTangentVec(a2Att);
//     // Check that relative position and heading have decreased
//     EXPECT_LT(relDistHist[num_steps-1] ,relDistHist[0]);
//     EXPECT_NEAR(a1Heading.dot(a2Heading),1,1e-3);

//     // Plot Results
//     std::vector<double> tvec, drvec, dhvec;
//     tvec.resize(timeVec.size());
//     drvec.resize(timeVec.size());
//     dhvec.resize(timeVec.size());
//     Eigen::Map<Eigen::VectorXd>(tvec.data(), tvec.size()) = timeVec;
//     Eigen::Map<Eigen::VectorXd>(drvec.data(), drvec.size()) = relDistHist;
//     Eigen::Map<Eigen::VectorXd>(dhvec.data(), dhvec.size()) = relHeadingHist;
    
//     plt::figure();
//     plt::plot(tvec, drvec, "bo-");
//     plt::title("Relative Distance over Time");
//     plt::xlabel("Time [s]");
//     plt::ylabel("Relative Distance [m]");
//     plt::grid(true);
//     plt::show();

//     plt::figure();
//     plt::plot(tvec, dhvec, "ro-");
//     plt::title("Relative Heading over Time");
//     plt::xlabel("Time [s]");
//     plt::ylabel("Relative Heading [UL]");
//     plt::grid(true);
//     plt::legend();
//     plt::show();

//     // std::cout << "Start Pose: " << std::endl << startPose << std::endl;
//     // std::cout << "End Pose: " << std::endl << endPose << std::endl;
//     // EXPECT_NE(startPose,endPose);
//     // EXPECT_FALSE(endPose.hasNaN()) << "End Pose Has NaN" << std::endl << endPose;

//     // std::cout << "Start Speed: " << startSpeed << std::endl;
//     // std::cout << "End Speed: " << endSpeed << std::endl;
//     // EXPECT_NE(startSpeed,endSpeed);
//     // EXPECT_FALSE(std::isnan(endSpeed)) << "End Speed is NaN" << std::endl << endSpeed;

//     // std::cout << "Start AV: " << startAngVel<< std::endl;
//     // std::cout << "End AV: " << endAngVel << std::endl;
//     // EXPECT_NE(startAngVel,endAngVel);
//     // EXPECT_FALSE(endAngVel.hasNaN()) << "End AngVel Has NaN" << std::endl << endAngVel;

// }

// *****************************************************************************************************
// ******************************************* Sensor Tests ********************************************
// *****************************************************************************************************
TEST_F(SensorTest, ConstructorTest){
    double range = 110;
    Sensor mySensor = Sensor(range);
    EXPECT_EQ(mySensor.GetSensorRange(),range);
}

TEST_F(SensorTest, SetDetectedObjectsTest){
    testSensor.SetDetectedObjects(sensedAgents);
    EXPECT_EQ(testSensor.GetDetectedObjects(), sensedAgents);
}

TEST_F(SensorTest, NoTargetFlagTest){
    std::vector<std::shared_ptr<SimObj>> testVec = {};
    testSensor.SetDetectedObjects(testVec);
    EXPECT_TRUE(testSensor.noTargetInRange);
    testVec.push_back(std::make_shared<SwarmAgent>(SwarmAgent()));
    testSensor.SetDetectedObjects(testVec);
    EXPECT_FALSE(testSensor.noTargetInRange);
}

TEST_F(SensorTest, NewNeighborFlagTest){
    testSensor.SetDetectedObjects(sensedAgents);
    EXPECT_TRUE(testSensor.neighborChange);
    testSensor.SetDetectedObjects(sensedAgents);
    EXPECT_FALSE(testSensor.neighborChange);
}

TEST_F(SensorTest, PoseDataTest){
    testSensor.SetDetectedObjects(sensedAgents);
    testSensor.SamplePoseSensor();
    std::vector<Eigen::Matrix4d> testPoses = testSensor.GetPoseData();
    EXPECT_EQ(testPoses.size(),sensedAgents.size());
    EXPECT_EQ(testPoses[0], sensedAgents[0]->GetCurrentPose());
    EXPECT_EQ(testPoses[1], sensedAgents[1]->GetCurrentPose());
}

TEST_F(SensorTest, PoseRateTest){
    testSensor.SetDetectedObjects(sensedAgents);
    testSensor.SampleRateSensor();
    std::vector<Eigen::Vector4d> testRates = testSensor.GetRateData();
    EXPECT_EQ(testRates.size(),sensedAgents.size());
    // Compare Speeds
    EXPECT_EQ(testRates[0][0], sensedAgents[0]->GetCurrentSpeed());
    EXPECT_EQ(testRates[1][0], sensedAgents[1]->GetCurrentSpeed());
    // Compare Angular Velocities
    Eigen::Vector3d av1 = testRates[0].block<3,1>(1,0);
    Eigen::Vector3d av2 = testRates[1].block<3,1>(1,0);

    EXPECT_EQ(av1, sensedAgents[0]->GetCurrentAngVel());
    EXPECT_EQ(av2, sensedAgents[1]->GetCurrentAngVel());
}

// *****************************************************************************************************
// ***************************************** Environment Tests *****************************************
// *****************************************************************************************************
TEST_F(EnvironmentTest, ConstructorTest){
    EXPECT_EQ(env.numAgents, simAgents.size());
    EXPECT_EQ(env.GetAgentList(), simAgents);
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

    simAgents[0]->SetCurrentPose(t1);
    simAgents[1]->SetCurrentPose(t2);
    simAgents[2]->SetCurrentPose(t3);

    Eigen::MatrixXi testLaplacian;
    testLaplacian.resize(3,3);
    testLaplacian << 1, -1,  0,
                    -1,  2, -1,
                     0, -1,  1;
    
    Eigen::MatrixXi laplacian = env.ComputeLaplacian();
    EXPECT_EQ(env.ComputeLaplacian(),testLaplacian);
    std::vector<std::shared_ptr<SwarmAgent>> neighborhood1 = env.GetNeighborhood(0);
    std::vector<std::shared_ptr<SwarmAgent>> neighborhood2 = env.GetNeighborhood(1);
    std::vector<std::shared_ptr<SwarmAgent>> neighborhood3 = env.GetNeighborhood(2);

    EXPECT_EQ(neighborhood1.size(),1);
    EXPECT_EQ(neighborhood2.size(),2);
    EXPECT_EQ(neighborhood3.size(),1);
    EXPECT_EQ(neighborhood1[0],simAgents[1]);
    EXPECT_EQ(neighborhood2[0],simAgents[0]);
    EXPECT_EQ(neighborhood2[1],simAgents[2]);
    EXPECT_EQ(neighborhood3[0],simAgents[1]);
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

    simAgents[0]->SetCurrentPose(t1);
    simAgents[1]->SetCurrentPose(t2);
    simAgents[2]->SetCurrentPose(t3);

    Eigen::MatrixXi testLaplacian;
    testLaplacian.resize(3,3);
    testLaplacian = Eigen::MatrixXi::Zero(3,3);

    Eigen::MatrixXi laplacian = env.ComputeLaplacian();
    EXPECT_EQ(env.ComputeLaplacian(),testLaplacian);
    std::vector<std::shared_ptr<SwarmAgent>> neighborhood1 = env.GetNeighborhood(0);
    std::vector<std::shared_ptr<SwarmAgent>> neighborhood2 = env.GetNeighborhood(1);
    std::vector<std::shared_ptr<SwarmAgent>> neighborhood3 = env.GetNeighborhood(2);

    EXPECT_EQ(neighborhood1.size(),0);
    EXPECT_EQ(neighborhood2.size(),0);
    EXPECT_EQ(neighborhood3.size(),0);
}

// *****************************************************************************************************
// ***************************************** Integration Tests *****************************************
// *****************************************************************************************************
TEST_F(IntegrationTests, N2_AttractionAPF_Test){
    EXPECT_EQ(numAgents, simAgentList.size());
    // Prepare Sensor
    std::vector<std::shared_ptr<SimObj>> neighborsA1, neighborsA2;
    neighborsA1.push_back(simAgentList[1]);
    simAgentList[0]->sensor.SetDetectedObjects(neighborsA1);
    neighborsA2.push_back(simAgentList[0]);
    simAgentList[1]->sensor.SetDetectedObjects(neighborsA2);

    // std::cout << "Agent 0 Ref'd: " << simAgentList[0].use_count() << std::endl;
    // std::cout << "Agent 1 Ref'd: " << simAgentList[1].use_count() << std::endl;
    
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
    for (uint ti = 0; ti < numSteps; ti++){
        // Run a Simluation Step
        for(uint ai = 0; ai < simAgentList.size(); ++ai){
            // std::cout << ai << std::endl;
            simAgentList[ai] -> Simulate();
            newSpeed = simAgentList[ai] -> GetCurrentSpeed();
            newAngVel = simAgentList[ai] -> GetCurrentAngVel();
            newPose = simAgentList[ai] -> GetCurrentPose();

            // Add New States to History
            speedHist[ti][ai] = newSpeed;
            angvelHist[ti][ai] = newAngVel;
            poseHist[ti][ai] = newPose;

            // Assert No NANs
            EXPECT_FALSE(std::isnan(newSpeed)) << "Agent " << ai << " speed is NaN at index: " << ti << std::endl << newSpeed;
            EXPECT_FALSE(newPose.hasNaN()) << "Agent " << ai << " pose has NaN at index: " << ti << std::endl << newPose;
            EXPECT_FALSE(newAngVel.hasNaN()) << "Agent " << ai << " angvel has NaN at index: " << ti << std::endl << newAngVel;
        }
        
        newDeltaR = (simAgentList[0] -> GetCurrentPosition() - simAgentList[1] -> GetCurrentPosition()).norm();
        a1Att = simAgentList[0] -> GetCurrentAttitude();
        a2Att = simAgentList[1] -> GetCurrentAttitude();
        a1Heading = simAgentList[0] -> GetTangentVec(a1Att);
        a2Heading = simAgentList[1] -> GetTangentVec(a2Att);
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
    // Eigen::Map<Eigen::VectorXd>(drvec.data(), drvec.size()) = relDistHist;
    // Eigen::Map<Eigen::VectorXd>(dhvec.data(), dhvec.size()) = relHeadingHist;

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

    // std::cout << "Start Pose: " << std::endl << startPose << std::endl;
    // std::cout << "End Pose: " << std::endl << endPose << std::endl;
    // EXPECT_NE(startPose,endPose);
    // EXPECT_FALSE(endPose.hasNaN()) << "End Pose Has NaN" << std::endl << endPose;

    // std::cout << "Start Speed: " << startSpeed << std::endl;
    // std::cout << "End Speed: " << endSpeed << std::endl;
    // EXPECT_NE(startSpeed,endSpeed);
    // EXPECT_FALSE(std::isnan(endSpeed)) << "End Speed is NaN" << std::endl << endSpeed;

    // std::cout << "Start AV: " << startAngVel<< std::endl;
    // std::cout << "End AV: " << endAngVel << std::endl;
    // EXPECT_NE(startAngVel,endAngVel);
    // EXPECT_FALSE(endAngVel.hasNaN()) << "End AngVel Has NaN" << std::endl << endAngVel;

}