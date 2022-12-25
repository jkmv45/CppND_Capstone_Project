#include <iostream>
#include <memory>

#include "environmentManager.hpp"
#include "gtest/gtest.h"

// ***************************************** Test Fixtures ****************************************
class SwarmAgentTest : public ::testing::Test{
    protected:
    SwarmAgentTest() {
        uint numAgents = 2;
        double timestep = 0.001;
        this -> agent1 = SwarmAgent(AgentRole::leader,timestep,numAgents);
        this -> agent2 = SwarmAgent(AgentRole::follower,timestep,numAgents);
    }

    // void TearDown() override {}

    // EnvironmentManager env;
    SwarmAgent agent1;
    SwarmAgent agent2;
};

class SensorTest : public ::testing::Test{
    protected:
    SensorTest() {
        uint numAgents = 2;
        double timestep = 0.001;
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
        double timestep = 0.001;
        double range = 100;
        simAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::leader,timestep,numAgents)));
        simAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::follower,timestep,numAgents)));
        simAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::follower,timestep,numAgents)));
        env = EnvironmentManager(simAgents);
    }

    EnvironmentManager env;
    std::vector<std::shared_ptr<SwarmAgent>> simAgents;
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
    EXPECT_EQ(mySpeed,0);
}


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
    Eigen::Matrix4d t1, t2, t3;

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
    Eigen::Matrix4d t1, t2, t3;

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