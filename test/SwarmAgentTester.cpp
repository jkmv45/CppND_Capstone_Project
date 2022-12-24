#include <iostream>
#include <memory>

#include "environmentManager.hpp"
#include "gtest/gtest.h"

// Test Fixture
class SwarmAgentTest : public ::testing::Test{
    protected:
    SwarmAgentTest() {
        double timestep = 0.001;
        this -> agent1 = SwarmAgent(AgentRole::leader,timestep);
        this -> agent2 = SwarmAgent(AgentRole::follower,timestep);
    }

    // void TearDown() override {}

    EnvironmentManager env;
    SwarmAgent agent1;
    SwarmAgent agent2;
};

class SensorTest : public ::testing::Test{
    protected:
    SensorTest() {
        double timestep = 0.001;
        double range = 100;
        // SwarmAgent agent1 = SwarmAgent(AgentRole::leader,timestep);
        // SwarmAgent agent2 = SwarmAgent(AgentRole::follower,timestep);
        sensedAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::leader,timestep)));
        sensedAgents.push_back(std::make_shared<SwarmAgent>(SwarmAgent(AgentRole::leader,timestep)));
        this -> testSensor = Sensor(range);
    }

    // void TearDown() override {}

    Sensor testSensor;
    std::vector<std::shared_ptr<SimObj>> sensedAgents;
};

// Test Definitions
TEST_F(SwarmAgentTest,ConstructorTest){
    AgentRole testrole = AgentRole::leader;
    double testval = 0.01;
    SwarmAgent myAgent = SwarmAgent(testrole, testval);
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

TEST_F(SensorTest, NewDataFlagTest){
    testSensor.SetDetectedObjects(sensedAgents);
    EXPECT_TRUE(testSensor.newDataAvailable);
    testSensor.SetDetectedObjects(sensedAgents);
    EXPECT_FALSE(testSensor.newDataAvailable);
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

// TEST_F(SwarmAgentTest,senseNeighborPoseTest){
//     Eigen::Matrix4d myNeighborPose = agent1.senseNeighborPose(&agent2);
//     Eigen::Matrix4d actualPose = agent2.getCurrentPose();
//     EXPECT_EQ(myNeighborPose,actualPose);
// }

// TEST_F(SwarmAgentTest,senseNeighborSpeedTest){
//     Eigen::Vector4d myNeighborVel = agent1.senseNeighborVel(&agent2);
//     Eigen::Matrix3d actualAngVel = agent2.getCurrentAngVel();
//     double actualSpeed = agent2.getCurrentSpeed();
//     Eigen::Vector4d actualVel;
//     actualVel << actualSpeed, actualAngVel;
//     EXPECT_EQ(myNeighborVel,actualVel);
// }