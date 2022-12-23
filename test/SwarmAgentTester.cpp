#include <iostream>

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