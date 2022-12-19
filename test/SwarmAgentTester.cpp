#include <iostream>

#include "../header/swarmAgent.hpp"
#include "gtest/gtest.h"

// Test Fixture
class swarmAgentTest : public ::testing::Test{
    protected:
    void SetUp() override {
        double timestep = 0.001;
        // agent1 = swarmAgent(agentRole::leader,timestep,nullptr);
        // agent2 = swarmAgent(agentRole::follower,timestep,nullptr);
        // agent1.myRole = agentRole::leader;
        // agent1.dt = timestep;
        // agent2.myRole = agentRole::follower;
        // agent2.dt = timestep;
    }

    // void TearDown() override {}

    std::shared_ptr<environmentManager> env = std::make_shared<environmentManager>();;
    swarmAgent agent1 = swarmAgent(agentRole::leader,0.01,env); 
    swarmAgent agent2 = swarmAgent(agentRole::follower,0.01,env);
};

// Test Definitions
TEST_F(swarmAgentTest,constructorTest){
    agentRole testrole = agentRole::leader;
    double testval = 0.01;
    swarmAgent myAgent = swarmAgent(testrole, testval, env);
    EXPECT_EQ(myAgent.dt, testval);
    EXPECT_EQ(myAgent.myRole,testrole);
}

TEST_F(swarmAgentTest,getPoseTest){
    Eigen::Matrix4d myPose = agent1.getCurrentPose();
    EXPECT_EQ(myPose,Eigen::Matrix4d::Identity());
}

TEST_F(swarmAgentTest,getAngVelTest){
    Eigen::Vector3d myVel = agent1.getCurrentAngVel();
    EXPECT_EQ(myVel,Eigen::Vector3d::Zero());
}

TEST_F(swarmAgentTest,getSpeed){
    double mySpeed = agent1.getCurrentSpeed();
    EXPECT_EQ(mySpeed,0);
}

// TEST_F(swarmAgentTest,senseNeighborPoseTest){
//     Eigen::Matrix4d myNeighborPose = agent1.senseNeighborPose(&agent2);
//     Eigen::Matrix4d actualPose = agent2.getCurrentPose();
//     EXPECT_EQ(myNeighborPose,actualPose);
// }

// TEST_F(swarmAgentTest,senseNeighborSpeedTest){
//     Eigen::Vector4d myNeighborVel = agent1.senseNeighborVel(&agent2);
//     Eigen::Matrix3d actualAngVel = agent2.getCurrentAngVel();
//     double actualSpeed = agent2.getCurrentSpeed();
//     Eigen::Vector4d actualVel;
//     actualVel << actualSpeed, actualAngVel;
//     EXPECT_EQ(myNeighborVel,actualVel);
// }

int main (int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    
    // Eigen::Matrix4d identityPose;
    // identityPose << 1, 0, 0, 0,
    //                 0, 1, 0, 0,
    //                 0, 0, 1, 0,
    //                 0, 0, 0, 1;

    // Eigen::Vector4d zeroVel = {0, 0, 0, 0};

    // double timestep = 0.001;
    // swarmAgent leader = swarmAgent(agentRole::leader, timestep);
    // swarmAgent follower = swarmAgent(agentRole::follower, timestep);

    // Eigen::Matrix3d rinit = identityPose.topLeftCorner(3,3);
    // std::cout << rinit << std::endl;
    // Eigen::Vector3d v1 = {0, -1, 0};
    // Eigen::Vector3d v2 = {1, 0, 0};
    // Eigen::Vector3d v3 = {0, 0, 1};
    // Eigen::Matrix3d rnew;
    // rnew << v1, v2, v3;
    // identityPose.topLeftCorner(3,3) = rnew;
    // std::cout << identityPose << std::endl;

    // Eigen::Matrix4d leadPose = leader.getCurrentPose();
    // std::cout << leadPose << std::endl;

    return RUN_ALL_TESTS();
}