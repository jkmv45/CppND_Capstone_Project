#pragma once

// Define Generic Simulation Object Parent Class
class SimObj {
    public:
    virtual void Simulate() = 0;

    Eigen::Matrix4d GetCurrentPose(){ return pose; };
    Eigen::Vector3d GetCurrentAngVel(){ return angVel; };
    double GetCurrentSpeed(){ return fwdSpd; };

    // Attributes
    private:
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d angVel = {0, 0, 0};
    double fwdSpd{0};
};