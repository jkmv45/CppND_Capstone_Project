#pragma once

#include "eigen3/Eigen/Geometry"

// Define Generic Simulation Object Parent Class
class SimObj {
    public:
    virtual void Simulate() = 0;

    // Getters
    Eigen::Matrix4d GetCurrentPose();
    Eigen::Matrix3d GetCurrentAttitude();
    Eigen::Vector3d GetCurrentPosition();
    Eigen::Vector3d GetCurrentAngVel();
    double GetCurrentSpeed();

    // Setters
    void SetCurrentPose(Eigen::Matrix4d);
    void SetCurrentAngVel(Eigen::Vector3d);
    void SetCurrentSpeed(double);

    // Attributes
    protected:
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d angVel = {0, 0, 0};
    double fwdSpd{0};
};