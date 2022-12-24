#pragma once

// Define Generic Simulation Object Parent Class
class SimObj {
    public:
    virtual void Simulate() = 0;

    // Getters
    Eigen::Matrix4d GetCurrentPose(){ return pose; };
    Eigen::Vector3d GetCurrentAngVel(){ return angVel; };
    double GetCurrentSpeed(){ return fwdSpd; };

    // Setters
    void SetCurrentPose(Eigen::Matrix4d newPose){ pose = newPose; }
    void SetCurrentAngVel(Eigen::Vector4d newVel){ angVel = newVel; }
    void SetCurrentSpeed(double newSpeed){ fwdSpd = newSpeed; }

    // Attributes
    protected:
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d angVel = {0, 0, 0};
    double fwdSpd{0};
};