#pragma once

#include <iostream>

#include "eigen3/Eigen/Geometry"

// Attitude Matrix Determinant Thresholds 
#define unitLTOL 1e-6   //  Under this tolerance, no normalization is needed
#define unitUTOL 1e-3   // Above this tolerance, attitude matrix is invalid

// Define Generic Simulation Object Parent Class
class SimObj {
    public:
    virtual void Simulate() = 0;

     // Methods
    Eigen::Matrix3d skewSymmetric(Eigen::Vector3d);

    // Getters
    Eigen::Matrix4d GetCurrentPose();
    Eigen::Matrix3d GetCurrentAttitude();
    Eigen::Vector3d GetCurrentPosition();
    Eigen::Vector3d GetCurrentAngVel();
    double GetCurrentSpeed();
    // Helper Functions
    Eigen::Vector3d GetPositionVec(Eigen::Matrix4d&);
    Eigen::Matrix3d GetAttitudeMatrix(Eigen::Matrix4d&);
    Eigen::Vector3d GetTangentVec(Eigen::Matrix3d&);
    Eigen::Vector3d GetNormalVec(Eigen::Matrix3d&);
    Eigen::Vector3d GetBinormalVec(Eigen::Matrix3d&);
    Eigen::Matrix3d NormalizeAttMat(Eigen::Matrix3d);

    // Setters
    void SetCurrentPose(Eigen::Matrix4d);
    void SetCurrentPosition(Eigen::Vector3d);
    void SetCurrentAttitude(Eigen::Matrix3d);
    void SetCurrentAngVel(Eigen::Vector3d);
    void SetCurrentSpeed(double);

    // Attributes
    protected:
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d angVel = {0, 0, 0};
    double fwdSpd{0};
};