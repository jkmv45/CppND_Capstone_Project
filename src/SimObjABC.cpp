#include "SimObjABC.hpp"

// *************************************** Getters ***************************************
/**
 * @brief Get current pose of object as 4x4 transformation matrix.
 * 
 * @return Eigen::Matrix4d Pose of object relative to world frame.
 */
Eigen::Matrix4d SimObj::GetCurrentPose(){ return pose; };

/**
 * @brief Get current attitude of object as a 3x3 direction cosine matrix.
 * 
 * @return Eigen::Matrix3d Attitude of object as a matrix relative to world frame.
 */
Eigen::Matrix3d SimObj::GetCurrentAttitude() {
    return pose.block<3,3>(0,0);
}

/**
 * @brief Get current position of object as a 3x1 vector.
 * 
 * @return Eigen::Vector3d Position of object relative to world frame in meters.
 */
Eigen::Vector3d SimObj::GetCurrentPosition() {
    return pose.block<3,1>(0,3);
}

/**
 * @brief Get current angular velocity of object.
 * 
 * @return Eigen::Vector3d Angular Velocity as 3x1 vector in world frame in rad/s.
 */
Eigen::Vector3d SimObj::GetCurrentAngVel(){ return angVel; };

/**
 * @brief Get current forward speed of object.
 * 
 * @return double Forward speed in m/s
 */
double SimObj::GetCurrentSpeed(){ return fwdSpd; };

// *************************************** Helper Functions ***************************************
/**
 * @brief Get position vector from an input pose.
 * 
 * @param inpose (Pass by reference) Eigen matrix 4x4 representing the pose of some object.
 * @return Eigen::Vector3d Position vector of the object described by input pose, units of meters.
 */
Eigen::Vector3d SimObj::GetPositionVec(Eigen::Matrix4d &inpose){ return inpose.block<3,1>(0,3); }

/**
 * @brief Get attitude as a 3x3 rotation matrix from an input pose.
 * 
 * @param inpose (Pass by reference) Eigen matrix 4x4 representing the pose of some object.
 * @return Eigen::Matrix3d Direction Cosine Matrix of the object described by input pose.
 */
Eigen::Matrix3d SimObj::GetAttitudeMatrix(Eigen::Matrix4d &inpose){ return inpose.block<3,3>(0,0); }

/**
 * @brief Get tangent vector of the input attitude.  This represents the X-axis of the object frame.
 * 
 * @param inAtt (Pass by reference) Eigen matrix 3x3 representing the attitude of some object.
 * @return Eigen::Vector3d X-axis direction of the object body frame
 */
Eigen::Vector3d SimObj::GetTangentVec(Eigen::Matrix3d &inAtt){ return inAtt.block<1,3>(0,0); }

/**
 * @brief Get normal vector of the input attitude.  This represents the Y-axis of the object frame.
 * 
 * @param inAtt (Pass by reference) Eigen matrix 3x3 representing the attitude of some object.
 * @return Eigen::Vector3d Y-axis direction of the object body frame
 */
Eigen::Vector3d SimObj::GetNormalVec(Eigen::Matrix3d &inAtt){ return inAtt.block<1,3>(1,0); }

/**
 * @brief Get binormal vector of the input attitude.  This represents the Z-axis of the object frame.
 * 
 * @param inAtt (Pass by reference) Eigen matrix 3x3 representing the attitude of some object.
 * @return Eigen::Vector3d Z-axis direction of the object body frame
 */
Eigen::Vector3d SimObj::GetBinormalVec(Eigen::Matrix3d &inAtt){ return inAtt.block<1,3>(2,0); }

/**
 * @brief Normalize the input direction cosine matrix so that it is orthogonal (i.e. determinant = 1.0).  This uses the singular value decomposition approach: https://math.stackexchange.com/questions/3292034/normalizing-a-rotation-matrix
 * 
 * @param inputMat Eigen 3x3 direction cosine matrix to be normalized.
 */
void SimObj::NormalizeAttMat(Eigen::Matrix3d &inputMat){
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(inputMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d matU = svd.matrixU();
    Eigen::Matrix3d matV = svd.matrixV();
    inputMat = matU*matV.transpose();
}

/**
 * @brief Return the skew symmetric matrix representation of a vector.
 * 
 * @param vec Arbitrary input vector of size 3x1.
 * @return Eigen::Matrix3d Skew Symmetric matrix representation of the input.
 */
Eigen::Matrix3d SimObj::skewSymmetric(Eigen::Vector3d &vec){
    Eigen::Matrix3d result;
    result <<   0,      -vec[2],     vec[1],
                vec[2],  0,         -vec[0],
               -vec[1],  vec[0],     0;
    return result;
}

// *************************************** Setters ***************************************
/**
 * @brief Setter for the current object pose.  Calls setters for position and attitude independently in case the input attitude requires normalization.
 * 
 * @param newPose Eigen 4x4 transformation matrix representing new pose to be set.
 */
void SimObj::SetCurrentPose(Eigen::Matrix4d &newPose){ 
    SetCurrentPosition(GetPositionVec(newPose));
    SetCurrentAttitude(GetAttitudeMatrix(newPose));
}

/**
 * @brief Setter for the current object position.
 * 
 * @param newPos New 3x1 position vector to be set.
 */
void SimObj::SetCurrentPosition(Eigen::Vector3d newPos){
    pose.block<3,1>(0,3) = newPos;
}

/**
 * @brief Setter for the current attitude of the object.  This method will ensure that the input matrix is well formed before updating.  
 *        If the matrix determinant is outside the lower tolerance but within the upper tolerance, the matrix will be normalized.
 *        If the matrix determinant is inside the lower tolerance, no correction will be made.
 *        If the matrix determinant is outside the upper limit, the input will be rejected as an invalid transformation matrix.
 * 
 * @param newAtt New direction cosine matrix to be set.
 */
void SimObj::SetCurrentAttitude(Eigen::Matrix3d newAtt){
    // Check if Attitude Matrix needs to be Normalized and is Right Handed
    double detVal = newAtt.determinant();
    double orthoCheck = abs(detVal - 1);
    if ( orthoCheck > unitLTOL && orthoCheck < unitUTOL && detVal > 0){ 
        NormalizeAttMat(newAtt); 
    } else if (orthoCheck > unitUTOL){ 
        return; // Invalid Matrix
    } else { ; } // no-op
    pose.block<3,3>(0,0) = newAtt;
}

/**
 * @brief Setter for current angular velocity in rad/s.
 * 
 * @param newVel New 3x1 angular velocity vector to be set.
 */
void SimObj::SetCurrentAngVel(Eigen::Vector3d &newVel){ 
    angVel = newVel; 
}

/**
 * @brief Setter for current forward speed in m/s.  Negative speeds are rejected.
 * 
 * @param newSpeed New speed to be set.
 */
void SimObj::SetCurrentSpeed(double &newSpeed){ 
    if (newSpeed < 0){
        return; // Speed cannot be negative
    } else {
        fwdSpd = newSpeed; 
    }
}