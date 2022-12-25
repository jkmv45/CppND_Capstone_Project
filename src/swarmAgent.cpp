#include "swarmAgent.hpp"

// Constructor
SwarmAgent::SwarmAgent(AgentRole role, double tstep, uint numVeh){
    this -> myRole = role;
    this -> dt = tstep;
    // Initialize Private Variables
    // this -> inputU = {0, 0, 0, 0};
    this -> pose = Eigen::Matrix4d::Identity();
    this -> angVel = {0, 0, 0};
    this -> fwdSpd = 0;
    this -> sensor = Sensor(vehParams.senseRadius);
    this -> numAgents = numVeh;
    // Compute APF Coefficient b
    this -> ctrlParams.bAPF = ctrlParams.aAPF * exp((ctrlParams.eAPF * ctrlParams.eAPF * vehParams.wingSpan*vehParams.wingSpan)/ctrlParams.cAPF);
}

// Public Methods
void SwarmAgent::Simulate(){
    ComputeControlInputs();
    PropagateStates();
}

// Getters
// Eigen::Matrix4d SwarmAgent::GetCurrentPose(){
//     Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
//     Eigen::Matrix3d rotm;
//     rotm << tanVecT, normVecN, bnormVecB;
//     pose.topLeftCorner(3,3) = rotm;
//     pose.block<3,1>(0,3) << posVecR.x(), posVecR.y(), posVecR.z();
//     return pose;
// }

// Eigen::Vector3d SwarmAgent::GetCurrentAngVel(){
//     return angVel;
// }

// double SwarmAgent::GetCurrentSpeed(){
//     return fwdSpd;
// }

// Private Methods
// Eigen::Matrix4d SwarmAgent::SenseNeighborPose(SwarmAgent* neighbor){
//     return neighbor->GetCurrentPose();
// }

// Eigen::Vector4d SwarmAgent::SenseNeighborVel(SwarmAgent* neighbor){
//     Eigen::Vector4d vvec;
//     vvec[0] = neighbor->GetCurrentSpeed();
//     vvec.block<1,3>(0,1) << neighbor->GetCurrentAngVel();
//     return vvec;
// }

Eigen::Vector4d SwarmAgent::ComputeControlInputs(){
    // Poll Sensor for Neighboring Agents
    sensor.SamplePoseSensor();
    sensor.SampleRateSensor();
    std::vector<Eigen::Matrix4d> poseData = sensor.GetPoseData();
    std::vector<Eigen::Vector4d> rateData =  sensor.GetRateData();

    if (poseData.size() != rateData.size()){
        // Error State TODO
    }

    // Define Nominal Gain
    uint neighborhoodSize = poseData.size();
    double gain0 = (1/((double)numAgents-1))*(1-(neighborhoodSize/((double)numAgents + 1)));
    // Initialize Control Inputs for Summation over Loop
    double u1APF, u1CNS, u1DMP, u2APF, u2CNS, u3APF, u3CNS, u4APF, u4CNS = 0;
    double u1OBJ, u2OBJ, u3OBJ, u4OBJ = 0;
    Eigen::Vector2d resAPF;

    // Define Pose Related Variables
    double relSpeed, relDist;
    Eigen::Vector3d relPos, relPosHat;
    Eigen::Vector3d tvec, nvec, bvec, tvecj;
    tvec = GetTangentVec(pose);
    nvec = GetNormalVec(pose);
    bvec = GetBinormalVec(pose);
    // Compute Artificial Potential Function (APF) and Consensus Controls
    jFlock = 0;
    for (uint j = 0; j < neighborhoodSize; j++){
        // Compute Relative Position of Agent J to This Agent
        relPos = GetCurrentPosition() - poseData[j].block<3,1>(0,3);
        relDist = relPos.norm();
        relPosHat = relPos/relDist;
        // Compute Speed of Agent J relative to This Agent
        relSpeed = fwdSpd - rateData[j].x();
        // Compute APF and its Derivative Value
        resAPF = ComputeAPF(relDist);
        // Get Neighbor's Heading Vector
        tvecj = GetTangentVec(poseData[j]);

        // Acceleration Control
        u1APF = u1APF - resAPF.x()*relPosHat.dot(tvec);
        u1CNS = u1CNS - relSpeed;

        // Angulary Rate 1 Control (disabled for now)
        u2APF = u2APF - 0;
        u2CNS = u2CNS + 0;

        // Angular Rate 2 Control
        u3APF = u3APF - resAPF.x()*relPosHat.dot(bvec);
        u3CNS = u3CNS + tvecj.dot(bvec);

        // Angular Rate 3 Control
        u4APF = u4APF - resAPF.x()*relPosHat.dot(nvec);
        u4CNS = u4CNS + tvecj.dot(nvec);

        // Lyapunov Function Update for Flocking
        jFlock = jFlock + resAPF.y();
    }

    // Apply Control Gains
    u1APF = gain0 * ctrlParams.gainAPF[0] * u1APF;
    u2APF = gain0 * ctrlParams.gainAPF[1] * u2APF;
    u3APF = gain0 * ctrlParams.gainAPF[2] * u3APF;
    u4APF = gain0 * ctrlParams.gainAPF[3] * u4APF;

    u1CNS = gain0 * ctrlParams.gainCons[0] * u1CNS;
    u2CNS = gain0 * ctrlParams.gainCons[1] * u2CNS;
    u3CNS = gain0 * ctrlParams.gainCons[2] * u3CNS;
    u4CNS = gain0 * ctrlParams.gainCons[3] * u4CNS;

    u1DMP = -1.0 * ctrlParams.damping * (fwdSpd - vehParams.cruiseSpeed);
    
    double u1, u2, u3, u4 = 0;
    if (myRole == AgentRole::leader){
        // TODO: Define Wapypoint Control Logic
    }

    u1 = u1APF + u1CNS + u1OBJ + u1DMP;
    u2 = u2APF + u2CNS + u2OBJ;
    u3 = u3APF + u3CNS + u3OBJ;
    u4 = u4APF + u4CNS + u4OBJ;

    // TODO: Apply Saturation Logic

    Eigen::Vector4d u = {u1, u2, u3, u4};
    return u;
}

void SwarmAgent::PropagateStates(){

}

Eigen::Vector2d SwarmAgent::ComputeAPF(double rrel){
    double g = 0;
    double rDiffSq = (rrel - vehParams.wingSpan)*(rrel - vehParams.wingSpan);
    g = rrel * (ctrlParams.aAPF - ctrlParams.bAPF * exp(-1.0 * rDiffSq / ctrlParams.cAPF));
    // TODO: Derive and compute integral of g(r)
    return {g, 0};
}