#include "swarmAgent.hpp"

// Constructor
SwarmAgent::SwarmAgent(AgentRole role, double tstep, uint numVeh){
    this -> ctrlParams = ControlParams();
    this -> vehParams = VehicleParams();
    this -> myRole = role;
    this -> dt = tstep;
    // Initialize Private Variables
    // this -> inputU = {0, 0, 0, 0};
    this -> pose = Eigen::Matrix4d::Identity();
    this -> angVel = {0, 0, 0};
    this -> fwdSpd = vehParams.cruiseSpeed;
    this -> sensor = Sensor(vehParams.senseRadius);
    this -> numAgents = numVeh;
    // Compute APF Coefficient b
    double rmin = ctrlParams.eAPF * vehParams.wingSpan;
    this -> ctrlParams.bAPF = ctrlParams.aAPF * exp((rmin - vehParams.wingSpan) * (rmin - vehParams.wingSpan)/ctrlParams.cAPF);
    // Compute Integration Constant to achieve global minimum
    Eigen::Vector2d minAPF = this -> ComputeAPF(rmin);
    this -> ctrlParams.c0APF = minAPF[1];
}

// Public Methods
void SwarmAgent::Simulate(){
    // std::cout << "Step 1" << std::endl;
    ComputeControlInputs();
    // std::cout << "Step 2" << std::endl;
    PropagateStates();
    // std::cout << "Step 3" << std::endl;
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

void SwarmAgent::ComputeControlInputs(){
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
    Eigen::Vector2d resAPF = Eigen::Vector2d::Zero();

    // Define Pose Related Variables
    double relSpeed, relDist;
    Eigen::Vector3d relPos, relPosHat;
    Eigen::Vector3d tvec, nvec, bvec, tvecj;
    Eigen::Matrix3d dcmNeighbor;
    Eigen::Matrix3d dcm = GetCurrentAttitude();
    tvec = GetTangentVec(dcm);
    nvec = GetNormalVec(dcm);
    bvec = GetBinormalVec(dcm);
    // Compute Artificial Potential Function (APF) and Consensus Controls
    jFlock = 0;
    for (uint j = 0; j < neighborhoodSize; j++){
        // Get Neighbor Attitude
        dcmNeighbor = GetAttitudeMatrix(poseData[j]);
        // Compute Relative Position of Agent J to This Agent
        relPos = GetCurrentPosition() - poseData[j].block<3,1>(0,3);
        relDist = relPos.norm();
        relPosHat = relPos/relDist;
        // Compute Speed of Agent J relative to This Agent
        relSpeed = fwdSpd - rateData[j].x();
        // Compute APF and its Derivative Value
        resAPF = ComputeAPF(relDist);
        // Get Neighbor's Heading Vector
        tvecj = GetTangentVec(dcmNeighbor);

        // Acceleration Control
        u1APF = u1APF - resAPF.x()*relPosHat.dot(tvec);
        u1CNS = u1CNS - relSpeed;

        // Angulary Rate 1 Control (disabled for now)
        u2APF = u2APF - 0;
        u2CNS = u2CNS + 0;

        // Angular Rate 2 Control
        u3APF = u3APF + resAPF.x()*relPosHat.dot(bvec);
        u3CNS = u3CNS - tvecj.dot(bvec);

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

    // Eigen::Vector4d u = {u1, u2, u3, u4};
    inputU = {u1, u2, u3, u4};
    // return u;
}

void SwarmAgent::PropagateStates(){
    // Get Last Known Values
    Eigen::Matrix3d lastAttitude = GetCurrentAttitude();
    Eigen::Vector3d lastHeading = GetTangentVec(lastAttitude);
    Eigen::Vector3d lastPosition = GetCurrentPosition();
    // Setup Integration Variables
    Eigen::Vector3d k1pos, k2pos, k3pos, k4pos, nextPosition;
    Eigen::Matrix3d k1att, k2att, k3att, k4att, nextAttitude;
    double deltaSpeed, nextSpeed;
    // Get Angular Velocity as Skew Symmetric Matrix
    angVel = inputU.block<3,1>(1,0);
    Eigen::Matrix3d omega = skewSymmetric(angVel);
    // Compute Delta Speed at this step
    deltaSpeed = dt * inputU[0];

    // Perform Runge-Kutta 4th Order Integration on Position and Attitude
    k1pos = dt * fwdSpd * lastHeading;
    k1att = -1.0 * dt * omega * lastAttitude;

    k2pos = dt * (fwdSpd + 0.5 * deltaSpeed) * (lastHeading + 0.5 * GetTangentVec(k1att));
    k2att = -1.0 * dt * omega * (lastAttitude + 0.5 * k1att);

    k3pos = dt * (fwdSpd + 0.5 * deltaSpeed) * (lastHeading + 0.5 * GetTangentVec(k2att));
    k3att = -1.0 * dt * omega * (lastAttitude + 0.5 * k2att);

    k4pos = dt * (fwdSpd + deltaSpeed) * (lastHeading + GetTangentVec(k3att));
    k4att = -1.0 * dt * omega * (lastAttitude + k3att);

    nextPosition = lastPosition + (k1pos + 2*k2pos + 2*k3pos + k4pos) / 6.0;
    nextAttitude = lastAttitude + (k1att + 2*k2att + 2*k3att + k4att) / 6.0;
    
    // Normalize Attitude Matrix
    Eigen::Vector3d tvec, nvec, bvec;
    double tvecNorm, nvecNorm, bvecNorm = 1;
    tvec = GetTangentVec(nextAttitude);
    nvec = GetNormalVec(nextAttitude);
    bvec = GetBinormalVec(nextAttitude);
    
    tvec = tvec / tvec.norm();
    nvec = nvec / nvec.norm();
    bvec = bvec / bvec.norm();

    nextAttitude << tvec, nvec, bvec;

    // Update Speed and Pose
    fwdSpd += deltaSpeed;
    pose.block<3,1>(0,3) = nextPosition;
    pose.block<3,3>(0,0) = nextAttitude;
}

Eigen::Vector2d SwarmAgent::ComputeAPF(double rrel){
    double g, gIntegral = 0;
    double exponent = (rrel - vehParams.wingSpan)*(rrel - vehParams.wingSpan) / ctrlParams.cAPF;
    // Compute APF Function Value for Control
    g = rrel * (ctrlParams.aAPF - ctrlParams.bAPF * exp(-1.0 * exponent ));
    // Compute Integral for Lyapunov Function (Stability Metric)
    gIntegral = ctrlParams.bAPF * (-0.5 * vehParams.wingSpan * sqrt(M_PI * ctrlParams.cAPF) * erf((vehParams.wingSpan - rrel)/sqrt(ctrlParams.cAPF))
                 - 0.5 * ctrlParams.cAPF * exp(-1.0 * exponent)) + 0.5 * ctrlParams.aAPF * rrel * rrel - ctrlParams.c0APF;
    return {g, gIntegral};
}