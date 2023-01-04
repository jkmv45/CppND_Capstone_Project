#include "swarmAgent.hpp"

// Constructor
SwarmAgent::SwarmAgent(AgentRole role, double tstep, uint numVeh){
    this -> ctrlParams = ControlParams();
    this -> vehParams = VehicleParams();
    this -> myRole = role;
    this -> dt = tstep;
    // Initialize Private Variables
    this -> inputU = {0, 0, 0, 0};
    this -> pose = Eigen::Matrix4d::Identity();
    this -> angVel = {0, 0, 0};
    this -> fwdSpd = vehParams.cruiseSpeed;
    this -> sensor = Sensor();
    this -> numAgents = numVeh;
    // Compute APF Coefficient b
    double rmin = ctrlParams.eAPF * vehParams.wingSpan;
    this -> ctrlParams.bAPF = ctrlParams.aAPF * exp((rmin - vehParams.wingSpan) * (rmin - vehParams.wingSpan)/ctrlParams.cAPF);
    // Compute Integration Constant to achieve global minimum
    Eigen::Vector2d minAPF = this -> ComputeAPF(rmin);
    this -> ctrlParams.c0APF = minAPF[1];
}

// TODO Add copy constructors and move constructors

// Public Methods
void SwarmAgent::Simulate(){
    ComputeControl();
    PropagateStates();
}

// Getters
double SwarmAgent::GetSensingRange(){
    return vehParams.senseRadius;
}


// Private Methods
void SwarmAgent::ComputeControl(){
    // Poll Sensor for Neighboring Agent Pose and Speed
    std::vector<Eigen::Matrix4d> poseData = sensor.GetPoseData();
    std::vector<double> speedData = sensor.GetSpeedData();

    if (poseData.size() != speedData.size()){
        // Error State TODO
        std::cout << "Error: Sensor pose and speed data are not the same size" << std::endl;
    }

    // Define Nominal Gain
    uint neighborhoodSize = poseData.size();
    double gain0 = (1/((double)numAgents-1))*(1-(neighborhoodSize/((double)numAgents + 1)));
    // Initialize Control Inputs for Summation over Loop
    double u1APF = 0, u1CNS = 0, u1DMP = 0, u2APF = 0, u2CNS = 0, u3APF = 0, u3CNS = 0, u4APF, u4CNS = 0;
    double u1OBJ = 0, u2OBJ = 0, u3OBJ = 0, u4OBJ = 0;
    Eigen::Vector2d resAPF = Eigen::Vector2d::Zero();

    // Define Pose Related Variables
    double relSpeed = 0, relDist = 0;
    Eigen::Vector3d relPos = {0,0,0}, 
                    relPosHat = {0,0,0};
    Eigen::Vector3d tvec  = {1,0,0}, 
                    nvec  = {0,1,0}, 
                    bvec  = {0,0,1}, 
                    tvecj = {1,0,0};
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
        relSpeed = fwdSpd - speedData[j];
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
    
    // std::cout << "Gain0: " << gain0 << " u1APF: " << u1APF << " u2APF: " << u2APF << " u3APF: " << u3APF << " u4APF: " << u4APF << " u1CNS: " << u1CNS << " u2CNS: " << u2CNS << " u3CNS: " << u3CNS << " u4CNS: " << u4CNS << " uDMP: " << u1DMP << std::endl;
    
    double u1, u2, u3, u4 = 0;
    if (myRole == AgentRole::leader){
        // TODO: Define Wapypoint Control Logic
    }

    u1 = u1APF + u1CNS + u1OBJ + u1DMP;
    u2 = u2APF + u2CNS + u2OBJ;
    u3 = u3APF + u3CNS + u3OBJ;
    u4 = u4APF + u4CNS + u4OBJ;

    // TODO: Apply Saturation Logic

    inputU = {u1, u2, u3, u4};
}

void SwarmAgent::PropagateStates(){
    // Get Last Known Values
    Eigen::Matrix3d attitude = GetCurrentAttitude();
    Eigen::Vector3d lastHeading = GetTangentVec(attitude);
    Eigen::Vector3d position = GetCurrentPosition();
    // Setup Integration Variables
    Eigen::Vector3d k1pos = {0,0,0}, 
                    k2pos = {0,0,0}, 
                    k3pos = {0,0,0}, 
                    k4pos = {0,0,0};
    Eigen::Matrix3d k1att = Eigen::Matrix3d::Identity(), 
                    k2att = Eigen::Matrix3d::Identity(), 
                    k3att = Eigen::Matrix3d::Identity(), 
                    k4att = Eigen::Matrix3d::Identity();
    double deltaSpeed, nextSpeed;
    // Get Angular Velocity as Skew Symmetric Matrix
    angVel = inputU.block<3,1>(1,0);
    Eigen::Matrix3d omega = skewSymmetric(angVel);
    // Compute Delta Speed at this step
    deltaSpeed = dt * inputU[0];

    // Perform Runge-Kutta 4th Order Integration on Position and Attitude
    k1pos = dt * fwdSpd * lastHeading;
    k1att = -1.0 * dt * omega * attitude;

    k2pos = dt * (fwdSpd + 0.5 * deltaSpeed) * (lastHeading + 0.5 * GetTangentVec(k1att));
    k2att = -1.0 * dt * omega * (attitude + 0.5 * k1att);

    k3pos = dt * (fwdSpd + 0.5 * deltaSpeed) * (lastHeading + 0.5 * GetTangentVec(k2att));
    k3att = -1.0 * dt * omega * (attitude + 0.5 * k2att);

    k4pos = dt * (fwdSpd + deltaSpeed) * (lastHeading + GetTangentVec(k3att));
    k4att = -1.0 * dt * omega * (attitude + k3att);

    position += (k1pos + 2*k2pos + 2*k3pos + k4pos) / 6.0;
    attitude += (k1att + 2*k2att + 2*k3att + k4att) / 6.0;

    // Update Speed and Pose
    fwdSpd += deltaSpeed;
    SetCurrentPosition(position);
    SetCurrentAttitude(attitude);
}

Eigen::Vector2d SwarmAgent::ComputeAPF(double rrel){
    double g = 0, gIntegral = 0;
    double exponent = (rrel - vehParams.wingSpan)*(rrel - vehParams.wingSpan) / ctrlParams.cAPF;
    // Compute APF Function Value for Control
    g = rrel * (ctrlParams.aAPF - ctrlParams.bAPF * exp(-1.0 * exponent ));
    // Compute Integral for Lyapunov Function (Stability Metric)
    gIntegral = ctrlParams.bAPF * (-0.5 * vehParams.wingSpan * sqrt(M_PI * ctrlParams.cAPF) * erf((vehParams.wingSpan - rrel)/sqrt(ctrlParams.cAPF))
                 - 0.5 * ctrlParams.cAPF * exp(-1.0 * exponent)) + 0.5 * ctrlParams.aAPF * rrel * rrel - ctrlParams.c0APF;
    return {g, gIntegral};
}