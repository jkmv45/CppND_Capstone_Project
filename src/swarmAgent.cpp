#include "swarmAgent.hpp"

// *************************************** Constructor ***************************************
/**
 * @brief Construct a new Swarm Agent:: Swarm Agent object
 * 
 * @param role Role of the agents. It can be either a leader or a follower
 * @param tstep Simulation time step in seconds. This is used in the integration scheme.
 * @param numVeh Number of agents participating in the swarm.  This is assumed as apriori knowledge to aid in control scaling.
 */
SwarmAgent::SwarmAgent(AgentRole role, double tstep, uint numVeh){
    // Initialize Object Properties
    this -> ctrlParams = ControlParams();
    this -> vehParams = VehicleParams();
    this -> myRole = role;
    this -> dt = tstep;
    // Initialize Private Variables
    this -> outputY = {0, 0, 0, 0};
    this -> pose = Eigen::Matrix4d::Identity();
    this -> angVel = {0, 0, 0};
    this -> fwdSpd = vehParams.cruiseSpeed;
    this -> sensor = Sensor();
    this -> numAgents = numVeh;
    // Compute APF Coefficient b (depends on vehicle params and other control params to ensure proper scaling)
    double rmin = ctrlParams.eAPF * vehParams.wingSpan;
    this -> ctrlParams.bAPF = ctrlParams.aAPF * exp((rmin - vehParams.wingSpan) * (rmin - vehParams.wingSpan)/ctrlParams.cAPF);
    // Compute Integration Constant to achieve global minimum (this is useful if the Lyapunov Function value is calculated)
    Eigen::Vector2d minAPF = this -> ComputeAPF(rmin);
    this -> ctrlParams.c0APF = minAPF[1];
}

// *************************************** Public Methods ***************************************
/**
 * @brief Simulate the agent dynamics. First the controller output is calculated and then the states of the agent are propagated forward in time using an integration scheme.
 * 
 */
void SwarmAgent::Simulate(){
    ComputeControl();
    PropagateStates();
}

// *************************************** Getters ***************************************
/**
 * @brief Return the agent's sensing range.
 * 
 * @return double Sensing radius of the agent's sensor in meters
 */
double SwarmAgent::GetSensingRange(){
    return vehParams.senseRadius;
}

// *************************************** Private Methods ***************************************
/**
 * @brief Compute the acceleration and angular rates required by this agent to achieve and maintain swarm behavior.  Several control objectives are sought by this control law:
 *          1) Collision Avoidance (uxAPF): The artificial potential function (APF) contains a repulsive action when the agent is too close to another agent.
 *          2) Separation Distance (uxAPF): The APF contains an attraction action when the agent is too far away from another agent.  The goal is to maintain a separation distance without collision.
 *          3) Speed Consensus (u1CNS): The agent will try to match speeds with its neighboring agents until all agents are heading at roughly the same speed.
 *          4) Speed Control (u1DMP): The agent will try to maintain its specified cruise speed.  This is valid mainly for fixed-wing aircraft.
 *          5) Heading Consensus (u3CNS and u4CNS): The agent will try to match heading direction with its neighboring agents.  The control laws here are a bit more complex, 
 *             but basically the idea is to minimize the dot product between a neighbor's tangent vector and this agents normal and binormal vectors by controlling the "yaw" and "pitch" rates.
 *             The rates being controlled are not exactly yaw and pitch, but have effectively the same result in this context.
 *          6) Objective Tracking (uxOBJ): (TODO:Future Work) In leader-follower schemes, some agents will have knowledge of some objective (e.g. waypoints) that must be achieved.
 *             In this case, those agents will have an additional attractive APF to drive it closer to the target.
 *        In a realistic system, output saturation (TODO:Future Work) is applied to ensure the control actions do not exceed the physical constraints of the vehicle.  There are a few ways this can be applied.
 *        Interested readers should refer to the thesis from which this work was based for additional information on saturation schemes.
 * 
 *        Units: m/s^2 and rad/s
 * 
 */
void SwarmAgent::ComputeControl(){
    // Poll Sensor for Neighboring Agent Pose and Speed
    std::vector<Eigen::Matrix4d> poseData = sensor.GetPoseData();
    std::vector<double> speedData = sensor.GetSpeedData();

    if (poseData.size() != speedData.size()){
        std::cout << "Error: Sensor pose and speed data are not the same size" << std::endl;
        return;
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
        
    double u1, u2, u3, u4 = 0;
    if (myRole == AgentRole::leader){
        /** TODO: Future Work: Define Wapypoint Control Logic */
    }

    u1 = u1APF + u1CNS + u1OBJ + u1DMP;
    u2 = u2APF + u2CNS + u2OBJ;
    u3 = u3APF + u3CNS + u3OBJ;
    u4 = u4APF + u4CNS + u4OBJ;

    /** TODO: Future Work: Apply Saturation Logic */

    outputY = {u1, u2, u3, u4};
}

/**
 * @brief Propagate the pose of the agent using a Runge-Kutta 4th Order numerical integration scheme.  In this simulation, speed is actually integrated through a backward Euler scheme due to the simplified dynamic model.
 *        NOTE: The agent is simulated as a point mass with attitude kinematics minus the "roll" axis rate components.
 * 
 *        Units: Speed, m/s, Position, m, Attitude, unitless
 * 
 */
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
    angVel = outputY.block<3,1>(1,0);
    Eigen::Matrix3d omega = skewSymmetric(angVel);
    // Compute Delta Speed at this step
    deltaSpeed = dt * outputY[0];

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

/**
 * @brief Compute the gradient of an artificial potential function (APF) and the APF value.  This particular APF has an exponential repulsion and linear attraction.
 *        The previous thesis work showed this to be the best performing function.  Currently the APF value is not used, but if the system Lyapunov function is desired,
 *        that value will be useful.  The gradient is the only value used for real-time control.
 * 
 * @param rrel Relative distance between two objects
 * @return Eigen::Vector2d Eigen vector to store the two function results: 
 *          idx0 is the APF gradient value.  
 *          idx1 is the APF value (i.e. artificial potential energy).
 */
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