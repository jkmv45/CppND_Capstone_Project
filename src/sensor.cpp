#include "sensor.hpp"

// Constructor
Sensor::Sensor(double range){
    this -> sensRange = range;
    this -> poseData.assign(1,Eigen::Matrix4d::Identity());
    this -> rateData.assign(1,Eigen::Vector4d::Zero());
}

void Sensor::SetDetectedObjects(std::vector<std::shared_ptr<SimObj>> objInRange){
    newDataAvailable = (objDet != objInRange);
    objDet.clear();
    objDet = objInRange;
    noTargetInRange = (objInRange.size() == 0);
}

void Sensor::SamplePoseSensor(){
    poseData.clear();
    for (std::shared_ptr<SimObj> obj : objDet){
        poseData.push_back(obj->GetCurrentPose());
    }
}

void Sensor::SampleRateSensor(){
    rateData.clear();
    for (std::shared_ptr<SimObj> obj : objDet){
        Eigen::Vector4d vvec;
        vvec[0] = obj->GetCurrentSpeed();
        vvec.block<3,1>(1,0) << obj->GetCurrentAngVel();
        rateData.push_back(vvec);
    }
}